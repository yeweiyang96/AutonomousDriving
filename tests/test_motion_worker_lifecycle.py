"""Control-thread deadlines must not include Windows process startup/reaping."""
from collections import deque
import json
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from adrive.models import PlanningError
from adrive.motion_worker import MotionWorker, _shutdown_workers


class Connection:
    def __init__(self):
        self.messages=deque();self.sent=[];self.closed=False

    def poll(self):return bool(self.messages)
    def recv(self):return self.messages.popleft()
    def send(self,message):self.sent.append(message)
    def close(self):self.closed=True


class Process:
    def __init__(self):
        self.start_entered=threading.Event();self.start_release=threading.Event()
        self.join_entered=threading.Event();self.join_release=threading.Event()
        self.start_release.set();self.join_release.set()
        self.pid=None;self.alive=False;self.closed=False;self.terminated=False;self.error=None

    def start(self):
        self.start_entered.set()
        if not self.start_release.wait(2):raise RuntimeError('test startup gate timed out')
        if self.error:raise self.error
        self.pid=123;self.alive=True

    def is_alive(self):return self.alive
    def terminate(self):self.terminated=True;self.alive=False
    def kill(self):self.alive=False
    def join(self,timeout=None):
        self.join_entered.set()
        if not self.join_release.wait(2):raise RuntimeError('test reap gate timed out')
    def close(self):self.closed=True


class MotionWorkerLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.connection=Connection();self.child=Connection();self.process=Process()
        context=SimpleNamespace(Pipe=lambda:(self.connection,self.child),Process=lambda **kw:self.process)
        self.context_patch=patch('adrive.motion_worker.mp.get_context',return_value=context)
        self.path_patch=patch('adrive.motion_worker.Path')
        self.context_patch.start();path=self.path_patch.start()
        path.return_value.read_bytes.return_value=json.dumps({'identity':'pack','library':{'sha256':'dll'}}).encode()
        self.addCleanup(self.context_patch.stop);self.addCleanup(self.path_patch.stop)
        self.workers=[];self.addCleanup(self.cleanup_workers)

    def cleanup_workers(self):
        self.process.start_release.set();self.process.join_release.set()
        for worker in self.workers:
            worker.close()
            self.assertTrue(worker._finished.wait(2),'worker owner did not finish')

    def create(self):
        worker=MotionWorker(None,None,(),'/fixture/manifest.json',1)
        self.workers.append(worker)
        return worker

    def started(self,worker):
        self.assertTrue(worker._started.wait(2),'worker owner did not start')

    def test_slow_start_does_not_block_constructor_or_poll(self):
        self.process.start_release.clear()
        worker=self.create()
        self.assertTrue(self.process.start_entered.wait(2))
        self.assertFalse(worker._started.is_set())
        self.assertIsNone(worker.poll())
        self.assertFalse(worker.ready)
        t=SimpleNamespace(session=1,seq=1)
        self.assertFalse(worker.submit(t,()))
        self.assertFalse(worker.probe(t,1))
        self.process.start_release.set();self.started(worker)
        # Process creation alone is not permission to calculate movement.
        self.assertIsNone(worker.poll());self.assertFalse(worker.ready)

    def test_cancel_during_start_reaps_late_process_without_publishing(self):
        self.process.start_release.clear()
        worker=self.create();self.assertTrue(self.process.start_entered.wait(2))
        worker.close()
        self.assertIsNone(worker.poll());self.assertFalse(worker.ready)
        self.process.start_release.set()
        self.assertTrue(worker._finished.wait(2))
        self.assertTrue(self.process.terminated);self.assertTrue(self.process.closed)
        self.assertTrue(self.connection.closed);self.assertTrue(self.child.closed)
        self.assertIsNone(worker.process);self.assertIsNone(worker.connection)

    def test_start_failure_is_reported_by_poll_and_pipes_are_closed(self):
        self.process.error=OSError('spawn failure')
        worker=self.create();self.assertTrue(worker._finished.wait(2))
        with self.assertRaisesRegex(PlanningError,'spawn failure'):worker.poll()
        self.assertFalse(worker.ready)
        self.assertTrue(self.connection.closed);self.assertTrue(self.child.closed)

    def test_manifest_read_failure_is_reported_without_spawning(self):
        with patch('adrive.motion_worker.Path') as path:
            path.return_value.read_bytes.side_effect=OSError('manifest failure')
            worker=self.create();self.assertTrue(worker._finished.wait(2))
        with self.assertRaisesRegex(PlanningError,'manifest failure'):worker.poll()
        self.assertFalse(self.process.start_entered.is_set())

    def test_ready_requires_the_exact_expected_package(self):
        worker=self.create();self.started(worker)
        self.connection.messages.append(('ready','other-pack','dll'))
        with self.assertRaisesRegex(PlanningError,'导航包不匹配'):worker.poll()
        self.assertFalse(worker.ready)
        self.connection.messages.append(('ready','pack','dll'))
        worker.poll();self.assertTrue(worker.ready)
        t=SimpleNamespace(session=1,seq=1)
        self.assertTrue(worker.submit(t,(26,)))
        self.assertFalse(worker.submit(t,(26,)))
        self.connection.messages.append(('result','window',1.5))
        self.assertEqual(worker.poll(),('window',1.5))
        self.assertFalse(worker.pending)

    def test_close_returns_before_reaping_and_cannot_return_late_results(self):
        worker=self.create();self.started(worker)
        self.connection.messages.append(('ready','pack','dll'));worker.poll()
        self.process.join_release.clear()
        start=time.monotonic();worker.close()
        self.assertLess(time.monotonic()-start,.05)
        self.assertTrue(self.process.join_entered.wait(2))
        self.assertFalse(worker._finished.is_set())
        self.connection.messages.append(('result','stale-window',1.5))
        self.assertIsNone(worker.poll());self.assertFalse(worker.ready)
        t=SimpleNamespace(session=1,seq=2)
        self.assertFalse(worker.submit(t,(26,)));self.assertFalse(worker.probe(t,1))
        self.process.join_release.set()

    def test_busy_lifecycle_lock_does_not_block_control_thread(self):
        worker=self.create();self.started(worker)
        self.connection.messages.append(('ready','pack','dll'));worker.poll()
        with worker._io_lock:
            self.assertIsNone(worker.poll())
            self.assertFalse(worker.submit(SimpleNamespace(session=1,seq=1),(26,)))
            self.assertFalse(worker.probe(None,1))

    def test_interpreter_cleanup_finishes_owners_before_multiprocessing_cleanup(self):
        worker=self.create();self.started(worker)
        _shutdown_workers()
        self.assertTrue(worker._finished.is_set())
        self.assertTrue(self.process.closed);self.assertTrue(self.connection.closed)


if __name__=='__main__':unittest.main()
