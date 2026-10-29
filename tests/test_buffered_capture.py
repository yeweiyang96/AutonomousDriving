"""Thread-isolation tests: no actual screen, window, keyboard or Pico access."""
import json
from pathlib import Path
import queue
import tempfile
import threading
import time
import unittest

from adrive.capture import BufferedCapture
from adrive.controller import Calibration
from adrive.eventlog import json_value
from adrive.models import Flags, Point, Route, Segment, Telemetry
from adrive.runtime import Runtime
from adrive.settings import ROOT, Settings
from adrive.telemetry import FreshnessTracker
from tests.test_continuous import FakeLease
from tests.test_runtime import FakePico


def wait_until(predicate, seconds=2):
    deadline = time.monotonic() + seconds
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError('fixture condition timed out')
        time.sleep(.001)


class FocusBackend:
    active = 33
    identity_value = (900, 'World of Warcraft')

    def foreground(self):
        return self.active

    def valid(self, hwnd):
        return hwnd == 33

    def identity(self, hwnd):
        return self.identity_value


class ControlledSource:
    """Block a grab until the test provides a frame; ownership is observable."""
    def __init__(self):
        self.tokens = queue.Queue()
        self.calls = 0
        self.status = 'fixture capture'
        self.phase = 'idle'
        self.last_timings = {}
        self.window_binding = (33, (900, 'World of Warcraft'))
        self.owner = None
        self.closed_by = None
        self.closed_in_poll = None
        self.in_poll = False

    def factory(self, **kw):
        self.owner = threading.get_ident()
        return self

    def poll(self):
        self.calls += 1
        self.in_poll = True
        self.phase = 'grab'
        captured = time.monotonic()
        try:
            token = self.tokens.get(timeout=3)
            if token is None:
                return None
            if isinstance(token, Exception):
                raise token
            seq, timestamp = token
            return Telemetry(9, seq, 0, 1429, 0., 0., 0., 7., Flags.VALID,
                             captured if timestamp is None else timestamp)
        finally:
            self.in_poll = False
            self.phase = 'idle'
            self.last_timings = {'grab': (time.monotonic() - captured) * 1000}

    def close(self):
        self.closed_by = threading.get_ident()
        self.closed_in_poll = self.in_poll


class BufferedCaptureTests(unittest.TestCase):
    def make_capture(self, source=None):
        source = source or ControlledSource()
        focus = FocusBackend()
        capture = BufferedCapture(capture_factory=source.factory, _focus_backend=focus,
                                  interval=.001, close_timeout=.01)

        def cleanup():
            capture.close()
            source.tokens.put(None)
            capture._worker_closed.wait(2)

        self.addCleanup(cleanup)
        return capture, source, focus

    def test_blocked_capture_cannot_block_poll_or_diagnostic_reads(self):
        capture, source, _ = self.make_capture()
        wait_until(lambda: source.in_poll)
        finished = threading.Event()
        results = []

        def reader():
            for _ in range(100):
                results.append(capture.poll())
                results.append(capture.diagnostics()['busy'])
            finished.set()

        thread = threading.Thread(target=reader)
        thread.start()
        # The grab remains blocked until cleanup. This tests the dependency,
        # not a benchmark threshold that would depend on machine load.
        self.assertTrue(finished.wait(1))
        thread.join()
        self.assertTrue(source.in_poll)
        self.assertTrue(all(value is None for value in results[::2]))
        self.assertTrue(all(value for value in results[1::2]))
        self.assertEqual(capture.diagnostics()['phase'], 'grab')

    def test_only_latest_completed_frame_is_delivered_once(self):
        capture, source, _ = self.make_capture()
        for seq in range(1, 4):
            source.tokens.put((seq, 100 + seq / 100))
            wait_until(lambda: capture.diagnostics()['generation'] >= seq)
        latest = capture.poll()
        self.assertEqual(latest.seq, 3)
        self.assertIsNone(capture.poll())

    def test_delayed_delivery_keeps_acquisition_time_and_is_not_fresh(self):
        capture, source, _ = self.make_capture()
        wait_until(lambda: source.in_poll)
        source.tokens.put((1, 100.))
        wait_until(lambda: capture.diagnostics()['generation'] == 1)
        t = capture.poll()
        self.assertEqual(t.captured_at, 100.)
        tracker = FreshnessTracker()
        self.assertTrue(tracker.accept(t, 100.4))
        self.assertFalse(tracker.fresh(100.4))

    def test_duplicate_frames_do_not_refresh_time(self):
        capture, source, _ = self.make_capture()
        tracker = FreshnessTracker()
        source.tokens.put((1, 100.))
        wait_until(lambda: capture.diagnostics()['generation'] == 1)
        self.assertTrue(tracker.accept(capture.poll(), 100.))
        source.tokens.put((1, 100.2))
        wait_until(lambda: capture.diagnostics()['generation'] == 2)
        self.assertFalse(tracker.accept(capture.poll(), 100.2))
        self.assertFalse(tracker.fresh(100.3))

    def test_focus_is_checked_live_while_next_grab_is_blocked(self):
        source = ControlledSource()
        source.tokens.put((1, None))
        capture, source, focus = self.make_capture(source)
        wait_until(lambda: capture.diagnostics()['generation'] == 1 and source.calls == 2)
        self.assertTrue(capture.foreground())
        focus.active = 22
        self.assertFalse(capture.foreground())
        focus.active = 33
        focus.identity_value = (901, 'Text Editor')
        self.assertFalse(capture.foreground())
        self.assertTrue(source.in_poll)

    def test_close_returns_before_grab_and_owner_later_closes_source(self):
        capture, source, _ = self.make_capture()
        wait_until(lambda: source.in_poll)
        close_done = threading.Event()
        thread = threading.Thread(target=lambda: (capture.close(), close_done.set()))
        thread.start()
        self.assertTrue(close_done.wait(1))
        thread.join()
        self.assertIsNone(source.closed_by)
        self.assertIsNone(capture.poll())
        self.assertFalse(capture.foreground())
        source.tokens.put((1, None))
        self.assertTrue(capture._worker_closed.wait(1))
        self.assertEqual(source.closed_by, source.owner)
        self.assertFalse(source.closed_in_poll)
        self.assertIsNone(capture.poll())  # Closing never publishes a late frame.

    def test_focus_change_during_identity_check_is_not_authorized(self):
        source = ControlledSource()
        source.tokens.put((1, None))
        capture, source, focus = self.make_capture(source)
        wait_until(lambda: capture.diagnostics()['generation'] == 1)

        def changing_identity(hwnd):
            focus.active = 22
            return focus.identity_value

        focus.identity = changing_identity
        self.assertFalse(capture.foreground())

    def test_capture_error_replaces_latest_slot_without_fabricated_frame(self):
        capture, source, _ = self.make_capture()
        source.tokens.put((1, None))
        wait_until(lambda: capture.diagnostics()['generation'] == 1)
        source.tokens.put(RuntimeError('fixture failure'))
        wait_until(lambda: capture.diagnostics()['generation'] == 2)
        self.assertIsNone(capture.poll())
        self.assertIn('fixture failure', capture.status)


class ProbeLease(FakeLease):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.poll_count = 0

    def poll(self, now):
        self.poll_count += 1
        return super().poll(now)


class DirectPlanner:
    def __init__(self, bundle):
        pass

    def plan(self, start, goal, **kw):
        return Route(start, goal, (Segment((start, goal), 'fixture', 'reviewed', 'edge'),))


class CaptureRuntimeIsolationTests(unittest.TestCase):
    def test_control_keeps_polling_and_stops_stale_without_renewing_frozen_input(self):
        source = ControlledSource()
        source.tokens.put((1, None))
        focus = FocusBackend()
        temp_root = ROOT / 'runtime' / 'tmp'
        temp_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=temp_root) as folder:
            settings = Settings(calibration=json_value(Calibration(3., 3.)))
            runtime = Runtime(None, settings, queue.Queue(),
                              capture_factory=lambda **kw: BufferedCapture(**kw, capture_factory=source.factory,
                                                                          _focus_backend=focus, close_timeout=.01),
                              pico_factory=lambda **kw: FakePico(time.monotonic),
                              lease_factory=ProbeLease, diagnostic_factory=None, planner_factory=DirectPlanner,
                              log_folder=Path(folder))
            runtime.command('arm', goal=Point(0, 100, 0))
            runtime.start()
            try:
                wait_until(lambda: source.calls == 2 and source.in_poll)
                wait_until(lambda: runtime.state == 'paused')
                self.assertIn('250 ms', runtime.reason)
                self.assertGreater(runtime.lease.poll_count, 5)
                # The one captured frame can authorize one initial target;
                # waiting for a new frame must never create more renewals.
                commands = [a[0] for a in runtime.lease.actions]
                self.assertLessEqual(commands.count('keys'), 1)
                self.assertEqual(commands.count('renew'), 0)
                self.assertIn('stop', commands)
                self.assertTrue(source.in_poll)
                runtime.command('stop')
                wait_until(lambda: runtime.state == 'stopped')
                self.assertTrue(source.in_poll)
                self.assertEqual(runtime.pico.actions, [])
            finally:
                runtime.close()
                source.tokens.put(None)
                runtime.capture._worker_closed.wait(2)
            rows = [json.loads(line) for line in runtime.log.path.read_text(encoding='utf-8').splitlines()]
            timing = [row for row in rows if row['kind'] == 'runtime_timing']
            self.assertTrue(timing)
            self.assertEqual(timing[-1]['capture']['source'], 'buffered_windows_capture')
            self.assertIn('capture_poll_ms', timing[-1]['phases_ms'])


if __name__ == '__main__':
    unittest.main()
