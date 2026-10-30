import math
import json
from pathlib import Path
import tempfile
from dataclasses import replace
import queue
import threading
import time
import unittest
from unittest.mock import Mock,patch

from adrive.controller import Calibration
from adrive.models import Point,Route,Segment,Telemetry,Flags
from adrive.motion import MotionProfile,MotionController
from adrive.motion_protocol import MotionClient
from adrive.motion_worker import MotionWorker
from adrive.runtime import Runtime
from adrive.settings import Settings
from adrive.pico import PicoError
from adrive.eventlog import EventLog
from tests import test_motion_native as native_tests
from tests.test_motion_native import BUILD,ROOT


class TranslationCalibrationTests(unittest.TestCase):
    def runtime(self):
        runtime=Runtime.__new__(Runtime)
        runtime.settings=Settings(calibration=dict(left_rate_rad_s=math.pi,right_rate_rad_s=math.pi,input_latency_s=.04))
        runtime.controller=Mock(calibration=Calibration(math.pi,math.pi,.04))
        runtime._calibration_candidate=Calibration(math.pi,math.pi,.04)
        runtime.stop_requested=threading.Event();runtime.quit=threading.Event();runtime.dispatch_lock=threading.Lock()
        runtime._calibration_epoch=runtime.control_epoch=0
        runtime.movement_samples=[];runtime.log=Mock();runtime._emit=Mock();runtime._status=Mock()
        return runtime

    def sample(self,runtime,direction,facing_change=0):
        before=Telemetry(1,1,0,1,10,10,0,0,Flags.VALID,10,z=0,ground_speed=7)
        runtime.pending=dict(before=before,translation=True)
        after=replace(before,seq=2,x=10+.84*math.cos(direction),y=10+.84*math.sin(direction),facing=facing_change)
        runtime._finish_translation_sample(after,10.2)

    def test_measures_both_strafe_and_diagonal_twice(self):
        runtime=self.runtime()
        with patch.object(Settings,'save') as save:
            for direction in (math.pi/2,-math.pi/2,math.pi/4,-math.pi/4)*2:self.sample(runtime,direction)
        self.assertEqual(save.call_count,1)
        self.assertTrue(runtime.settings.movement_calibration['strafe_verified'])
        self.assertAlmostEqual(runtime.settings.movement_calibration['diagonal_right_angle'],-math.pi/4)

    def test_wrong_binding_or_repeated_disagreement_is_rejected(self):
        for direction,yaw in ((-math.pi/2,0),(math.pi/2,.2)):
            with self.assertRaises(PicoError):self.sample(self.runtime(),direction,yaw)
        runtime=self.runtime()
        for direction in (math.pi/2,-math.pi/2,math.pi/4,-math.pi/4):self.sample(runtime,direction)
        for direction in (math.pi/2+.3,-math.pi/2,math.pi/4):self.sample(runtime,direction)
        with self.assertRaises(PicoError):self.sample(runtime,-math.pi/4)


class MotionDispatchTests(unittest.TestCase):
    def runtime(self):
        t=Telemetry(1,1,0,1,0,0,0,7,Flags.VALID,100,z=0,ground_speed=7)
        a,b=t.point,Point(0,30,0,0)
        trajectory=MotionController(MotionProfile(math.pi,math.pi),(26,4,7,20,8,84),lambda p,m:True,
            Route(a,b,(Segment((a,b),'fixture','navmesh','edge'),)),9).update(t,100)
        runtime=Runtime.__new__(Runtime)
        runtime.stop_requested=threading.Event();runtime.quit=threading.Event();runtime.dispatch_lock=threading.Lock()
        runtime.lease=Mock(spec=MotionClient);runtime.lease.is_active=True;runtime.lease.revision=1
        status=Mock(submitted_revision=0,applied_event=0,keys=())
        runtime._poll_lease=Mock(return_value=status);runtime._lease_renew=Mock()
        runtime.motion_worker=Mock();runtime.motion_worker.poll.return_value=(trajectory,1.)
        runtime.generation=9;runtime._motion_usb_pending=None;runtime._motion_event_pending=None
        runtime.capture=Mock();runtime.capture.foreground.return_value=True
        runtime.tracker=Mock();runtime.tracker.fresh.return_value=True
        runtime.log=Mock();runtime._emit=Mock();runtime._status=Mock()
        return runtime,t

    def test_fresh_window_dispatches_but_old_pose_or_speed_cannot_authorize(self):
        for change,age,expected in (({},.03,True),({},.10,False),({'x':3},.03,False),({'speed':14},.03,False),({'session':2},.03,False)):
            with self.subTest(change=change,age=age):
                runtime,t=self.runtime()
                with patch('adrive.runtime.time.monotonic',return_value=100+age):
                    runtime._motion_running_step(replace(t,**change),100+age)
                self.assertEqual(runtime.lease.set_window.called,expected)

    def test_stop_during_foreground_check_cannot_dispatch(self):
        runtime,t=self.runtime()
        runtime.capture.foreground.side_effect=lambda:runtime.stop_requested.set() or True
        with patch('adrive.runtime.time.monotonic',return_value=100.03):runtime._motion_running_step(t,100.03)
        runtime.lease.set_window.assert_not_called()

    def test_stopped_and_expired_decisions_are_serializable_with_source_evidence(self):
        for kind,age in (('stop',.03),('window',.10)):
            with self.subTest(kind=kind),tempfile.TemporaryDirectory() as folder:
                runtime,t=self.runtime();window,_=runtime.motion_worker.poll.return_value
                window=replace(window,kind=kind,reason='当前速度下弯道空间不足',reason_code='motion_tight')
                runtime.motion_worker.poll.return_value=(window,5.)
                runtime.motion_worker.identity=('navigation-id','dll-hash');runtime._halt=Mock()
                runtime.log=EventLog(Path(folder))
                try:
                    with patch('adrive.runtime.time.monotonic',return_value=100+age):
                        runtime._motion_running_step(t,100+age)
                finally:runtime.log.close()
                row=json.loads(runtime.log.path.read_text(encoding='utf-8').splitlines()[0])
                self.assertEqual(row['kind'],'motion_decision')
                self.assertEqual(row['decision_kind'],kind)
                self.assertEqual(row['reason_code'],'motion_tight')
                self.assertEqual(row['source']['seq'],t.seq)
                self.assertEqual(row['usable'],kind=='stop')
                self.assertEqual(row['navigation'],['navigation-id','dll-hash'])
                runtime.lease.set_window.assert_not_called()

    def test_worker_receives_usb_applied_action_id(self):
        runtime,t=self.runtime();runtime.motion_worker.poll.return_value=None
        runtime._poll_lease.return_value.applied_event=7
        runtime._poll_lease.return_value.keys=(26,)
        runtime._motion_running_step(t,100.03)
        runtime.motion_worker.submit.assert_called_once_with(t,(26,),7)

    def test_renewal_status_does_not_cover_missing_usb_submission(self):
        runtime,t=self.runtime();runtime._motion_usb_pending=(1,99.8)
        with self.assertRaises(PicoError):runtime._motion_running_step(t,100.01)
        runtime.lease.set_window.assert_not_called()

    def test_arrival_requires_fresh_stationary_position_within_tolerance(self):
        for x,expected in ((29.5,'arrived'),(29.3,None)):
            runtime,t=self.runtime();runtime.state='settling';runtime._settling_frame=(1,1);runtime._settling_at=100
            runtime._halt=Mock();runtime.route=Route(t.point,Point(0,30,0,0),())
            observed=replace(t,seq=2,x=x,speed=0,captured_at=100.03)
            with patch('adrive.runtime.time.monotonic',return_value=100.03):runtime._active_step(observed,100.03)
            args=runtime._halt.call_args.args
            self.assertEqual(args[1] if len(args)>1 else None,expected)


@unittest.skipUnless((BUILD/'adrive_navmesh.dll').exists(),'需要候选 DLL')
class IsolatedMotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):native_tests.MotionNativeTests.setUpClass()

    def test_worker_uses_real_detour_and_rejects_changed_package(self):
        a,b=Point(0,15,5,0),Point(0,25,5,0)
        r=Route(a,b,(Segment((a,b),'fixture','navmesh','fixture-edge'),))
        manifest=ROOT/'runtime/motion-fixtures/manifest.json'
        worker=MotionWorker(r,MotionProfile(math.pi,math.pi), (26,4,7,20,8,84),manifest,17)
        self.addCleanup(worker.close)
        deadline=time.monotonic()+10
        while not worker.ready:
            worker.poll();self.assertLess(time.monotonic(),deadline);time.sleep(.005)
        t=Telemetry(1,1,0,1,a.x,a.y,0,7,Flags.VALID,time.monotonic(),z=0,ground_speed=7)
        self.assertTrue(worker.submit(t,()))
        self.assertFalse(worker.submit(replace(t,seq=2),()))
        result=None
        while result is None:
            result=worker.poll();self.assertLess(time.monotonic(),deadline);time.sleep(.001)
        window,elapsed=result
        self.assertEqual(window.route_version,17)
        self.assertEqual(window.kind,'window',window.reason)
        self.assertTrue(all(26 in frame.keys for frame in window.frames))
        # A package change invalidates the existing worker's authority.
        manifest.touch()
        worker.submit(replace(t,seq=3,captured_at=time.monotonic()),())
        result=None
        while result is None:
            result=worker.poll();self.assertLess(time.monotonic(),deadline);time.sleep(.001)
        self.assertEqual(result[0].kind,'replan')
        self.assertIn('导航包',result[0].reason)
