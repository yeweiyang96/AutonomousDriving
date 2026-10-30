from dataclasses import replace
import unittest
from unittest.mock import patch

from adrive.models import Telemetry,Flags,PlanningError
from adrive.motion import MotionController
from tests.test_motion import route,PROFILE,BINDINGS


class MotionRefreshTests(unittest.TestCase):
    def controller(self,r):
        c=MotionController(replace(PROFILE,walk_toggle_allowed=False),BINDINGS,lambda p,m:True,r)
        t=Telemetry(1,1,0,1,0,0,0,7,Flags.VALID,10,z=0,ground_speed=7)
        self.assertEqual(c.update(t,10).kind,'window')
        return c,t

    def test_optional_extension_keeps_certified_curve_but_stops_before_it_expires(self):
        c,t=self.controller(route((0,0),(20,0),(40,0),(60,0)))
        old=c.trajectory
        error=PlanningError('motion_join','fixture: removed source vertex cannot yet be rejoined')
        with patch('adrive.motion.smooth_route',side_effect=error):
            w=c.update(replace(t,seq=2,x=31,captured_at=10.04),10.04,(26,))
            self.assertEqual(w.kind,'window');self.assertIs(c.trajectory,old)
            self.assertFalse(c.local_final)
            w=c.update(replace(t,seq=3,x=39,captured_at=10.08),10.08,(26,))
            self.assertEqual(w.kind,'stop');self.assertEqual(w.reason_code,'motion_join')

    def test_speed_increase_cannot_reuse_a_curve_with_insufficient_turning_radius(self):
        c,t=self.controller(route((0,0),(20,0),(20,20),(40,20)))
        c.progress=c.trajectory.length-10
        with patch('adrive.motion.smooth_route',side_effect=PlanningError('motion_tight','fixture')):
            w=c.update(replace(t,seq=2,x=20,y=10,facing=1.5707963267948966,speed=14,
                               ground_speed=14,captured_at=10.04),10.04,(26,))
        self.assertEqual(w.kind,'stop');self.assertFalse(w.frames)

    def test_kept_curve_still_requires_a_fresh_prediction_collision_certificate(self):
        c,t=self.controller(route((0,0),(20,0),(40,0),(60,0)))
        c.validate=lambda p,m:False
        with patch('adrive.motion.smooth_route',side_effect=PlanningError('motion_join','fixture')):
            w=c.update(replace(t,seq=2,x=31,captured_at=10.04),10.04,(26,))
        self.assertEqual(w.kind,'stop');self.assertEqual(w.reason_code,'motion_input_collision')


if __name__=='__main__':unittest.main()
