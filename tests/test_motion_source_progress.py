"""Source-route order survives fillets, portal merging and moving joins."""
import math
import unittest

from adrive.models import Telemetry,Flags
from adrive.motion import MotionController,smooth_route,join_trajectory
from tests.test_motion import PROFILE,BINDINGS,route


class SourceProgressTests(unittest.TestCase):
    def test_corner_and_trimmed_portals_pass_only_at_arc_exit(self):
        source=route((0,0),(2,0),(19,0),(20,0),(20,1),(20,2),(20,20)).points
        trajectory=smooth_route(source,7,PROFILE,lambda p,m:True)
        straight,arc,tail=trajectory.primitives
        exit_s=straight.length+arc.length
        self.assertEqual(trajectory.source_points,source)
        self.assertEqual(len(trajectory.source_progress),len(source))
        self.assertAlmostEqual(trajectory.source_progress[1],2.)
        for index in (2,3,4,5):self.assertAlmostEqual(trajectory.source_progress[index],exit_s)
        self.assertAlmostEqual(trajectory.source_progress[-1],trajectory.length)

    def test_many_collinear_portals_keep_individual_order_after_merging(self):
        source=route(*((i,0) for i in range(101))).points
        trajectory=smooth_route(source,7,PROFILE,lambda p,m:True)
        self.assertEqual(len(trajectory.primitives),1)
        self.assertEqual(trajectory.source_points,source)
        for i,s in enumerate(trajectory.source_progress):self.assertAlmostEqual(s,i)

    def test_loop_leg_near_the_start_has_late_source_progress(self):
        source=route((0,0),(30,0),(30,30),(0,30),(0,5)).points
        trajectory=smooth_route(source,7,PROFILE,lambda p,m:True)
        progress=trajectory.source_progress
        self.assertEqual(progress[0],0)
        self.assertTrue(all(a<b for a,b in zip(progress,progress[1:])))
        self.assertGreater(progress[-1],100)
        self.assertAlmostEqual(progress[-1],trajectory.length)

    def test_join_defers_consumed_portals_and_shifts_remaining_progress(self):
        source=route((0,0),(1,0),(2,0),(5,0),(10,0),(60,0)).points
        base=smooth_route(source,14,PROFILE,lambda p,m:True)
        t=Telemetry(1,1,0,1,0,0,math.pi,14,Flags.VALID,10,z=0,ground_speed=14)
        joined,prefix=join_trajectory(base,t,14,PROFILE,lambda p,m:True)
        consumed=joined.at(prefix)[0].x
        self.assertGreater(prefix,0)
        self.assertEqual(joined.source_points,source)
        for raw_s,new_s in zip(base.source_progress[1:],joined.source_progress[1:]):
            self.assertAlmostEqual(new_s,prefix if raw_s<=consumed else raw_s+prefix-consumed)
        self.assertAlmostEqual(joined.source_progress[-1],joined.length)

    def test_outgoing_progress_advances_source_despite_small_lateral_deadband(self):
        r=route((0,0),(30,0),(30,30),(60,30),(60,60))
        controller=MotionController(PROFILE,BINDINGS,lambda p,m:True,r)
        trajectory=smooth_route(r.points[:3],7,PROFILE,lambda p,m:True)
        controller.trajectory=trajectory;controller.source_start_index=0
        controller.progress=trajectory.source_progress[1]-.1
        # This is 0.0312 yd left of the exact outgoing line. The removed
        # corner's old 99.9% projection test never considered it passed.
        t=Telemetry(1,1,0,1,29.9688,3.3,math.pi/2,7,Flags.VALID,10,z=0,ground_speed=7)
        result=controller.update(t,10,(26,))
        self.assertEqual(result.kind,'window',result.reason)
        self.assertEqual(controller.reference_index,1)


if __name__=='__main__':unittest.main()
