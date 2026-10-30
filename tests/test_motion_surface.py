"""Ground fitting preserves native swept, clearance and floor certificates."""
from collections import OrderedDict
from dataclasses import replace
import math
from pathlib import Path
import time
import unittest
from unittest.mock import Mock,patch

from adrive.models import Point
from adrive.motion_worker import SurfaceValidator
from tests import test_motion_native

ROOT=Path(__file__).resolve().parents[1]


class GroundTraceTests(unittest.TestCase):
    def validator(self):
        v=SurfaceValidator.__new__(SurfaceValidator)
        v.mesh=Mock();v.pack=Mock();v.pack.around.return_value=()
        v.c=0
        v.corridor=(123,456);v.corridor_points=()
        v._ground_cache=OrderedDict();v._height_cache=OrderedDict();v._height_queries=0
        v.mesh.motion_validate.side_effect=[False,True]
        v.mesh.heights.side_effect=lambda p:[.3*math.sin(math.pi*p.x)]
        return v

    def test_ground_fit_keeps_xy_anchors_and_requires_whole_trace_certificate(self):
        v=self.validator();points=(Point(0,0,0,0),Point(0,1,0,0))
        trace=v.grounded(points,.37)
        self.assertEqual((trace[0],trace[-1]),points)
        self.assertGreater(max(p.z for p in trace),.29)
        self.assertTrue(all(a.distance(b)<=.100001 for a,b in zip(trace,trace[1:])))
        self.assertTrue(all(p.y==0 for p in trace))
        self.assertEqual(v.mesh.motion_validate.call_args.args,(trace,.37,v.corridor))
        self.assertEqual(v.mesh.motion_validate.call_count,2)

    def test_samples_never_override_failed_swept_certificate(self):
        v=self.validator();v.mesh.motion_validate.side_effect=[False,False]
        self.assertIsNone(v.grounded((Point(0,0,0,0),Point(0,1,0,0))))

    def test_fast_path_does_not_read_heights_or_change_points(self):
        v=self.validator();v.mesh.motion_validate.side_effect=None;v.mesh.motion_validate.return_value=True
        points=(Point(0,0,0,0),Point(0,1,0,0))
        self.assertEqual(v.grounded(points),points)
        v.mesh.heights.assert_not_called()

    def test_missing_ambiguous_or_distant_ground_is_rejected(self):
        for heights in ([],[0,.4],[.6]):
            with self.subTest(heights=heights):
                v=self.validator();v.mesh.heights.side_effect=lambda p:heights
                self.assertIsNone(v.grounded((Point(0,0,0,0),Point(0,1,0,0))))
                self.assertEqual(v.mesh.motion_validate.call_count,1)

    def test_cannot_relocate_endpoint_to_a_nearby_floor(self):
        v=self.validator();v.mesh.heights.side_effect=lambda p:[.3]
        self.assertIsNone(v.grounded((Point(0,0,0,0),Point(0,1,0,0))))
        self.assertEqual(v.mesh.motion_validate.call_count,1)

    def test_work_limits_fail_closed_and_do_not_cache_budget_starvation(self):
        v=self.validator();v._height_queries=v.MAX_HEIGHT_QUERIES
        points=(Point(0,0,0,0),Point(0,1,0,0))
        self.assertIsNone(v.grounded(points))
        self.assertNotIn((points,.1),v._ground_cache)
        v.mesh.heights.assert_not_called()
        v=self.validator()
        self.assertIsNone(v.grounded((Point(0,0,0,0),Point(0,300,0,0))))
        v.mesh.heights.assert_not_called()

    def test_repeat_trace_uses_the_matching_margin_certificate(self):
        v=self.validator();points=(Point(0,0,0,0),Point(0,1,0,0))
        first=v.grounded(points,.1);queries=v.mesh.heights.call_count
        self.assertEqual(v.grounded(points,.1),first)
        self.assertEqual(v.mesh.heights.call_count,queries)
        self.assertEqual(v.mesh.motion_validate.call_count,2)
        v.mesh.motion_validate.side_effect=[False,False]
        self.assertIsNone(v.grounded(points,.9))
        self.assertEqual(v.mesh.motion_validate.call_count,4)

    def test_prediction_fits_generated_endpoint_but_source_anchors_stay_fixed(self):
        v=self.validator();points=(Point(0,0,0,0),Point(0,1,0,0))
        v.mesh.heights.side_effect=lambda p:[.3*p.x]
        v.mesh.motion_validate.side_effect=[False,False,True]
        self.assertIsNone(v.grounded(points,.37))
        self.assertTrue(v.validate_prediction(points,.37))
        trace,margin,corridor=v.mesh.motion_validate.call_args.args
        self.assertEqual(trace[0],points[0])
        self.assertEqual(trace[-1],replace(points[-1],z=.3))
        self.assertTrue(all(p.y==0 and 0<=p.x<=1 for p in trace))
        self.assertEqual((margin,corridor),(.37,v.corridor))
        # Both cache entries retain their distinct endpoint semantics.
        self.assertIsNone(v.grounded(points,.37))
        self.assertTrue(v.validate_prediction(points,.37))
        self.assertEqual(v.mesh.motion_validate.call_count,3)

    def test_prediction_samples_never_override_failed_swept_certificate(self):
        v=self.validator();v.mesh.heights.side_effect=lambda p:[.3*p.x]
        v.mesh.motion_validate.side_effect=[False,False]
        self.assertFalse(v.validate_prediction((Point(0,0,0,0),Point(0,1,0,0)),.37))
        self.assertEqual(v.mesh.motion_validate.call_count,2)
        self.assertEqual(v.mesh.motion_validate.call_args.args[1:],(.37,v.corridor))

    def test_prediction_rejects_missing_ambiguous_or_distant_endpoint_ground(self):
        for endpoint_heights in ([],[.2,.4],[.6]):
            with self.subTest(heights=endpoint_heights):
                v=self.validator()
                v.mesh.heights.side_effect=lambda p: endpoint_heights if p.x==1 else [0]
                self.assertFalse(v.validate_prediction((Point(0,0,0,0),Point(0,1,0,0))))
                self.assertEqual(v.mesh.motion_validate.call_count,1)

    def test_prediction_cannot_relocate_observed_start_height(self):
        v=self.validator();v.mesh.heights.side_effect=lambda p:[0]
        points=(Point(0,0,0,.3),Point(0,1,0,.3))
        self.assertFalse(v.validate_prediction(points))
        self.assertEqual(v.mesh.motion_validate.call_count,1)
        self.assertEqual(v.mesh.motion_validate.call_args.args[0][0],points[0])

    def test_prediction_requires_same_map_corridor_and_respects_work_limit(self):
        v=self.validator();points=(Point(0,0,0,0),Point(0,1,0,0))
        v.corridor=()
        self.assertFalse(v.validate_prediction(points));v.mesh.motion_validate.assert_not_called()
        v.corridor=(123,456)
        self.assertFalse(v.validate_prediction((points[0],replace(points[1],c=1))))
        v.mesh.motion_validate.assert_not_called()
        v._height_queries=v.MAX_HEIGHT_QUERIES
        self.assertFalse(v.validate_prediction(points))
        self.assertNotIn(('prediction',points,.1),v._ground_cache)

    def test_mesh_replacement_discards_old_polygon_and_height_certificates(self):
        v=self.validator();points=(Point(0,0,0,0),Point(0,1,0,0))
        v.c=0;v.manifest=Mock();v.manifest.stat.return_value.st_mtime_ns=1;v.manifest_stamp=1
        v.corridor_points=points;v._ground_cache[(points,.1)]=points;v._height_cache[points[0]]=(0,)
        v.mesh.bytes=97*1024*1024;v.mesh.loaded=set();v.pack.around.return_value=((31,31),)
        new_mesh=Mock();new_mesh.motion_corridor.return_value=(789,)
        with patch('adrive.motion_worker.Mesh',return_value=new_mesh):v.prepare(points)
        self.assertEqual(v.corridor,(789,));self.assertFalse(v._ground_cache);self.assertFalse(v._height_cache)
        new_mesh.motion_corridor.assert_called_once_with(points,4.)


@unittest.skipUnless((test_motion_native.BUILD/'adrive_navmesh.dll').exists(),'需要候选连续运动 DLL')
class NativeGroundTraceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):test_motion_native.MotionNativeTests.setUpClass()

    def validator(self,points):
        v=SurfaceValidator(ROOT/'runtime/motion-fixtures/manifest.json')
        self.addCleanup(v.close);v.prepare(points)
        return v

    def test_wall_water_and_unconnected_floor_remain_blocked(self):
        # This ground corridor goes around the fixture's central hole.
        route=(Point(0,15,5,0),Point(0,25,5,0),Point(0,25,25,0),Point(0,15,25,0))
        v=self.validator(route)
        self.assertIsNone(v.grounded((route[0],route[-1])))
        self.assertIsNone(v.grounded((Point(0,5,15,0),)))
        self.assertIsNone(v.grounded((Point(0,15,.05,0),Point(0,25,.05,0)),.1))
        low=(Point(0,2,2,0),Point(0,8,2,0));v.prepare(low)
        high=tuple(replace(p,z=5) for p in low)
        self.assertIsNone(v.grounded(high))
        self.assertIsNone(v.grounded((low[0],high[1])))

    def test_ground_corridor_and_cross_tile_still_use_native_certificate(self):
        points=(Point(1,25,25,0),Point(1,25,35,0))
        v=self.validator(points)
        self.assertEqual(v.grounded(points),points)

    def test_prediction_fitting_preserves_wall_water_floor_and_observed_anchor_rejection(self):
        route=(Point(0,15,5,0),Point(0,25,5,0),Point(0,25,25,0),Point(0,15,25,0))
        v=self.validator(route)
        self.assertFalse(v.validate_prediction((route[0],replace(route[-1],z=.3))))
        self.assertFalse(v.validate_prediction((Point(0,5,15,0),)))
        self.assertFalse(v.validate_prediction((Point(0,15,.05,0),Point(0,25,.05,.3)),.1))
        low=(Point(0,2,2,0),Point(0,8,2,0));v.prepare(low)
        self.assertFalse(v.validate_prediction(tuple(replace(p,z=5) for p in low)))
        self.assertFalse(v.validate_prediction((low[0],replace(low[1],z=5))))
        self.assertFalse(v.validate_prediction((replace(low[0],z=.3),low[1])))
        self.assertTrue(v.validate_prediction((low[0],replace(low[1],z=.3))))


@unittest.skipUnless((ROOT/'data/navigation/manifest.json').exists(),'需要实际导航数据')
class ProductionGroundTraceTests(unittest.TestCase):
    def test_recorded_prediction_endpoint_on_crest_is_grounded_with_same_clearance(self):
        # Closed-loop replay, 5 yd/s and 5 ms integration: all 337 physical
        # chords passed, but this future endpoint underestimated ground by
        # .154730 yd. It is a generated endpoint, not a source-route anchor.
        source=(Point(0,-4895.625,-1115.125,502.0308532714844),
                Point(0,-4895.524514758771,-1115.6609260251928,502.0367126464844),
                Point(0,-4888.25927734375,-1114.2987060546875,502.3995056152344),
                Point(0,-4887.36328125,-1114.130615234375,502.1595764160156))
        trace=tuple(Point(0,*p) for p in (
            (-4889.8125,-1113.625,502.2488098144531),
            (-4889.729437972383,-1113.62950364081,502.15703818812085),
            (-4889.646254619295,-1113.6298397681337,502.15857244861496),
            (-4889.563158908086,-1113.6260075375756,502.1601003778563),
            (-4889.480359585935,-1113.6180165761948,502.1616181374889),
            (-4889.380796612205,-1113.60904333046,502.16344390353424),
            (-4889.280984659464,-1113.6031984853157,502.165277799471),
            (-4889.1811727067225,-1113.5973536401714,502.16711169540787),
            (-4889.081360753981,-1113.591508795027,502.1689455913447),
            (-4888.981430941305,-1113.5888000286184,502.17078520805535),
            (-4888.881496467131,-1113.5862181610648,502.17262505426817),
            (-4888.781560095592,-1113.5837632637792,502.17446507909403),
            (-4888.681619384944,-1113.5814352767811,502.1763053274902),
            (-4888.58167709949,-1113.5792342644613,502.17814574855646),
            (-4888.481730797341,-1113.5771601730426,502.1799863872598)))
        v=SurfaceValidator(ROOT/'data/navigation/manifest.json');self.addCleanup(v.close)
        if v.pack.identity!='7a4606dd9c967264':self.skipTest('实际导航几何版本已变更')
        v.prepare(source);margin=.745
        self.assertFalse(v.mesh.motion_validate(trace,margin,v.corridor))
        self.assertIsNone(v.grounded(trace,margin))
        with patch.object(v.mesh,'motion_validate',wraps=v.mesh.motion_validate) as validate:
            self.assertTrue(v.validate_prediction(trace,margin))
        fitted,used_margin,used_corridor=validate.call_args.args
        self.assertEqual((used_margin,used_corridor),(margin,v.corridor))
        self.assertEqual(fitted[0],trace[0])
        self.assertEqual((fitted[-1].x,fitted[-1].y),(trace[-1].x,trace[-1].y))
        self.assertAlmostEqual(fitted[-1].z,502.334716796875)
        self.assertTrue(v.mesh.motion_validate(fitted,margin,v.corridor))

    def test_recorded_ironforge_crest_fits_without_weakening_clearance(self):
        # Recorded 2026-10-30 startup: the original road climbs a small crest.
        # Its shortcut's endpoint-linear Z misses the detail surface by .25 yd.
        points=(Point(0,-4895.625,-1115.125,502.0308532714844),
                Point(0,-4895.524514758771,-1115.6609260251928,502.0367126464844),
                Point(0,-4888.25927734375,-1114.2987060546875,502.3995056152344),
                Point(0,-4887.36328125,-1114.130615234375,502.1595764160156))
        v=SurfaceValidator(ROOT/'data/navigation/manifest.json');self.addCleanup(v.close)
        if v.pack.identity!='7a4606dd9c967264':self.skipTest('实际导航几何版本已变更')
        v.prepare(points);chord=(points[0],points[-1])
        self.assertFalse(v.mesh.motion_validate(chord,.1,v.corridor))
        trace=v.grounded(chord,.1)
        self.assertIsNotNone(trace)
        self.assertEqual((trace[0],trace[-1]),chord)
        self.assertTrue(v.mesh.motion_validate(trace,.1,v.corridor))
        start=time.perf_counter()
        for _ in range(10):self.assertIsNotNone(v.grounded(chord,.1))
        self.assertLess(time.perf_counter()-start,.08)


if __name__=='__main__':unittest.main()
