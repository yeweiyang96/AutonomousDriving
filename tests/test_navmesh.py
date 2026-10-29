"""Real native Detour tests against constructed geometry, never game input."""
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
import threading
import unittest

from adrive.models import Flags, Point, PlanningError, Telemetry, Route, Segment
from adrive.controller import Calibration
from adrive.continuous_controller import ContinuousController
from adrive.navmesh import NavMeshPack, Native, load_pack
from adrive.data import DataBundle
from adrive.planner import Planner
from adrive.telemetry import decode_frame, decode_image, encode_frame, render_frame
from tests.test_addon_lua import AddonHarness, LuaRuntime

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / 'runtime/navmesh-build/Release/navmesh_fixture.exe'
DLL = ROOT / 'data/navigation/adrive_navmesh.dll'


def entry(path, root, **metadata):
    data = path.read_bytes()
    return dict(path=path.relative_to(root).as_posix(), bytes=len(data),
                sha256=hashlib.sha256(data).hexdigest(), **metadata)


@unittest.skipUnless(FIXTURE.exists() and DLL.exists(), '需要先运行 tools/build_navmesh.ps1 编译真实 Detour 测试工具')
class NativeNavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = ROOT / 'runtime/navmesh-fixtures'
        root.mkdir(parents=True, exist_ok=True)
        subprocess.run([str(FIXTURE), str(root)], check=True, capture_output=True)
        shutil.copyfile(DLL, root / DLL.name)
        raw = {'schema': 1, 'kind': 'adrive-ground-navmesh', 'mmap_version': 16,
               'identity': 'fixture', 'client': {'version': 'constructed-fixture'},
               'settings': {'walkable_climb_yards': .533333, 'agent_radius_yards': .533333},
               'library': entry(root / DLL.name, root),
               'maps': [entry(root / f'{c:04}.mmap', root, c=c) for c in (0, 1)],
               'tiles': [entry(root / f'{c:04}_31_31.mmtile', root, c=c, row=31, col=31) for c in (0, 1)]}
        (root / 'manifest.json').write_text(json.dumps(raw), encoding='utf-8')
        cls.pack = NavMeshPack(root / 'manifest.json')

    def test_two_floors_are_ambiguous_without_height(self):
        heights = self.pack.heights(Point(0, 5, 5))
        self.assertEqual([0., 5.], heights)
        with self.assertRaises(PlanningError) as raised:
            self.pack.plan(Point(0, 5, 5), Point(0, 25, 25, 0))
        self.assertEqual(raised.exception.code, 'navmesh_height')

    def test_same_xy_on_other_floor_is_not_arrived_or_a_straight_jump(self):
        with self.assertRaises(PlanningError) as raised:
            self.pack.plan(Point(0, 5, 5, 0), Point(0, 5, 5, 5))
        self.assertEqual(raised.exception.code, 'no_route')

    def test_height_equally_close_to_two_floors_is_rejected(self):
        with self.assertRaises(PlanningError) as raised:
            self.pack.plan(Point(0,5,5,2.5),Point(0,25,25,0))
        self.assertEqual(raised.exception.code,'navmesh_height')

    def test_failed_road_hint_cannot_turn_into_unchecked_connector(self):
        a,b=Point(0,15,5,0),Point(0,15,25,0)
        guide=Route(a,b,(Segment((a,Point(0,15,15),b),'old road','reviewed','road:1'),))
        route=self.pack.plan(a,b,guide)
        self.assertAlmostEqual(route.distance,10+2*math.sqrt(50),places=5)
        self.assertTrue(all(s.reliability=='navmesh' for s in route.segments))

    def test_hole_and_offmesh_shortcut_are_avoided(self):
        a, b = Point(0, 15, 5, 0), Point(0, 15, 25, 0)
        route = self.pack.plan(a, b)
        self.assertAlmostEqual(route.distance, 10+2*math.sqrt(50), places=5)
        self.assertEqual(route.start, a)
        self.assertEqual(route.goal, b)
        self.assertTrue(all(s.reliability == 'navmesh' for s in route.segments))
        for point in route.points:
            self.assertFalse(10 < point.x < 20 and 10 < point.y < 20)
            self.assertEqual(point.z, 0)

    def test_water_and_outside_mesh_are_never_nearest_free_cell_snapped(self):
        for a in (Point(0, 5, 15, 0), Point(0, 15, 15, 0), Point(0, -1, 5, 0)):
            with self.assertRaises(PlanningError) as raised:
                self.pack.plan(a, Point(0, 25, 25, 0))
            self.assertEqual(raised.exception.code, 'navmesh_outside')

    def test_budget_and_cancellation_never_return_partial(self):
        for budget, cancel, code in ((0, None, 'budget'), (1000, threading.Event(), 'cancelled')):
            if cancel:
                cancel.set()
            with self.assertRaises(PlanningError) as raised:
                self.pack.plan(Point(0, 15, 5, 0), Point(0, 15, 25, 0), max_expansions=budget, cancel=cancel)
            self.assertEqual(raised.exception.code, code)

    def test_blocked_polygon_transition_is_not_used(self):
        a,b = Point(0,15,5,0),Point(0,15,25,0)
        first = self.pack.plan(a,b)
        blocked = frozenset(s.edge_id for s in first.segments)
        with self.assertRaises(PlanningError):
            self.pack.plan(a,b,blocked_edges=blocked)

    def test_corrupt_file_rejected_before_native_decode(self):
        bad = dict(self.pack.raw['tiles'][0], sha256='0'*64)
        with self.assertRaises(PlanningError) as raised:
            self.pack.read(bad)
        self.assertEqual(raised.exception.code, 'navmesh_integrity')


class HeightProtocolTests(unittest.TestCase):
    def test_v2_roundtrip_pixel_panel_and_crc(self):
        for z in (-340.123, 0, 950.99):
            t = Telemetry(8, 16, 0, 1415, -9458, 50, .4, 7, Flags.VALID, 12, z)
            data = encode_frame(t)
            self.assertEqual((38,2), (len(data),data[2]))
            decoded = decode_frame(data,12)
            self.assertLessEqual(abs(decoded.z-z),1/32)
            self.assertEqual(decoded,decode_image(render_frame(t),12))
            damaged=bytearray(data);damaged[-3]^=1
            with self.assertRaises(ValueError):decode_frame(bytes(damaged))

    @unittest.skipIf(LuaRuntime is None, '需要 Lupa 执行生产 Lua 代码')
    def test_actual_lua_height_and_unknown_height_clear_old_pixels(self):
        addon=AddonHarness()
        addon.state.z=123.4375
        addon.tick()
        self.assertEqual(addon.telemetry().z,123.4375)
        self.assertEqual(addon.telemetry(),decode_image(addon.image(),addon.state.clock))
        addon.state.position_available=False
        addon.tick()
        self.assertIsNone(addon.telemetry().z)
        self.assertEqual(addon.telemetry(),decode_image(addon.image(),addon.state.clock))


class HeightControllerTests(unittest.TestCase):
    def controller(self):
        a,b=Point(0,0,0,5),Point(0,10,0,5)
        controller=ContinuousController(Calibration(2.5,2.5))
        controller.set_route(Route(a,b,(Segment((a,b),'navmesh fixture','navmesh','nav:fixture'),)))
        return controller

    def test_height_is_required_and_wrong_floor_never_counts_as_arrival(self):
        for height in (None,0):
            controller=self.controller()
            result=controller.update(Telemetry(1,1,0,1415,10,0,0,0,Flags.VALID,12,height),12)
            self.assertEqual(result.kind,'stop')
            self.assertNotEqual(controller.state,'arrived')

    def test_leaving_fine_corridor_replans_instead_of_direct_rejoin(self):
        controller=self.controller()
        result=controller.update(Telemetry(1,1,0,1415,3,1,0,0,Flags.VALID,12,5),12)
        self.assertEqual(result.kind,'replan')


@unittest.skipUnless((ROOT/'data/navigation/manifest.json').exists(),'需要当前客户端生成的精细导航包')
class InstalledNavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle=DataBundle.load(ROOT/'data')
        cls.pack=load_pack(cls.bundle.asset_root)

    def test_planner_routes_across_tiles_on_both_continents(self):
        cases=((self.bundle.nodes[3766],self.bundle.nodes[3908]),
               (Point(1,1377.398,-4371.786,26.32518),self.bundle.nodes[512]))
        for a,b in cases:
            with self.subTest(continent=a.c):
                route=Planner(self.bundle).plan(a,b)
                self.assertGreater(route.distance,1000)
                self.assertTrue(all(s.reliability=='navmesh' for s in route.segments))
                self.assertEqual((route.start.x,route.start.y),(a.x,a.y))
                self.assertEqual((route.goal.x,route.goal.y),(b.x,b.y))
                self.assertTrue(all(p.z is not None and math.isfinite(p.z) for p in route.points))
                tiles={s.edge_id.split(':')[3].rsplit(',',1)[0] for s in route.segments}
                self.assertGreater(len(tiles),1)
                ContinuousController(Calibration(2.5,2.5)).set_route(route)

    def test_ambiguous_city_point_is_not_silently_replaced_by_legacy_grid(self):
        with self.assertRaises(PlanningError) as raised:
            Planner(self.bundle).plan(Point(1,1377.398,-4371.786),Point(1,1300,-4380))
        self.assertEqual(raised.exception.code,'navmesh_height')

    def test_real_exact_xy_upper_floor_path(self):
        a,b=Point(1,1377.398,-4371.786,26.32518),Point(1,1300,-4380)
        route=self.pack.plan(a,b)
        self.assertAlmostEqual(route.distance,77.83608,places=3)
        self.assertAlmostEqual(route.start.z,26.32518,places=3)
        self.assertAlmostEqual(route.goal.z,27.78383,places=3)
