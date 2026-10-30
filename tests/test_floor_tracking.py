"""Offline floor inference: no game capture, server writes or device input."""
from dataclasses import replace
import json
import math
import time
import statistics
from pathlib import Path
import unittest
from unittest.mock import patch

from adrive.height import TerrainHeightResolver
from adrive.models import Flags, PlanningError, Point, Segment, Telemetry
from adrive.navmesh import load_pack
from adrive.telemetry import encode_frame, decode_frame, decode_image
from tests.test_addon_lua import AddonHarness, LuaRuntime

ROOT = Path(__file__).resolve().parents[1]


def sample(x, seq=1, **kw):
    return replace(Telemetry(1,seq,0,1426,x,0,0,7,Flags.VALID,seq*.1), **kw)


class Corridor:
    """Two overlapping corridors; only the lower one reaches the exterior."""
    build = 'fixture'
    def __init__(self):
        self.problem = None
    def heights(self, p):
        return [10.] if p.x < 0 else [10., 15.]
    def path(self, a,b,budget,**kwargs):
        if self.problem:
            raise PlanningError(self.problem, 'fixture')
        if a.z != b.z:
            raise PlanningError('no_route', 'disconnected floors')
        return [Segment((a,b),'fixture','navmesh','fixture')], 2


class FloorTrackingTests(unittest.TestCase):
    def setUp(self):
        self.geometry = Corridor()
        self.resolver = TerrainHeightResolver(None)
        self.resolver._pack = self.geometry
        self.resolver._mesh = self.geometry
        self.resolver._surfaces = self.geometry.heights

    def test_enter_and_leave_cave_with_zone_change_keeps_connected_floor(self):
        for seq,x in enumerate((-1,-.5,0,.5,1,.5,0,-.5),1):
            result=self.resolver.resolve(sample(x,seq,ui_map=1455 if x>=0 else 1426,zone_token=10 if x>=0 else 20))
            self.assertEqual(result.z,10)
            self.assertEqual((result.x,result.captured_at,result.seq),(x,seq*.1,seq))
        self.assertEqual(result.height_source,'trajectory')

    def test_ambiguous_cold_start_resolves_after_observed_exit(self):
        result=self.resolver.resolve(sample(.5))
        self.assertIsNone(result.z)
        self.assertEqual(result.floor_candidates,(10,15))
        result=self.resolver.resolve(sample(-.5,2))
        self.assertEqual(result.z,10)

    def test_never_uses_zone_or_indoors_alone_as_floor(self):
        result=self.resolver.resolve(sample(0,environment=3,zone_token=123,ui_map=1455))
        self.assertIsNone(result.z)
        result=self.resolver.resolve(sample(0,2,environment=1,zone_token=456))
        self.assertIsNone(result.z)

    def test_reload_gap_teleport_and_falling_invalidate_history(self):
        for change in (dict(session=2),dict(captured_at=10),dict(x=20),dict(environment=12)):
            with self.subTest(change=change):
                self.resolver._reset_track()
                self.resolver.resolve(sample(-.5))
                result=self.resolver.resolve(replace(sample(.5,2),**change))
                self.assertIsNone(result.z)

    def test_duplicate_does_not_refresh_sample_or_floor(self):
        first=self.resolver.resolve(sample(-.5))
        result=self.resolver.resolve(sample(.5,1,captured_at=10))
        self.assertEqual(result,first)

    def test_budget_failure_cannot_prove_selected_floor(self):
        self.resolver.resolve(sample(-.5))
        self.geometry.problem='budget'
        result=self.resolver.resolve(sample(.5,2))
        self.assertIsNone(result.z)
        result=self.resolver.resolve(sample(.5,3))
        self.assertIsNone(result.z)

    def test_no_connection_does_not_jump_to_unique_other_floor(self):
        self.resolver.resolve(sample(-.5))
        self.geometry.problem='no_route'
        result=self.resolver.resolve(sample(.5,2))
        self.assertIsNone(result.z)


class RegionProtocolTests(unittest.TestCase):
    def test_context_roundtrip_and_old_frames_rejected(self):
        t=sample(1,zone_token=0xfedcba98,environment=15)
        self.assertEqual(t,decode_frame(encode_frame(t),t.captured_at))
        with self.assertRaises(ValueError):
            decode_frame(bytes.fromhex('4144010078563412feffffff010083050000d8b2ffffe4620100004000070100066c'))

    @unittest.skipIf(LuaRuntime is None,'requires Lua')
    def test_actual_lua_zone_events_and_optional_api_failures(self):
        addon=AddonHarness()
        addon.lua.execute("function GetSubZoneText() return '铁炉堡' end; function IsIndoors() return true end; function IsFalling() return false end")
        addon.event('ZONE_CHANGED_INDOORS')
        t=addon.telemetry()
        value=5381
        for b in '铁炉堡'.encode():value=(value*33+b)&0xffffffff
        self.assertEqual((t.zone_token,t.environment),(value,7))
        self.assertEqual(t,decode_image(addon.image(),t.captured_at))
        addon.lua.execute("function GetSubZoneText() error('unavailable') end; function IsIndoors() error('unavailable') end")
        addon.event('ZONE_CHANGED')
        self.assertEqual((addon.telemetry().zone_token,addon.telemetry().environment),(0,4))


@unittest.skipUnless((ROOT/'data/navigation/manifest.json').exists(),'requires installed navigation')
class InstalledFloorTests(unittest.TestCase):
    def test_ironforge_entry_and_cave_route_observations(self):
        pack=load_pack(ROOT/'data')
        cases=[('ironforge',Point(0,-5040,-800),Point(0,-4821,-1152),1455),
               ('cave',Point(1,1452,-4788),Point(1,1440,-4830),1411)]
        report=[]
        for name,start,goal,ui in cases:
            route=pack.plan(start,goal)
            resolver=TerrainHeightResolver(ROOT/'data')
            try:
                seq=0
                timings=[]
                points=(*route.points,*reversed(route.points[:-1]))
                for a,b in zip(points,points[1:]):
                    n=max(1,math.ceil(a.distance(b)/.3))
                    for i in range(n):
                        t=i/n;seq+=1
                        raw=Telemetry(1,seq,a.c,ui if seq>100 else 1426,a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,0,7,Flags.VALID,seq*.1)
                        raw=decode_frame(encode_frame(raw),raw.captured_at)
                        began=time.perf_counter()
                        result=resolver.resolve(raw)
                        timings.append((time.perf_counter()-began)*1000)
                        self.assertIsNotNone(result.z,(name,seq,result,resolver.error))
                        self.assertLess(abs(result.z-(a.z+(b.z-a.z)*t)),1)
                report.append(dict(name=name,start=vars(route.start),goal=vars(route.goal),samples=seq,length=route.distance,
                                   median_ms=statistics.median(timings), max_ms=max(timings)))
            finally:resolver.close()
        folder=ROOT/'runtime/floor-tracking';folder.mkdir(exist_ok=True)
        (folder/'validation.json').write_text(json.dumps(dict(identity=pack.identity,build=pack.build,cases=report),indent=2))
