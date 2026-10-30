"""Road-only policy and real native corridor regression fixtures."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import threading
import unittest
from unittest.mock import patch

from adrive.models import Point, PlanningError, Segment
from adrive.navmesh import NavMeshPack
from adrive.roads import RoadNetwork,RoadPlanner
from adrive.data import DataBundle
from adrive.planner import Planner

ROOT=Path(__file__).resolve().parents[1]
BUILD=ROOT/'runtime/roads-native-build/Release'


def entry(p,root,**kw):
    data=p.read_bytes()
    return dict(path=p.relative_to(root).as_posix(),bytes=len(data),sha256=hashlib.sha256(data).hexdigest(),**kw)


@unittest.skipUnless((BUILD/'adrive_navmesh.dll').exists(),'Build the additive road ABI first')
class NativeRoadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root=ROOT/'runtime/unified-road-fixture';cls.root.mkdir(exist_ok=True)
        subprocess.run([str(BUILD/'navmesh_fixture.exe'),str(cls.root)],check=True,capture_output=True)
        shutil.copyfile(BUILD/'adrive_navmesh.dll',cls.root/'adrive_navmesh.dll')
        raw=dict(schema=1,kind='adrive-ground-navmesh',mmap_version=16,identity='roads-fixture',client={'version':'fixture'},
            settings={'walkable_climb_yards':.533333,'agent_radius_yards':.533333},
            library=entry(cls.root/'adrive_navmesh.dll',cls.root),
            maps=[entry(cls.root/f'{c:04}.mmap',cls.root,c=c) for c in (0,1)],
            tiles=[entry(cls.root/f'{c:04}_31_31.mmtile',cls.root,c=c,row=31,col=31) for c in (0,1)])
        (cls.root/'manifest.json').write_text(json.dumps(raw))
        cls.pack=NavMeshPack(cls.root/'manifest.json');cls.pack.record_diagnostics=False
        cls.a,cls.b=Point(0,15,5,0),Point(0,15,25,0)
    def network(self):
        nodes=[self.a,Point(0,25,5,0),Point(0,25,25,0),self.b]
        edges=[dict(id=f'unified:{i}',a=i,b=i+1,length=nodes[i].distance(nodes[i+1]),shard='fixture',sources=['fixture']) for i in range(3)]
        geometry=[dict(id=e['id'],points=[[p.x,p.y,p.z] for p in nodes[e['a']:e['b']+1]]) for e in edges]
        (self.root/'road-tile.json').write_text(json.dumps(geometry))
        raw=dict(schema=1,kind='adrive-unified-roads',identity='test',navigation_identity=self.pack.identity,build=self.pack.build,
            native_library_sha256=self.pack.raw['library']['sha256'],nodes=[[p.c,p.x,p.y,p.z] for p in nodes],edges=edges,
            shards={'fixture':entry(self.root/'road-tile.json',self.root)})
        (self.root/'roads.json').write_text(json.dumps(raw));return RoadNetwork(self.root/'roads.json',self.pack)
    def test_native_rejects_shortcut_outside_road_ribbon(self):
        with self.pack.mesh(0) as mesh:
            mesh.load({(31,31)})
            native,_=mesh.path(self.a,self.b,1000)
            self.assertLess(sum(s.distance for s in native),30)
            with self.assertRaises(PlanningError):mesh.path(self.a,self.b,1000,road_reference=[self.a,self.b],corridor_width=1)
    def test_native_keeps_floor_water_and_offmesh_filters(self):
        with self.pack.mesh(0) as mesh:
            mesh.load({(31,31)})
            for a,b in [(Point(0,5,5,5),Point(0,25,25,0)),(Point(0,5,15,0),Point(0,25,15,0))]:
                with self.subTest(a=a,b=b),self.assertRaises(PlanningError):
                    mesh.path(a,b,1000,road_reference=[a,b],corridor_width=1)
    def test_road_route_wins_over_shorter_open_geometry(self):
        route=RoadPlanner(self.pack,self.network()).plan(self.a,self.b)
        self.assertAlmostEqual(route.distance,40,places=3)
        self.assertTrue(all(s.reliability=='navmesh' for s in route.segments))
        self.assertEqual((route.start,route.goal),(self.a,self.b))
        self.assertEqual(self.pack.last_route_diagnostics['route_selection']['selected'],'road_only')
    def test_cancel_and_budget_do_not_fall_back(self):
        event=threading.Event();event.set();planner=RoadPlanner(self.pack,self.network())
        for opts,code in ((dict(cancel=event),'cancelled'),(dict(max_expansions=0),'budget')):
            with self.subTest(opts=opts),self.assertRaises(PlanningError) as raised:planner.plan(self.a,self.b,**opts)
            self.assertEqual(raised.exception.code,code)
    def test_start_floor_must_be_known(self):
        with self.assertRaises(PlanningError) as raised:RoadPlanner(self.pack,self.network()).plan(Point(0,5,5),self.b)
        self.assertEqual(raised.exception.code,'navmesh_height')
    def test_corrupt_road_geometry_fails_closed(self):
        network=self.network();(self.root/'road-tile.json').write_text('[]')
        with self.assertRaises(PlanningError) as raised:network.points(0)
        self.assertEqual(raised.exception.code,'roads_integrity')
    def test_missing_native_validation_records_fail_closed(self):
        with self.assertRaises(PlanningError) as raised:self.network().validation('fixture')
        self.assertEqual(raised.exception.code,'roads_invalid')
    def test_native_blocked_connection_still_rejected(self):
        with self.pack.mesh(0) as mesh:
            mesh.load({(31,31)});b=Point(0,25,5,0);line=[self.a,b]
            parts,_=mesh.path(self.a,b,1000,road_reference=line,corridor_width=3)
            mesh.block({s.edge_id for s in parts})
            with self.assertRaises(PlanningError):mesh.path(self.a,b,1000,road_reference=line,corridor_width=3)


class MissingRoadTests(unittest.TestCase):
    def test_production_never_uses_legacy_terrain_fallback(self):
        with self.assertRaises(PlanningError) as raised:Planner(DataBundle([],[],{})).plan(Point(0,0,0),Point(0,10,0))
        self.assertEqual(raised.exception.code,'navmesh_missing')


class FlatMesh:
    def __init__(self,*args):pass
    def load(self,*args):pass
    def block(self,*args):pass
    def close(self):pass
    def heights(self,p):return [0.]
    def path(self,a,b,budget,road_reference=None,**kw):
        if budget<2:
            e=PlanningError('budget','fixture limit');e.expanded=budget;raise e
        points=road_reference or [a,b]
        return [Segment((p,q),'fixture','navmesh','nav:fake:0:0,0,1:0,0,2') for p,q in zip(points,points[1:]) if p.distance(q)>1e-6],2


class PolicyNetwork:
    identity='policy'
    entry_candidates=RoadNetwork.entry_candidates
    def __init__(self,disconnected=False):
        from collections import defaultdict
        self.nodes=[Point(0,0,0,0),Point(0,0,100,0),Point(0,1000,0,0),Point(0,1000,100,0)]
        self.edges=[dict(id='unified:0',a=0,b=1,length=100),dict(id='unified:1',a=2,b=3,length=100)] if disconnected else [dict(id='unified:0',a=0,b=1,length=100)]
        self.adj=defaultdict(list);self.components={0:0,1:0,2:2,3:2}
        for i,e in enumerate(self.edges):self.adj[e['a']].append((e['b'],i,False));self.adj[e['b']].append((e['a'],i,True))
        self.entry_tiles=[(0,(min(self.nodes[e['a']].x,self.nodes[e['b']].x)-24.01,
            max(self.nodes[e['a']].x,self.nodes[e['b']].x)+24.01,-24.01,124.01),[i]) for i,e in enumerate(self.edges)]
    def points(self,i,reverse=False):
        e=self.edges[i];p=[self.nodes[e['a']],self.nodes[e['b']]];return list(reversed(p)) if reverse else p


class EmptyOutputMesh(FlatMesh):
    def path(self,*args,**kw):return [],2


class RoadPolicyTests(unittest.TestCase):
    def planner(self,disconnected=False):
        from types import SimpleNamespace
        pack=SimpleNamespace(identity='fake',raw={'library':{'sha256':'fixture'}},record_diagnostics=False,around=lambda *args:{(0,0)})
        return RoadPlanner(pack,PolicyNetwork(disconnected))
    @patch('adrive.roads.Mesh',FlatMesh)
    def test_disconnected_roads_cannot_use_long_access_as_a_field_bypass(self):
        with self.assertRaises(PlanningError) as raised:self.planner(True).plan(Point(0,0,0,0),Point(0,1000,100,0))
        self.assertEqual(raised.exception.code,'road_connection_unverified')
    @patch('adrive.roads.Mesh',FlatMesh)
    def test_truly_remote_endpoint_has_no_fixed_access_distance_limit(self):
        route=self.planner().plan(Point(0,-1000,0,0),Point(0,0,100,0))
        self.assertGreater(route.distance,1000)
        self.assertEqual(route.start,Point(0,-1000,0,0))
    @patch('adrive.roads.Mesh',FlatMesh)
    def test_confirmed_start_height_cannot_snap_to_an_unrelated_floor(self):
        with self.assertRaises(PlanningError) as raised:self.planner().plan(Point(0,0,0,50),Point(0,0,100,0))
        self.assertEqual(raised.exception.code,'navmesh_height')
    @patch('adrive.roads.Mesh',FlatMesh)
    def test_source_review_labels_do_not_change_road_cost(self):
        planner=self.planner();a,b=Point(0,0,0,0),Point(0,0,100,0)
        first=planner.plan(a,b)
        planner.network.edges[0].update(original_status='road_proposal',source='different')
        self.assertEqual(planner.plan(a,b).points,first.points)
    def test_spatial_entry_batches_match_global_distance_order(self):
        network=PolicyNetwork(True);p=Point(0,900,0,0)
        indices=[i for _,_,i in network.entry_candidates(p,lambda:None)]
        self.assertEqual(indices,[1,0])
        self.assertEqual([i for _,_,i in network.entry_candidates(p,lambda:None,{'unified:1'})],[0])
    @patch('adrive.roads.Mesh',EmptyOutputMesh)
    def test_nonzero_road_cannot_pass_with_empty_native_output(self):
        with self.assertRaises(PlanningError) as raised:self.planner().plan(Point(0,0,0,0),Point(0,0,100,0))
        self.assertEqual(raised.exception.code,'navmesh_endpoint')
    def test_road_floor_hints_cannot_select_an_observers_floor(self):
        from adrive.height import TerrainHeightResolver
        from tests.test_floor_tracking import Corridor,sample
        from types import SimpleNamespace
        resolver=TerrainHeightResolver(None);geometry=Corridor()
        resolver._pack=geometry;resolver._mesh=geometry;resolver._surfaces=geometry.heights
        resolver._road_hints=SimpleNamespace(heights=lambda *args:(15.,))
        self.assertIsNone(resolver.resolve(sample(0)).z)
        self.assertEqual(resolver.resolve(sample(-.5,2)).z,10)


if __name__=='__main__':unittest.main()
