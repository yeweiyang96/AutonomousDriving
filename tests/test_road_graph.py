"""Whole-road topology, explicit connectors and bounded graph data caches."""
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from adrive.models import Point,PlanningError,Segment
from adrive.navmesh import NavMeshPack
from adrive.roads import RoadNetwork,RoadPlanner,length,projection,slice_line


class RecordingMesh:
    calls=[]
    def __init__(self,*args):pass
    def load(self,*args):pass
    def block(self,*args):pass
    def close(self):pass
    def heights(self,p):return [0.]
    def path(self,a,b,budget,road_reference=None,**kwargs):
        self.calls.append((a,b,road_reference,kwargs))
        if budget<2:
            exc=PlanningError('budget','Fixture budget exhausted');exc.expanded=budget;raise exc
        points=road_reference or [a,b]
        return [Segment((p,q),'native-fixture','navmesh','nav:fixture:0:0,0,1:0,0,2')
                for p,q in zip(points,points[1:]) if p.distance(q)>1e-6],2


class BrokenConnectorMesh(RecordingMesh):
    def path(self,a,b,budget,road_reference=None,**kwargs):
        if a.y==100 and b.y==100 and a.x!=b.x:
            self.calls.append((a,b,road_reference,kwargs))
            exc=PlanningError('road_scope_unconnected','Fixture connector blocked');exc.expanded=2;raise exc
        return super().path(a,b,budget,road_reference=road_reference,**kwargs)


class DirectionalSourceMesh(RecordingMesh):
    """Opposite legal detours share one source corridor, not each other's line."""
    def path(self,a,b,budget,road_reference=None,**kwargs):
        if road_reference is None:return super().path(a,b,budget,**kwargs)
        self.calls.append((a,b,road_reference,kwargs))
        if kwargs['corridor_width']<24 or any(p.z is not None for p in road_reference):
            exc=PlanningError('road_scope_unconnected','Need the unchanged source corridor');exc.expanded=1;raise exc
        side=-15 if a.x<b.x else 15
        points=[a,Point(0,50,side,0),b]
        return [Segment((p,q),'native-fixture','navmesh','nav:fixture:0:0,0,1:0,0,2') for p,q in zip(points,points[1:])],2


class FullSourceRetryMesh(RecordingMesh):
    def path(self,a,b,budget,road_reference=None,**kwargs):
        if road_reference is None:return super().path(a,b,budget,**kwargs)
        if road_reference[0].x!=0 or road_reference[-1].x!=100:
            self.calls.append((a,b,road_reference,kwargs))
            exc=PlanningError('road_scope_unconnected','Fixture needs source arc outside trimmed projections');exc.expanded=1;raise exc
        return super().path(a,b,budget,road_reference=[a,b],**kwargs)


class RoadGraphTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory();self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name);RecordingMesh.calls=[]
        self.pack=SimpleNamespace(identity='fixture',build='fixture',raw={'library':{'sha256':'native'}},
            native=SimpleNamespace(corridor_path=object()),record_diagnostics=False,around=lambda *args:{(0,0)})
    def network(self,nodes,items,*,validation=False):
        raw=dict(schema=1,kind='adrive-unified-roads',identity='fixture-roads',navigation_identity=self.pack.identity,
            build=self.pack.build,native_library_sha256=self.pack.raw['library']['sha256'],nodes=[[p.c,p.x,p.y,p.z] for p in nodes],edges=[],shards={})
        if validation:
            raw['validation']=dict(kind='native-road-corridor-v1',navigation_identity=self.pack.identity,
                native_library_sha256=self.pack.raw['library']['sha256'],edge_count=len(items),shards={})
        def save(filename,value):
            data=json.dumps(value,separators=(',',':')).encode();(self.root/filename).write_bytes(data)
            return dict(path=filename,bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
        for i,item in enumerate(items):
            a,b=item['a'],item['b'];pts=item.get('points',[nodes[a],nodes[b]])
            e={k:v for k,v in item.items() if k not in ('points','reverse_points','reference')}
            e.update(id=f'edge:{i}',length=length(pts),shard=str(i))
            if 'bounds' not in item:e['bounds']=[min(p.x for p in pts),max(p.x for p in pts),min(p.y for p in pts),max(p.y for p in pts)]
            raw['edges'].append(e)
            geometry=dict(id=e['id'],points=[[p.x,p.y,p.z] for p in pts])
            for name in ('reverse_points','reference'):
                if name in item:geometry[name]=[[p.x,p.y,p.z] for p in item[name]]
            raw['shards'][str(i)]=save(f'geometry-{i}.json',[geometry])
            if validation:
                raw['validation']['shards'][str(i)]=save(f'validation-{i}.json',[dict(id=e['id'],
                    entry=[nodes[a].c,nodes[a].x,nodes[a].y,nodes[a].z],exit=[nodes[b].c,nodes[b].x,nodes[b].y,nodes[b].z],
                    forward={'status':'passed'},reverse={'status':'passed'})])
        manifest=self.root/'manifest.json';manifest.write_text(json.dumps(raw),encoding='utf-8')
        return RoadNetwork(manifest,self.pack)
    @patch('adrive.roads.Mesh',RecordingMesh)
    def test_full_u_bend_bounds_allow_access_at_its_distant_middle(self):
        a,b=Point(0,0,0,0),Point(0,0,200,0)
        nodes=[a,b,Point(0,20,50,0),Point(0,20,150,0)]
        whole=[a,Point(0,1000,0,0),Point(0,1000,200,0),b]
        network=self.network(nodes,[dict(a=0,b=1,kind='road',points=whole),dict(a=2,b=3,kind='road')])
        start,goal=Point(0,1000,100,0),Point(0,1000,150,0)
        self.assertEqual(next(network.entry_candidates(start,lambda:None))[2],0)
        route=RoadPlanner(self.pack,network).plan(start,goal)
        self.assertAlmostEqual(route.distance,50)
        self.assertEqual((route.start,route.goal),(start,goal))
    def test_exact_polyline_distance_orders_a_near_road_before_a_large_enclosing_box(self):
        nodes=[Point(0,-1000,-1000,0),Point(0,-1000,1000,0),Point(0,5,-10,0),Point(0,5,10,0)]
        distant_u=[nodes[0],Point(0,1000,-1000,0),Point(0,1000,1000,0),nodes[1]]
        network=self.network(nodes,[dict(a=0,b=1,kind='road',points=distant_u),dict(a=2,b=3,kind='road')])
        candidates=list(network.entry_candidates(Point(0,0,0,0),lambda:None))
        self.assertEqual([i for _,_,i in candidates],[1,0])
        self.assertAlmostEqual(candidates[0][0],5)
        self.assertAlmostEqual(candidates[1][0],1000)
    @patch('adrive.roads.Mesh',RecordingMesh)
    def test_parallel_roads_in_one_component_both_participate_in_entry_search(self):
        nodes=[Point(0,0,5,0),Point(0,20,5,0),Point(0,0,10,0),Point(0,100,10,0)]
        long_join=[nodes[1],Point(0,20,1000,0),Point(0,100,1000,0),nodes[3]]
        network=self.network(nodes,[dict(a=0,b=1,kind='road',sources=['near-dead-end']),
            dict(a=2,b=3,kind='road',sources=['parallel-through-road']),
            dict(a=1,b=3,kind='road',sources=['long-road-around'],points=long_join)])
        self.assertEqual(network.components[0],network.components[2])
        start,goal=Point(0,0,0,0),Point(0,100,0,0)
        route=RoadPlanner(self.pack,network).plan(start,goal)
        self.assertAlmostEqual(route.distance,120)
        self.assertAlmostEqual(self.pack.last_route_diagnostics['guide']['weighted_cost'],140)
        self.assertEqual(self.pack.last_route_diagnostics['guide']['links'][0]['edge'],'edge:1')
    @patch('adrive.roads.Mesh',RecordingMesh)
    def test_two_roads_join_through_an_explicit_validated_connector(self):
        nodes=[Point(0,0,0,0),Point(0,0,100,0),Point(0,200,100,0),Point(0,200,200,0)]
        network=self.network(nodes,[dict(a=0,b=1,kind='road',cost_factor=1),dict(a=2,b=3,kind='road',cost_factor=1),
            dict(a=1,b=2,kind='connector',cost_factor=2)],validation=True)
        self.assertNotIn(2,[i for _,_,i in network.entry_candidates(Point(0,100,100,0),lambda:None)])
        route=RoadPlanner(self.pack,network).plan(nodes[0],nodes[-1])
        diagnostic=self.pack.last_route_diagnostics
        self.assertAlmostEqual(route.distance,400)
        self.assertAlmostEqual(diagnostic['guide']['weighted_cost'],600)
        self.assertAlmostEqual(diagnostic['guide']['length_xy'],400)
        self.assertAlmostEqual(diagnostic['guide']['road_length_xy'],200)
        self.assertAlmostEqual(diagnostic['guide']['connector_length_xy'],200)
        self.assertAlmostEqual(diagnostic['guide']['connector_weighted_extra_xy'],200)
        self.assertEqual(diagnostic['route_selection']['connector_count'],1)
        self.assertEqual(diagnostic['route_selection']['selected'],'road_only')
        self.assertTrue(any(e.get('kind')=='connector' for e in diagnostic['guide']['links']))
        self.assertTrue(all(reference is not None or a.distance(b)<1e-5 or (a,b)==(nodes[1],nodes[2])
                            for a,b,reference,_ in RecordingMesh.calls))
    @patch('adrive.roads.Mesh',RecordingMesh)
    def test_shorter_open_geometry_does_not_replace_a_connected_road(self):
        a,b=Point(0,0,0,0),Point(0,20,0,0)
        path=[a,Point(0,0,7.5,0),Point(0,20,7.5,0),b]
        network=self.network([a,b],[dict(a=0,b=1,kind='road',points=path),dict(a=0,b=1,kind='connector',cost_factor=2)])
        route=RoadPlanner(self.pack,network).plan(a,b)
        self.assertAlmostEqual(route.distance,35)
        self.assertGreater(route.distance,a.distance(b))
        self.assertEqual(self.pack.last_route_diagnostics['route_selection']['connector_count'],0)
    @patch('adrive.roads.Mesh',BrokenConnectorMesh)
    def test_failed_connector_is_excluded_without_an_open_field_fallback(self):
        nodes=[Point(0,0,0,0),Point(0,0,100,0),Point(0,200,100,0),Point(0,200,200,0)]
        network=self.network(nodes,[dict(a=0,b=1,kind='road'),dict(a=2,b=3,kind='road'),dict(a=1,b=2,kind='connector')])
        with self.assertRaises(PlanningError) as caught:RoadPlanner(self.pack,network).plan(nodes[0],nodes[-1])
        self.assertEqual(caught.exception.code,'road_connection_unverified')
        trace=self.pack.last_route_diagnostics['road_trace']
        self.assertEqual(len([r for r in trace if r['mode']=='rejected_connector']),1)
        self.assertTrue(all(reference is not None or a.distance(b)<1e-5 or (a,b)==(nodes[1],nodes[2])
                            for a,b,reference,_ in RecordingMesh.calls))
    def test_geometry_and_validation_caches_evict_old_shards(self):
        nodes=[Point(0,i*10,0,0) for i in range(42)]
        network=self.network(nodes,[dict(a=i,b=i+1,kind='road') for i in range(41)],validation=True)
        for i in range(41):
            network.points(i)
            self.assertLessEqual(len(network.geometry),32)
            self.assertLessEqual(len(network._validation_cache),8)
            self.assertLessEqual(network._geometry_bytes,network._geometry_limit)
            self.assertLessEqual(network._validation_bytes,network._validation_limit)
        self.assertNotIn('0',network.geometry);self.assertNotIn('0',network._validation_cache)
        self.assertEqual(network.points(0),nodes[:2])
    def test_oversized_geometry_shard_fails_before_loading(self):
        network=self.network([Point(0,0,0,0),Point(0,10,0,0)],[dict(a=0,b=1)])
        network._geometry_limit=1
        with self.assertRaises(PlanningError) as caught:network.points(0)
        self.assertEqual(caught.exception.code,'budget')
    def test_legacy_kind_and_bounds_defaults_remain_readable(self):
        network=self.network([Point(0,0,0,0),Point(0,100,0,0)],[dict(a=0,b=1,bounds=None)])
        self.assertEqual(next(network.entry_candidates(Point(0,50,0,0),lambda:None))[2],0)
        self.assertEqual(network.points(0),network.nodes)
    def test_invalid_cost_factor_cannot_reintroduce_source_priority(self):
        with self.assertRaises(PlanningError) as caught:
            self.network([Point(0,0,0,0),Point(0,10,0,0)],[dict(a=0,b=1,kind='road',cost_factor=2)])
        self.assertEqual(caught.exception.code,'roads_invalid')
    def test_unknown_source_heights_are_not_interpolated_from_actual_geometry(self):
        source=[Point(0,0,0),Point(0,100,0)]
        self.assertIsNone(projection(Point(0,50,10,20),source)[3].z)
        self.assertEqual(slice_line(source,20,80),[Point(0,20,0),Point(0,80,0)])
        mixed=[Point(0,0,0,10),Point(0,100,0)]
        self.assertEqual(slice_line(mixed,0,50),[Point(0,0,0,10),Point(0,50,0)])
        a,b=Point(0,0,0,0),Point(0,100,0,0)
        forward=[a,Point(0,50,-15,0),b]
        reverse=[b,Point(0,50,15,0),a]
        network=self.network([a,b],[dict(a=0,b=1,points=forward,reverse_points=reverse,reference=source,
            reverse_length=length(reverse))],validation=True)
        self.assertEqual(network.points(0,True),reverse)
        self.assertEqual(network.reference(0),source)
        self.assertEqual(network.reference(0,b,a),list(reversed(source)))
        self.assertEqual(network.reference(0,Point(0,20,-6,0),Point(0,80,-6,0)),[Point(0,20,0),Point(0,80,0)])
    @patch('adrive.roads.Mesh',DirectionalSourceMesh)
    def test_each_direction_keeps_the_same_source_corridor_instead_of_recentering(self):
        a,b=Point(0,0,0,0),Point(0,100,0,0)
        source=[Point(0,0,0),Point(0,100,0)]
        forward=[a,Point(0,50,-15,0),b];reverse=[b,Point(0,50,15,0),a]
        network=self.network([a,b],[dict(a=0,b=1,points=forward,reverse_points=reverse,reference=source,
            reverse_length=length(reverse))])
        f=RoadPlanner(self.pack,network).plan(a,b)
        r=RoadPlanner(self.pack,network).plan(b,a)
        self.assertIn(Point(0,50,-15,0),f.points)
        self.assertIn(Point(0,50,15,0),r.points)
        for _,_,ref,_ in RecordingMesh.calls:
            if ref is not None:self.assertTrue(ref==source or ref==list(reversed(source)))
    @patch('adrive.roads.Mesh',FullSourceRetryMesh)
    def test_partial_access_can_retry_only_the_same_full_source_corridor(self):
        a,b=Point(0,0,0,0),Point(0,100,0,0)
        forward=[a,Point(0,40,-15,0),Point(0,60,-15,0),b]
        source=[Point(0,0,0),Point(0,100,0)]
        network=self.network([a,b],[dict(a=0,b=1,points=forward,reference=source)])
        route=RoadPlanner(self.pack,network).plan(forward[1],forward[2])
        self.assertAlmostEqual(route.distance,20)
        self.assertTrue(any(t['mode']=='accepted_full_source_corridor' and t['width']==24
                            for t in self.pack.last_route_diagnostics['road_trace']))
        self.assertFalse(any(reference is None and a.distance(b)>1e-5 for a,b,reference,_ in RecordingMesh.calls))
    @patch('adrive.roads.Mesh',RecordingMesh)
    def test_reverse_lengths_are_used_for_graph_costs_and_unweighted_guide_length(self):
        a,b=Point(0,0,0,0),Point(0,100,0,0)
        forward=[a,Point(0,50,-15,0),b];reverse=[b,Point(0,50,30,0),a]
        network=self.network([a,b],[dict(a=0,b=1,points=forward,reverse_points=reverse,
            reverse_length=length(reverse),reference=[Point(0,0,0),Point(0,100,0)])])
        RoadPlanner(self.pack,network).plan(b,a)
        guide=self.pack.last_route_diagnostics['guide']
        self.assertAlmostEqual(guide['weighted_cost'],length(reverse))
        self.assertAlmostEqual(guide['length_xy'],length(reverse))
        self.assertAlmostEqual(guide['road_length_xy'],length(reverse))
    @unittest.skipUnless((Path(__file__).resolve().parents[1]/'data/navigation/manifest.json').exists(),'Needs installed collision pack')
    def test_real_cave_1676_two_way_routes_share_the_original_seven_point_corridor(self):
        self.pack=NavMeshPack(Path(__file__).resolve().parents[1]/'data/navigation/manifest.json')
        self.pack.record_diagnostics=False
        if self.pack.identity!='7a4606dd9c967264':self.skipTest('Known geometry identity changed; revalidate the recorded source case')
        a=Point(0,-5199.22119140625,-1506.351806640625,509.5183410644531)
        b=Point(0,-4893,-1691,504.1294250488281)
        source=[Point(0,x,y) for x,y in [(-5197,-1509),(-5212,-1521),(-5221,-1529),(-5212,-1554),
            (-5196,-1571),(-5146,-1604),(-4893,-1691)]]
        used=0
        with self.pack.mesh(0) as mesh:
            mesh.load(set().union(*(self.pack.around(p) for p in [*source,a,b])))
            def native(start,goal,reference):
                nonlocal used
                last=None
                for width in (3.,9.,24.):
                    try:
                        parts,n=mesh.path(start,goal,min(8192,131072-used),road_reference=reference,corridor_width=width)
                        used+=n;return [parts[0].points[0],*(s.points[-1] for s in parts)]
                    except PlanningError as exc:
                        used+=getattr(exc,'expanded',0);last=exc
                        if exc.code not in ('road_scope_unconnected','no_route','blocked_navmesh'):raise
                raise last
            forward=native(a,b,source);reverse=native(b,a,list(reversed(source)))
        network=self.network([a,b],[dict(a=0,b=1,points=forward,reverse_points=reverse,
            reverse_length=length(reverse),reference=source,sources=['agps:Caves:0:0:1676'])])
        f=RoadPlanner(self.pack,network).plan(a,b,max_expansions=131072-used);used+=f.expanded
        r=RoadPlanner(self.pack,network).plan(b,a,max_expansions=131072-used);used+=r.expanded
        self.assertGreater(f.distance,400);self.assertLess(f.distance,450)
        self.assertGreater(r.distance,400);self.assertLess(r.distance,450)
        self.assertNotEqual(f.points,tuple(reversed(r.points)))
        self.assertEqual(network.reference(0),source)
        self.assertLessEqual(used,131072)
        self.assertAlmostEqual(self.pack.last_route_diagnostics['guide']['length_xy'],length(reverse),places=4)


if __name__=='__main__':unittest.main()
