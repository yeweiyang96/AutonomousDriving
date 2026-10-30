"""Reference import preserves road topology and declared portals, never executes Lua."""
from collections import Counter
from pathlib import Path
import tempfile
import unittest

from adrive.data import DataBundle,RoadEdge
from adrive.models import Point
from adrive.road_sources import SourceRoad,source_roads


ROOT=Path(__file__).resolve().parents[1]


class SourceTopologyTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name)
        (self.root/'data').mkdir()
        (self.root/'data/bundle.json').write_text('{}',encoding='utf-8')
        self.data=self.root/'addons/AzerothGPS/Data';self.data.mkdir(parents=True)
        for name in ('Roads','Cities','Capitals','Caves','Indoors'):
            self.write(name,'ns.Roads = {[0]={n={},e={}},[1]={n={},e={}}}' if name=='Roads' else '')
        self.bundle=DataBundle([],[],{})
    def write(self,name,source):
        (self.data/(name+'.lua')).write_text(source,encoding='utf-8')
    def load(self):return source_roads(self.root,self.bundle)
    def test_legacy_source_constructor_and_metadata_defaults_are_independent(self):
        a,b=SourceRoad('a',[],{}),SourceRoad('b',[],{})
        self.assertIsNone(a.a);self.assertIsNone(a.b)
        a.metadata['tag']=True
        self.assertEqual(b.metadata,{})
    def test_existing_shared_node_identity_survives_without_geometric_reconstruction(self):
        self.bundle=DataBundle([Point(0,0,0,0),Point(0,1.001,0,0),Point(0,2,0,0)],
            [RoadEdge('a',0,1,'user_reviewed','walk','fixture'),RoadEdge('b',1,2,'road_proposal','walk','fixture')],{})
        roads,_,_=self.load()
        self.assertEqual(roads[0].b,roads[1].a)
        self.assertEqual(roads[0].b,'existing:1')
        self.assertEqual(roads[0].points[1],Point(0,1.001,0,0))
        self.assertEqual(roads[0].metadata['height_a'],0)
        self.assertEqual(roads[1].provenance['original_status'],'road_proposal')
    def test_normal_and_drawn_roads_keep_whole_polylines_and_original_nodes(self):
        self.write('Roads','''ns.Roads = {
            [0]={n={0,0,10,0,20,0},e={{1,2,10,0,0,0,5,1,10,0},{2,3,10,2,10,0,20,0}}},
            [1]={n={},e={}}}
            error("The importer must not execute this Lua")''')
        roads,_,excluded=self.load()
        self.assertEqual(len(roads),2);self.assertEqual(excluded,[])
        self.assertEqual(roads[0].b,roads[1].a)
        self.assertEqual(roads[0].points,[Point(0,0,0),Point(0,5,1),Point(0,10,0)])
        self.assertEqual(roads[1].provenance['original_source'],2)
        self.assertEqual(roads[0].metadata['original_a'],1)
    def test_lifts_swimming_and_drops_stay_disabled_but_stairs_survive(self):
        self.write('Roads','''ns.Roads = {[0]={n={0,0,10,0},e={
            {1,2,10,3,0,0,10,0},{1,2,10,5,0,0,10,0},{1,2,10,7,0,0,10,0},
            {1,2,10,6,0,0,10,0}}},[1]={n={},e={}}}''')
        roads,_,excluded=self.load()
        self.assertEqual([r.provenance['original_source'] for r in roads],[6])
        self.assertEqual(len(excluded),3)
    def test_overlay_portals_and_floor_heights_are_retained(self):
        self.write('Indoors','''ns.RoadOverlays[0] = { {indoor=true,n={0,0,10,0,20,0},nz={10,12,18},
            joins={1},bridge={2},e={{1,2,10,0,0,0,10,0},{2,3,10,6,10,0,20,0}}} }''')
        roads,_,_=self.load();a,b=roads
        self.assertEqual(a.metadata['special'],'indoor')
        self.assertTrue(a.metadata['join_a']);self.assertFalse(a.metadata['join_b'])
        self.assertTrue(a.metadata['bridge_b'])
        self.assertTrue(a.metadata['bridge_allowed_a']);self.assertTrue(a.metadata['bridge_allowed_b'])
        self.assertFalse(b.metadata['bridge_allowed_b'])
        self.assertEqual((b.metadata['height_a'],b.metadata['height_b']),(12,18))
        self.assertEqual(a.b,b.a)
    def test_same_xy_different_source_graphs_do_not_merge_nodes(self):
        self.write('Roads','ns.Roads={[0]={n={0,0,10,0},e={{1,2,10,0,0,0,10,0}}},[1]={n={},e={}}}')
        self.write('Caves','ns.RoadOverlays[0]={{cave=true,n={0,0,10,0},e={{1,2,10,0,0,0,10,0}}}}')
        roads,_,_=self.load()
        self.assertEqual(roads[0].points,roads[1].points)
        self.assertNotEqual(roads[0].a,roads[1].a)
        self.assertFalse(roads[1].metadata['bridge_allowed_a'])
    def test_pseudo_city_world_height_offset_does_not_leak_to_capitals(self):
        self.write('Cities','''ns.CityLevels[10001]={base=0,zoff=62.55}
            ns.Roads[10001]={n={0,0,10,0},z={-10,-8},e={{1,2,10,0,0,0,10,0}}}''')
        self.write('Capitals','''ns.RoadOverlays[0]={{capital="capitals",n={20,0,30,0},
            nz={100,102},e={{1,2,10,0,20,0,30,0}}}}''')
        roads,_,_=self.load();city,capital=roads
        self.assertAlmostEqual(city.points[0].z,52.55)
        self.assertEqual(city.metadata['source_height_a'],-10)
        self.assertEqual(city.metadata['source_level'],10001)
        self.assertEqual(city.metadata['height_offset'],62.55)
        self.assertEqual(capital.points[0].z,100)
        self.assertEqual(capital.metadata['height_offset'],0)
    def test_floor_tracks_preserve_each_height_and_only_endpoint_portals(self):
        self.write('Roads','''ns.Roads={[0]={n={},e={}},[1]={n={},e={}}}
            ns.FloorTracks={[0]={{op="add",time=123,stair=true,pts={0,0,5,0,5,5},zs={0,2,7}}},[1]={}}''')
        roads,_,_=self.load();road=roads[0]
        self.assertEqual([p.z for p in road.points],[0,2,7])
        self.assertEqual(road.provenance['original_source'],6)
        self.assertEqual(road.metadata['special'],'floor_track')
        self.assertTrue(road.metadata['join_a']);self.assertTrue(road.metadata['join_b'])
        self.assertFalse(road.metadata['internal_bridge_allowed'])
        self.assertEqual(road.metadata['track_time'],123)
        self.assertEqual(road.metadata['height_a'],0)
        self.assertNotEqual(road.a,road.b)


@unittest.skipUnless((ROOT/'data/bundle.json').exists(),'Requires checked local reference data')
class InstalledSourceTopologyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.roads,cls.files,cls.excluded=source_roads(ROOT,DataBundle.load(ROOT/'data'))
    def test_all_drawn_roads_and_floor_tracks_are_imported(self):
        normal=[r for r in self.roads if r.provenance.get('file')=='Roads']
        self.assertEqual(Counter(r.provenance['original_source'] for r in normal),{0:1992,2:417})
        tracks=[r for r in self.roads if r.provenance.get('file')=='FloorTracks']
        self.assertEqual(len(tracks),8)
        self.assertTrue(all(r.a and r.b and r.metadata['point_heights'] for r in tracks))
    def test_all_source_edges_have_topology_and_portal_metadata(self):
        self.assertEqual(len(self.roads),29625)
        self.assertEqual(sum(r.provenance.get('file')=='Cities' for r in self.roads),3396)
        self.assertTrue(all(r.a and r.b and r.metadata.get('graph_id') for r in self.roads))
        self.assertEqual(sum(r.provenance['source']=='existing' for r in self.roads),6902)
        special=[r for r in self.roads if r.metadata['cave'] or r.metadata['indoor']]
        self.assertTrue(any(r.metadata['join_a'] or r.metadata['join_b'] for r in special))
        self.assertTrue(any(not r.metadata['bridge_allowed_a'] for r in special))


if __name__=='__main__':unittest.main()
