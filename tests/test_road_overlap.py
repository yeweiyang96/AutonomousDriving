import math
import unittest

from adrive.models import Point
from adrive.road_sources import SourceRoad
from adrive.road_overlap import OverlapIndex,find_overlap


def road(identity,points,a,b):
    return SourceRoad(identity,[Point(0,*p) for p in points],{},a,b,{})


class RoadOverlapTests(unittest.TestCase):
    def test_offset_polyline_matches_connected_split_chain_and_midroad_entries(self):
        preferred=[road('old:a',[(0,0,0),(50,0,0)],'n0','n1'),road('old:b',[(50,0,0),(100,0,0)],'n1','n2')]
        new=road('gps',[(10,2,0),(40,2,0),(90,2,0)],'gps:0','gps:1')
        match=find_overlap(new,preferred)
        self.assertIsNotNone(match)
        self.assertEqual(match['entry']['edge_id'],'old:a');self.assertAlmostEqual(match['entry']['along'],10)
        self.assertEqual(match['exit']['edge_id'],'old:b');self.assertAlmostEqual(match['exit']['along'],40)
        self.assertEqual([p['edge_id'] for p in match['path']],['old:a','old:b'])
        self.assertAlmostEqual(match['existing_length'],80);self.assertAlmostEqual(match['candidate_total_length'],84)
        self.assertTrue(match['keep_existing']);self.assertFalse(match['collision_verified'])
        self.assertEqual((new.a,new.b),('gps:0','gps:1'))

    def test_reversed_source_edge_orientation_is_preserved_in_path_intervals(self):
        old=road('reverse',[(100,0,0),(0,0,0)],'n100','n0')
        match=find_overlap(road('gps',[(10,1,0),(90,1,0)],'g0','g1'),[old])
        self.assertIsNotNone(match)
        self.assertAlmostEqual(match['path'][0]['from_along'],90)
        self.assertAlmostEqual(match['path'][0]['to_along'],10)

    def test_bridge_above_ground_is_not_an_overlap(self):
        old=road('ground',[(0,0,0),(100,0,0)],'n0','n1')
        self.assertIsNone(find_overlap(road('bridge',[(0,1,12),(100,1,12)],'b0','b1'),[old]))

    def test_interior_floor_mismatch_is_rejected_even_when_endpoints_match(self):
        old=road('ground',[(0,0,0),(100,0,0)],'n0','n1')
        self.assertIsNone(find_overlap(road('arch',[(0,0,0),(50,0,12),(100,0,0)],'b0','b1'),[old]))

    def test_nearby_unconnected_preferred_pieces_do_not_form_a_chain(self):
        old=[road('a',[(0,0,0),(45,0,0)],'n0','n1'),road('b',[(55,0,0),(100,0,0)],'n2','n3')]
        index=OverlapIndex(old)
        self.assertIsNone(index.find(road('gps',[(0,1,0),(100,1,0)],'g0','g1')))
        self.assertEqual(index.last_stats['reason'],'disconnected')

    def test_coincident_coordinates_do_not_replace_declared_topology(self):
        old=[road('a',[(0,0,0),(50,0,0)],'n0','n1'),road('b',[(50,0,0),(100,0,0)],'different','n3')]
        self.assertIsNone(find_overlap(road('gps',[(0,1,0),(100,1,0)],'g0','g1'),old))

    def test_missing_node_ids_do_not_implicitly_join_preferred_edges(self):
        old=[road('a',[(0,0,0),(50,0,0)],None,None),road('b',[(50,0,0),(100,0,0)],None,None)]
        self.assertIsNone(find_overlap(road('gps',[(0,1,0),(100,1,0)],'g0','g1'),old))

    def test_declared_junction_cannot_hide_a_known_vertical_discontinuity(self):
        old=[road('low',[(0,0,0),(50,0,0)],'n0','n1'),road('high',[(50,0,12),(100,0,12)],'n1','n2')]
        index=OverlapIndex(old)
        self.assertIsNone(index.find(road('unknown',[(0,1,None),(100,1,None)],'g0','g1')))
        self.assertEqual(index.last_stats['reason'],'junction_geometry_mismatch')

    def test_perpendicular_junction_is_not_a_duplicate_road(self):
        old=road('horizontal',[(-20,0,0),(20,0,0)],'n0','n1')
        self.assertIsNone(find_overlap(road('vertical',[(0,-4,0),(0,4,0)],'g0','g1'),[old]))

    def test_five_percent_replacement_threshold_compares_complete_geometry(self):
        def preferred(target):
            def line(h):return [(0,0,0),(25,h,0),(50,-h,0),(75,h,0),(100,0,0)]
            lo,hi=0.,8.
            for _ in range(60):
                mid=(lo+hi)/2;ps=line(mid)
                if sum(math.dist(a[:2],b[:2]) for a,b in zip(ps,ps[1:]))<target:lo=mid
                else:hi=mid
            return road('old',line((lo+hi)/2),'n0','n1')
        new=road('straight',[(0,0,0),(100,0,0)],'g0','g1')
        insufficient=find_overlap(new,[preferred(105.)]);enough=find_overlap(new,[preferred(100/.95)])
        self.assertIsNotNone(insufficient);self.assertIsNotNone(enough)
        self.assertTrue(insufficient['keep_existing']);self.assertFalse(enough['keep_existing'])

    def test_spatial_index_excludes_distant_graphs_and_matching_has_a_budget(self):
        old=[road('local:a',[(0,0,0),(50,0,0)],'n0','n1'),road('local:b',[(50,0,0),(100,0,0)],'n1','n2')]
        old.extend(road(f'far:{i}',[(10000+i*30,10000,0),(10010+i*30,10000,0)],f'f{i}a',f'f{i}b') for i in range(3000))
        new=road('gps',[(0,1,0),(100,1,0)],'g0','g1')
        index=OverlapIndex(old);self.assertIsNotNone(index.find(new));self.assertEqual(index.last_stats['candidate_roads'],2)
        limited=OverlapIndex(old,max_expansions=1)
        self.assertIsNone(limited.find(new));self.assertEqual(limited.last_stats['reason'],'budget')


if __name__=='__main__':unittest.main()
