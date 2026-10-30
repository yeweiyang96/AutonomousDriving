from collections import Counter
from types import MethodType,SimpleNamespace
import unittest

from adrive.models import Point
from adrive.road_sources import SourceRoad
from adrive.roads import length
from tools.build_topology_roads import GraphBuilder


def builder():
    pack=SimpleNamespace(raw={'maps':[],'tiles':[]},read=lambda row:b'')
    graph=GraphBuilder(pack)
    def query(self,a,b,reference,budget=32768,connector=False):
        points=[a,b] if connector else reference
        return points,dict(status='passed',entry_polygon='0,0,1',exit_polygon='0,0,1',
            expanded=1,corridor_width=3.,output_length_xy=length(points))
    graph.query=MethodType(query,graph)
    return graph


def emit(graph,identity,points,a,b,source='AzerothGPS',special='capital'):
    points=[Point(0,*p) for p in points]
    road=SourceRoad(identity,points,{'source':source},a,b,{'special':special})
    proof=dict(status='passed',entry_polygon='0,0,1',exit_polygon='0,0,1',expanded=1,corridor_width=3.)
    return graph.emit(road,points[0],points[-1],points,proof,proof)


class WholeRoadDeduplicationTests(unittest.TestCase):
    def test_old_city_edge_is_replaced_by_complete_verified_special_chain(self):
        graph=builder()
        old=emit(graph,'road:1',[(0,0,0),(100,0,0)],'existing:0','existing:1','existing','normal')
        emit(graph,'agps:a',[(0,2,0),(50,2,0)],'agps:n0','agps:n1')
        emit(graph,'agps:b',[(50,2,0),(100,2,0)],'agps:n1','agps:n2')
        graph.deduplicate()
        self.assertFalse(old['active'])
        kept=[e for e in graph.edges if e['active'] and e['kind']=='road']
        self.assertEqual(len(kept),2)
        self.assertTrue(all('road:1' in e['sources'] and 'road:1' in e['metadata']['replacement_sources'] for e in kept))
        connectors=[e for e in graph.edges if e['active'] and e['kind']=='connector']
        self.assertEqual(len(connectors),2)
        self.assertTrue(all(e['cost_factor']==2 and 'road:1' in e['metadata']['replacement_sources'] for e in connectors))
        self.assertEqual({e['metadata']['connection_reason'] for e in connectors},{'overlap_entry','overlap_exit'})
        self.assertEqual(graph.counts['existing_edges_canonicalized_to_special_roads'],1)

    def test_midroad_replacement_updates_only_active_interval_children(self):
        graph=builder()
        old=emit(graph,'road:1',[(20,2,0),(80,2,0)],'existing:0','existing:1','existing','normal')
        parent=emit(graph,'agps:long',[(0,0,0),(100,0,0)],'agps:n0','agps:n1')
        graph.deduplicate()
        self.assertFalse(old['active']);self.assertFalse(parent['active'])
        children=[e for e in graph.edges if e['active'] and e['kind']=='road']
        self.assertEqual(len(children),3)
        traced=[e for e in children if 'road:1' in e['sources']]
        self.assertEqual(len(traced),1)
        self.assertAlmostEqual(traced[0]['length'],60.)
        self.assertIn('road:1',traced[0]['metadata']['replacement_sources'])

    def test_known_upper_bridge_does_not_replace_lower_old_road(self):
        graph=builder()
        old=emit(graph,'road:1',[(0,0,0),(100,0,0)],'existing:0','existing:1','existing','normal')
        emit(graph,'agps:bridge',[(0,1,12),(100,1,12)],'agps:n0','agps:n1')
        graph.deduplicate()
        self.assertTrue(old['active'])
        self.assertEqual(graph.counts['existing_edges_canonicalized_to_special_roads'],0)

    def test_disconnected_nearby_special_pieces_do_not_replace_old_corridor(self):
        graph=builder()
        old=emit(graph,'road:1',[(0,0,0),(100,0,0)],'existing:0','existing:1','existing','normal')
        emit(graph,'agps:a',[(0,1,0),(45,1,0)],'agps:n0','agps:n1')
        emit(graph,'agps:b',[(55,1,0),(100,1,0)],'agps:n2','agps:n3')
        graph.deduplicate()
        # The forward pass may split the retained old road to absorb the two
        # disconnected overlay pieces, but its full original geometry survives.
        self.assertAlmostEqual(sum(e['length'] for e in graph.edges if e['active'] and e['kind']=='road'
            and e['provenance'][0]['source']=='existing'),100.)
        self.assertEqual(graph.counts['existing_edges_canonicalized_to_special_roads'],0)

    def test_failed_native_endpoint_connection_keeps_original_road(self):
        graph=builder()
        old=emit(graph,'road:1',[(0,0,0),(100,0,0)],'existing:0','existing:1','existing','normal')
        emit(graph,'agps:a',[(0,2,0),(100,2,0)],'agps:n0','agps:n1')
        graph.connector=MethodType(lambda self,*args,**kwargs:False,graph)
        graph.deduplicate()
        self.assertTrue(old['active'])

    def test_same_anchor_merge_preserves_old_node_aliases_on_retained_graph(self):
        graph=builder()
        old=emit(graph,'road:1',[(0,0,0),(100,0,0)],'existing:0','existing:1','existing','normal')
        emit(graph,'agps:a',[(0,0,0),(100,0,0)],'agps:n0','agps:n1')
        graph.deduplicate()
        self.assertFalse(old['active'])
        kept=next(e for e in graph.edges if e['active'] and e['kind']=='road')
        self.assertIn('existing:0',graph.node_aliases[kept['a']]);self.assertIn('agps:n0',graph.node_aliases[kept['a']])
        self.assertIn('existing:1',graph.node_aliases[kept['b']]);self.assertIn('agps:n1',graph.node_aliases[kept['b']])

    def test_dedicated_floor_binding_excludes_other_static_surface_variants(self):
        graph=builder()
        correct=emit(graph,'road:1',[(0,0,0),(100,0,0)],'existing:0','existing:1','existing','normal')
        unsupported=emit(graph,'road:1',[(0,0,-80),(100,0,-80)],'existing:0','existing:1','existing','normal')
        emit(graph,'agps:a',[(0,2,0),(100,2,0)],'agps:n0','agps:n1')
        graph.deduplicate()
        self.assertFalse(correct['active']);self.assertFalse(unsupported['active'])
        self.assertEqual(graph.counts['unbound_surface_variants'],1)
        row=next(r for r in graph.audit if r['id']=='road:1')
        self.assertEqual(row['excluded_surface_variants'][0]['code'],'unbound_surface_variant')
        self.assertEqual(row['excluded_surface_variants'][0]['entry']['z'],-80)
        self.assertTrue(any(e['active'] and 'road:1' in e['sources'] for e in graph.edges))


if __name__=='__main__':unittest.main()
