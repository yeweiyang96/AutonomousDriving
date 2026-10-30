"""Source junctions and complete edges survive collision geometry refinement."""
from types import SimpleNamespace
import unittest
from adrive.models import Point,PlanningError
from adrive.road_sources import SourceRoad
from tools.build_topology_roads import GraphBuilder


def builder(floors=(0.,)):
    pack=SimpleNamespace(raw={'maps':[],'tiles':[]},read=lambda entry:b'')
    graph=GraphBuilder(pack)
    graph.validator.at=lambda points:SimpleNamespace(heights=lambda p:list(floors),project_road=lambda p:[])
    graph.queries=[]
    def query(a,b,reference,**kw):
        graph.queries.append((a,b,reference))
        if abs(a.z-b.z)>.1:raise PlanningError('no_route','Separate fixture floors')
        points=[Point(p.c,p.x,p.y,a.z if p.z is None else p.z) for p in reference]
        proof=dict(status='passed',expanded=1,corridor_width=3,entry_polygon='0,0,1',exit_polygon='0,0,1',output_length_xy=1)
        graph.expanded+=1;return points,proof
    graph.query=query
    return graph


class BuilderTopologyTests(unittest.TestCase):
    def test_dense_shape_vertices_are_not_graph_nodes(self):
        graph=builder();shape=[Point(0,i,0) for i in range(101)]
        graph.validate(SourceRoad('a',shape,{'source':'existing'},'n0','n1'))
        graph.validate(SourceRoad('b',[Point(0,100,0),Point(0,120,0)],{'source':'existing'},'n1','n2'))
        self.assertEqual((len(graph.nodes),len(graph.edges)),(3,2))
        self.assertEqual(graph.edges[0]['b'],graph.edges[1]['a'])
        self.assertEqual(len(graph.edges[0]['points']),101)
    def test_unknown_road_height_does_not_invent_linear_height_ramp(self):
        graph=builder();graph.validate(SourceRoad('road',[Point(0,0,0),Point(0,100,0)],{},'a','b'))
        first=graph.queries[0]
        self.assertIsNotNone(first[0].z)
        self.assertIsNone(first[2][0].z);self.assertIsNone(first[2][-1].z)
    def test_shared_junction_has_separate_connected_floor_variants(self):
        graph=builder((0.,10.))
        for identity,a,b,ka,kb in [('a',0,100,'n0','n1'),('b',100,120,'n1','n2')]:
            graph.validate(SourceRoad(identity,[Point(0,a,0),Point(0,b,0)],{},ka,kb))
        self.assertEqual((len(graph.nodes),len(graph.edges)),(6,4))
        for e in graph.edges:self.assertEqual(graph.nodes[e['a']].z,graph.nodes[e['b']].z)
        self.assertEqual(len({e['b'] for e in graph.edges[:2]} & {e['a'] for e in graph.edges[2:]}),2)
    def test_midroad_junction_splits_complete_edges_without_duplicate_parent(self):
        graph=builder();graph.validate(SourceRoad('road',[Point(0,0,0),Point(0,100,0)],{},'a','b'))
        parent=graph.edges[0];graph.split(parent,25);graph.split(parent,75)
        active=[e for e in graph.edges if e['active']]
        self.assertEqual(len(active),3);self.assertEqual(sum(e['length'] for e in active),100)
        self.assertEqual({e['display_id'] for e in active},{'road'})
        self.assertFalse(parent['active'])
    def test_zero_xy_distance_cannot_merge_different_floors(self):
        graph=builder();a=graph.node('a',Point(0,0,0,0));b=graph.node('b',Point(0,0,0,3))
        self.assertFalse(graph.connector(a,b,'fixture'))
    def test_terrain_grade_is_not_a_declared_floor_difference(self):
        graph=builder();a=graph.node('a',Point(0,0,0,323));b=graph.node('b',Point(0,72,0,330))
        self.assertTrue(graph.source_floor_compatible(a,b))
        graph.node_meta[a]['source_height']=323;graph.node_meta[b]['source_height']=330
        self.assertFalse(graph.source_floor_compatible(a,b))
    def test_closed_road_keeps_shape_as_one_graph_edge(self):
        graph=builder();shape=[Point(0,0,0),Point(0,100,0),Point(0,100,100),Point(0,0,0)]
        graph.validate(SourceRoad('loop',shape,{},'same','same'))
        self.assertEqual((len(graph.nodes),len(graph.edges)),(1,1))
        self.assertEqual(graph.edges[0]['a'],graph.edges[0]['b'])
    def test_cave_drawing_cannot_become_an_unanchored_terrain_road(self):
        graph=builder();meta=dict(cave=True,graph_id='cave',bridge_allowed_a=False,bridge_allowed_b=False)
        graph.validate(SourceRoad('cave',[Point(0,0,0),Point(0,100,0)],{},'a','b',meta))
        graph.bind_special_surfaces()
        self.assertFalse(any(e['active'] for e in graph.edges))
        self.assertEqual(graph.audit[0]['status'],'unbound')
    def test_cave_source_floor_or_declared_entrance_binds_its_component(self):
        for binding in (dict(bridge_allowed_a=True),dict(height_a=0.)):
            graph=builder();meta=dict(cave=True,graph_id='cave',bridge_allowed_a=False,bridge_allowed_b=False,**{})
            meta.update(binding)
            graph.validate(SourceRoad('cave',[Point(0,0,0),Point(0,100,0)],{},'a','b',meta))
            graph.bind_special_surfaces();self.assertTrue(all(e['active'] for e in graph.edges))


if __name__=='__main__':unittest.main()
