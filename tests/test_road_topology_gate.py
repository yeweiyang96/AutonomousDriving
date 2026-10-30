import copy
from functools import partial
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from adrive.models import Point,Route,Segment
from tools.check_road_topology import inspect_graph as inspect_production_graph,check

inspect_graph=partial(inspect_production_graph,required_sources=())


CASE=dict(name='fixture_junction',source_node_id='existing:1',source_entry_id='existing:0',source_exit_id='existing:2',
    source_roads=('road:1','road:2'),start=(0,0.,0.,0.),junction=(0,10.,0.,0.),goal=(0,20.,0.,0.),reference_length_xy=20.)


def graph():
    return dict(topology_version=2,policy=dict(shape_points_are_nodes=False,water=False),
        nodes=[[0,0.,0.,0.],[0,10.,0.,0.],[0,20.,0.,0.]],node_source_ids=['existing:0','existing:1','existing:2'],
        edges=[dict(id='g0',a=0,b=1,length=10.,kind='road',cost_factor=1,sources=['road:1'],raw_a='existing:0',raw_b='existing:1',bounds=[0.,10.,0.,0.]),
               dict(id='g1',a=1,b=2,length=10.,kind='road',cost_factor=1,sources=['road:2'],raw_a='existing:1',raw_b='existing:2',bounds=[10.,20.,0.,0.])])


class GraphTopologyGateTests(unittest.TestCase):
    def test_original_source_junction_and_degree_statistics(self):
        result=inspect_graph(graph(),cases=[CASE])
        self.assertEqual(result['status'],'passed',result['failures'])
        self.assertEqual(result['stats']['connected_components'],1)
        self.assertEqual(result['stats']['degree_histogram'],{'1':2,'2':1})
        self.assertTrue(all(r['connected'] for r in result['junctions'][0]['source_legs']))

    def test_components_are_reported_without_requiring_one_global_component(self):
        raw=graph();raw['nodes'] += [[1,0.,0.,0.],[1,10.,0.,0.]]
        raw['node_source_ids'] += ['agps:separate:0','agps:separate:1']
        raw['edges'].append(dict(id='other',a=3,b=4,length=10.,kind='road',cost_factor=1,sources=['agps:other'],
            raw_a='agps:separate:0',raw_b='agps:separate:1',bounds=[0.,10.,0.,0.]))
        result=inspect_graph(raw,cases=[CASE])
        self.assertEqual(result['status'],'passed',result['failures'])
        self.assertEqual(result['stats']['connected_components'],2)

    def test_isolated_pseudo_nodes_and_invalid_endpoints_are_rejected(self):
        raw=graph();raw['nodes'].append([0,100.,0.,0.]);raw['node_source_ids'].append('unused')
        self.assertIn('orphan_nodes',inspect_graph(raw,cases=())['failure_counts'])
        raw=graph();raw['edges'][0]['a']=99
        self.assertIn('edge_endpoint',inspect_graph(raw,cases=())['failure_counts'])

    def test_connector_and_road_weights_cannot_be_silently_interchanged(self):
        raw=graph();raw['edges'][0].update(kind='connector',cost_factor=1)
        self.assertIn('edge_weight',inspect_graph(raw,cases=())['failure_counts'])
        raw['edges'][0]['cost_factor']=2
        self.assertEqual(inspect_graph(raw,cases=())['status'],'passed')
        raw['edges'][1]['cost_factor']=2
        self.assertIn('edge_weight',inspect_graph(raw,cases=())['failure_counts'])

    def test_original_endpoint_and_source_identity_are_required(self):
        raw=graph();raw['edges'][0]['raw_b']='lost_original_node'
        self.assertIn('edge_raw_endpoint',inspect_graph(raw,cases=())['failure_counts'])
        raw=graph();raw['edges'][0]['sources']=[]
        self.assertIn('edge_source_trace',inspect_graph(raw,cases=())['failure_counts'])

    def test_replacement_sources_and_merged_node_aliases_retain_original_junction(self):
        raw=graph();raw['node_source_ids'][1]='agps:merged:1'
        raw['node_source_aliases']=[[],['existing:1'],[]]
        raw['edges'][1]['sources']=['agps:replacement']
        raw['edges'][1]['replacement_sources']=['road:2']
        self.assertEqual(inspect_graph(raw,cases=[CASE])['status'],'passed')

    def test_nearby_wrong_floor_cannot_satisfy_the_original_source_junction(self):
        raw=graph();raw['nodes'][1][3]=12.
        result=inspect_graph(raw,cases=[CASE])
        self.assertIn('junction_missing_floor',result['failure_counts'])

    def test_connector_cannot_hide_a_lost_original_road_leg(self):
        raw=graph();raw['edges'][1].update(kind='connector',cost_factor=2,sources=['connector:1:2'])
        result=inspect_graph(raw,cases=[CASE])
        self.assertIn('junction_source_disconnected',result['failure_counts'])

    def test_explicit_overlap_connector_may_preserve_a_traced_replacement_leg(self):
        raw=graph();raw['nodes'].append([0,1.,1.,0.]);raw['node_source_ids'].append('agps:replacement:1')
        raw['edges'][0].update(a=3,raw_a='agps:replacement:1',sources=['agps:replacement','road:1'],bounds=[1.,10.,0.,1.],
            metadata={'replacement_sources':['road:1']})
        raw['edges'].append(dict(id='entry',a=0,b=3,length=2.,kind='connector',cost_factor=2,
            sources=['connector:0:3','road:1'],raw_a='existing:0',raw_b='agps:replacement:1',bounds=[0.,1.,0.,1.],
            metadata={'replacement_sources':['road:1'],'connection_reason':'overlap_entry'}))
        self.assertEqual(inspect_graph(raw,cases=[CASE])['status'],'passed')
        raw['edges'][-1]['metadata']['connection_reason']='different_road_components'
        result=inspect_graph(raw,cases=[CASE])
        self.assertIn('connector_replacement_trace',result['failure_counts'])
        self.assertIn('junction_source_disconnected',result['failure_counts'])

    def test_duplicate_edge_ids_and_sample_node_topology_are_rejected(self):
        raw=graph();raw['edges'][1]['id']='g0'
        self.assertIn('edge_identity',inspect_graph(raw,cases=())['failure_counts'])
        raw=graph();raw['policy']['shape_points_are_nodes']=True
        self.assertIn('shape_point_topology',inspect_graph(raw,cases=())['failure_counts'])

    def test_original_closed_road_is_not_an_orphan_or_fake_edge(self):
        raw=graph();raw['nodes']=[[0,0.,0.,0.]];raw['node_source_ids']=['agps:loop:1']
        raw['edges']=[dict(id='loop',a=0,b=0,length=40.,kind='road',cost_factor=1,sources=['agps:loop'],
            raw_a='agps:loop:1',raw_b='agps:loop:1',bounds=[0.,10.,0.,10.])]
        result=inspect_graph(raw,cases=())
        self.assertEqual(result['status'],'passed',result['failures'])
        self.assertEqual(result['stats']['degree_histogram'],{'2':1})
        raw['edges'][0]['bounds']=[0.,0.,0.,0.]
        self.assertIn('edge_endpoint',inspect_graph(raw,cases=())['failure_counts'])
        raw['edges'][0].update(length=0.,bounds=[0.,10.,0.,10.])
        self.assertIn('edge_length',inspect_graph(raw,cases=())['failure_counts'])

    def test_known_complete_cave_source_cannot_be_deleted_to_pass_remaining_edges(self):
        raw=graph();result=inspect_production_graph(raw,cases=())
        self.assertIn('required_source_missing',result['failure_counts'])
        raw['edges'][0]['sources'].append('agps:Caves:0:0:1676')
        self.assertEqual(inspect_production_graph(raw,cases=())['status'],'passed')


class RouteTopologyGateTests(unittest.TestCase):
    def run_gate(self,corrupt=None):
        raw=graph();pack=SimpleNamespace(identity='fixture-nav',raw={'library':{'sha256':'fixture-dll'}},last_route_diagnostics=None)
        network=SimpleNamespace(raw=raw,identity='fixture-road')
        def plan(a,b):
            middle=Point(*CASE['junction'])
            pieces=(Segment((a,middle),'fixture','navmesh','nav:a:b'),Segment((middle,b),'fixture','navmesh','nav:b:c'))
            route=Route(a,b,pieces,20)
            pack.last_route_diagnostics={'route_selection':{'selected':'road_only'},'road_trace':[]}
            if corrupt=='coarse':route=Route(a,b,(Segment((a,b),'unverified','terrain','old-grid'),),20)
            elif corrupt=='endpoint':route=Route(Point(a.c,a.x+.1,a.y,a.z),b,pieces,20)
            elif corrupt=='fallback':pack.last_route_diagnostics['route_selection']['selected']='direct'
            return route
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory);manifest=path/'roads.json';manifest.write_text(json.dumps(raw))
            planner=SimpleNamespace(plan=plan)
            with patch('tools.check_road_topology.RoadPlanner',return_value=planner):
                result=check(path/'unused-nav.json',manifest,path/'report.json',pack=pack,network=network,cases=[CASE],required_sources=())
            self.assertTrue((path/'report.json').exists())
            return result

    def test_both_directions_require_complete_collision_paths(self):
        result=self.run_gate();self.assertEqual(result['status'],'passed',result['failures'])
        self.assertEqual(len(result['route_cases']),2);self.assertTrue(result['full_route_validation'])

    def test_old_terrain_fallback_changed_coordinates_and_open_fallback_fail(self):
        for corrupt in ('coarse','endpoint','fallback'):
            with self.subTest(corrupt=corrupt):
                result=self.run_gate(corrupt);self.assertEqual(result['status'],'failed')
                self.assertEqual(result['failure_count'],2)


if __name__=='__main__':unittest.main()
