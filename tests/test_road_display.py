"""Road artwork cannot load execution data or block view updates on it."""
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from adrive.data import DataBundle,RoadEdge
from adrive.models import Point
from adrive.road_display import RoadDisplayIndex,RoadDisplaySource,merge_lines,simplify

ROOT=Path(__file__).resolve().parents[1]


def wait_for(source,predicate):
    until=time.monotonic()+5
    while time.monotonic()<until:
        source.poll()
        if predicate():return
        time.sleep(.005)
    raise AssertionError('Road artwork did not complete')


def bundle(root=None):
    return DataBundle([Point(0,0,0),Point(0,10,0),Point(0,20,0)],
        [RoadEdge('first',0,1,'user_reviewed','walk','test'),RoadEdge('second',1,2,'road_proposal','walk','test')],{},asset_root=root)


class RoadArtworkTests(unittest.TestCase):
    def test_degree_two_chains_merge_without_deleting_junctions_or_floor_separation(self):
        roads=[dict(id=str(i),c=0,points=p) for i,p in enumerate([
            [(0,0),(10,0)],[(10,0),(20,0)],[(20,0),(30,0)],[(20,0),(20,10)],
            [(0,0,8),(10,0,8)],[(0,0,0),(10,0,0)]])]
        lines=merge_lines(roads)
        self.assertEqual(len(lines),5)
        self.assertTrue(any(line.points==((0.,0.),(10.,0.),(20.,0.)) for line in lines))
        self.assertEqual(sum(len(line.points)-1 for line in lines),6)

    def test_artwork_deduplicates_only_exact_same_geometry_and_preserves_connector_style(self):
        points=[(0,0),(10,0),(20,10)]
        lines=merge_lines([dict(c=0,points=points),dict(c=0,points=list(reversed(points))),
            dict(c=0,points=points,kind='connector')])
        self.assertEqual(len(lines),2)
        self.assertEqual({line.kind for line in lines},{'road','connector'})

    def test_spatial_query_uses_entire_curve_instead_of_endpoint_chord(self):
        index=RoadDisplayIndex([dict(c=0,points=[(0,0),(50,300),(100,0)])])
        frame=index.query(0,(45,55,290,310),20)
        self.assertEqual(len(frame.lines),1)
        self.assertEqual(index.query(1,(45,55,290,310),20).lines,())

    def test_simplification_keeps_endpoints_and_visible_bend(self):
        pts=((0,0),(5,.1),(10,0),(15,10),(20,0))
        line=simplify(pts,.5)
        self.assertEqual((line[0],line[-1]),(pts[0],pts[-1]))
        self.assertIn((15,10),line)
        self.assertNotIn((5,.1),line)

    def test_legacy_pointer_does_not_open_execution_manifest_or_collision_data(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'roads').mkdir()
            (root/'roads/manifest.json').write_text(json.dumps(dict(kind='adrive-road-pointer',path='missing-execution.json')))
            source=RoadDisplaySource(bundle(root))
            try:
                wait_for(source,lambda:not source.loading)
                self.assertEqual(source.origin,'original_bundle')
                self.assertEqual(source.error,'')
                self.assertEqual(source.line_count,1)
            finally:source.close()

    def test_published_artwork_isolated_worker_loads_without_an_execution_pack(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);directory=root/'roads';directory.mkdir()
            raw=dict(schema=1,kind='adrive-road-display',roads=[dict(id='only',c=1,kind='connector',points=[[0,0],[20,0]])])
            data=json.dumps(raw).encode();(directory/'display.json').write_bytes(data)
            (directory/'manifest.json').write_text(json.dumps(dict(kind='adrive-road-pointer',path='no-execution-graph',
                display=dict(path='display.json',bytes=len(data),sha256=hashlib.sha256(data).hexdigest()))))
            source=RoadDisplaySource(bundle(root))
            try:
                wait_for(source,lambda:not source.loading)
                self.assertEqual(source.origin,'published',source.error)
                source.request(SimpleNamespace(c=1,north=0.,west=0.,scale=1.,width=100,height=100))
                wait_for(source,lambda:source.frame is not None)
                self.assertEqual(source.frame.lines[0].kind,'connector')
                self.assertEqual(source.frame.lines[0].c,1)
            finally:source.close()

    def test_cached_overscan_does_not_repeat_queries_while_panning(self):
        source=RoadDisplaySource(bundle())
        try:
            wait_for(source,lambda:not source.loading)
            source._client.query=Mock(wraps=source._client.query)
            view=SimpleNamespace(c=0,north=0.,west=0.,scale=1.,width=100,height=100)
            source.request(view);wait_for(source,lambda:source.frame is not None)
            view.west=50.;source.request(view);source.poll()
            self.assertEqual(source._client.query.call_count,1)
            self.assertFalse(source.pending)
        finally:source.close()

    def test_closed_source_does_not_queue_more_work(self):
        source=RoadDisplaySource(bundle());source.close()
        source.request(SimpleNamespace(c=0,north=0.,west=0.,scale=1.,width=100,height=100))
        self.assertIsNone(source._query_future)

    def test_returning_to_cached_zoom_discards_inflight_stale_frame(self):
        source=RoadDisplaySource(bundle());release=threading.Event();entered=threading.Event()
        try:
            wait_for(source,lambda:not source.loading)
            view=SimpleNamespace(c=0,north=0.,west=0.,scale=1.,width=100,height=100)
            source.request(view);wait_for(source,lambda:source.frame is not None)
            cached=source.frame;original=source._client.query
            def delayed(*args):
                entered.set();release.wait(2);return original(*args)
            source._client.query=Mock(side_effect=delayed)
            view.scale=2.;source.request(view);self.assertTrue(entered.wait(2))
            view.scale=1.;source.request(view);release.set()
            wait_for(source,lambda:not source.pending)
            self.assertIs(source.frame,cached)
            self.assertEqual(source._client.query.call_count,1)
        finally:release.set();source.close()


if __name__=='__main__':unittest.main()
