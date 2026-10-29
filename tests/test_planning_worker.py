import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

from adrive.data import DataBundle
from adrive.models import PlanningError, Point
from adrive.planning_worker import IsolatedPlanner


class IsolatedPlanningTests(unittest.TestCase):
    def test_roundtrip_preserves_original_endpoint(self):
        point = Point(0, 12.25, -8.5, 3.)
        route = IsolatedPlanner(DataBundle([], [], {})).plan(point, point)
        self.assertEqual(route.start, point)
        self.assertEqual(route.goal, point)
        self.assertFalse(route.segments)

    def test_planning_error_keeps_code(self):
        with self.assertRaises(PlanningError) as caught:
            IsolatedPlanner(DataBundle([], [], {})).plan(Point(0, 0, 0), Point(1, 0, 0))
        self.assertEqual(caught.exception.code, 'different_continent')

    def test_loads_exact_custom_bundle_path(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'custom.json'
            source.write_text(json.dumps(dict(schema=1, nodes=[], edges=[], terrain={}, tiles=[], map_boxes={})))
            bundle = DataBundle.load(source)
            point = Point(0, 1, 2)
            self.assertEqual(IsolatedPlanner(bundle).plan(point, point).goal, point)

    def test_cancel_reaps_unresponsive_worker(self):
        cancel = threading.Event()
        original = subprocess.Popen
        children = []
        def launch(*args, **kwargs):
            child = original([sys.executable, '-c', 'import time; time.sleep(60)'], **kwargs)
            children.append(child)
            cancel.set()
            return child
        with patch('adrive.planning_worker.subprocess.Popen', side_effect=launch):
            with self.assertRaises(PlanningError) as caught:
                IsolatedPlanner(DataBundle([], [], {})).plan(Point(0, 0, 0), Point(0, 1, 1), cancel=cancel)
        self.assertEqual(caught.exception.code, 'cancelled')
        self.assertIsNotNone(children[0].poll())

    def test_already_cancelled_never_spawns(self):
        cancel = threading.Event()
        cancel.set()
        with patch('adrive.planning_worker.subprocess.Popen') as spawn:
            with self.assertRaises(PlanningError):
                IsolatedPlanner(DataBundle([], [], {})).plan(Point(0, 0, 0), Point(0, 1, 1), cancel=cancel)
        spawn.assert_not_called()
