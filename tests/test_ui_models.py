"""Headless checks of desktop coordinate/configuration/replay behavior."""
from __future__ import annotations

from dataclasses import asdict
import json
import math
from pathlib import Path
import random
import queue
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from adrive.app import ACTIVE_STATES, Application
from adrive.controller import Calibration, Controller
from adrive.data import DataBundle
from adrive.eventlog import EventLog, json_value
from adrive.mapview import MapView, Viewport
from adrive.models import Flags, Point, Route, Segment, Telemetry
from adrive.planner import Planner
from adrive.replay import Replay, route_from_dict, telemetry_from_dict
from adrive.settings import Settings, key_code


CALIBRATION = {"left_rate_rad_s": 2.5, "right_rate_rad_s": 2.6, "input_latency_s": .03}


def temporary_workspace():
    workspace = Path(__file__).resolve().parents[1]
    folder = (workspace / 'runtime' / 'test-ui').resolve()
    if not folder.is_relative_to(workspace):
        raise RuntimeError('Test temporary folder escaped the workspace')
    folder.mkdir(parents=True, exist_ok=True)
    return tempfile.TemporaryDirectory(dir=folder, prefix='ui-')


def telemetry(x=0., y=0., *, session=5, seq=1, flags=Flags.VALID):
    return Telemetry(session, seq, 0, 1429, x, y, 0., 7., flags, captured_at=1.)


def route():
    a, b = Point(0, 0., 0.), Point(0, 10., 0.)
    return Route(a, b, (Segment((a, b), "fixture", "reviewed", "road:fixture"),), 3)


class ViewportTests(unittest.TestCase):
    def test_north_west_orientation_and_roundtrip_both_continents(self):
        rng = random.Random(41)
        for c in (0, 1):
            for scale in (.012, .04, .25, 1., 4.):
                view = Viewport(c, -9000., 1500., scale, 1053, 713)
                cx, cy = view.screen(Point(c, view.north, view.west))
                self.assertEqual((cx, cy), (1053 / 2, 713 / 2))
                self.assertLess(view.screen(Point(c, view.north + 100, view.west))[1], cy)
                self.assertLess(view.screen(Point(c, view.north, view.west + 100))[0], cx)
                for _ in range(30):
                    p = Point(c, rng.uniform(-16000., 15000.), rng.uniform(-10000., 10000.))
                    result = view.world(*view.screen(p))
                    self.assertEqual(result.c, c)
                    self.assertAlmostEqual(result.x, p.x, places=8)
                    self.assertAlmostEqual(result.y, p.y, places=8)

    def test_zoom_keeps_pointer_world_anchor_even_at_limits(self):
        view = Viewport(1, 1700., -4200., .25, 1100, 730)
        for factor, px, py in ((1.25, 50., 90.), (.8, 1000., 650.), (10000., 200., 200.), (1e-8, 830., 100.)):
            before = view.world(px, py)
            view.zoom(factor, px, py)
            after = view.world(px, py)
            self.assertAlmostEqual(before.x, after.x)
            self.assertAlmostEqual(before.y, after.y)
            self.assertGreaterEqual(view.scale, .012)
            self.assertLessEqual(view.scale, 4.)

    def test_resize_preserves_world_centre(self):
        view = Viewport(0, -9458.437, 50.658, .5)
        view.width, view.height = 1600, 900
        self.assertEqual(view.world(800, 450), Point(0, -9458.437, 50.658))

    def test_long_track_draw_is_one_polyline_without_dropping_points(self):
        view = Viewport(0, 0, 0, 1.)
        track = [Point(0, i * 1.1, 0) for i in range(1500)]
        canvas = SimpleNamespace(view=view, track=track, pose=None, delete=Mock(), create_line=Mock())
        MapView.draw_pose(canvas)
        canvas.create_line.assert_called_once()
        self.assertEqual(canvas.create_line.call_args.args,
                         tuple(v for p in track for v in view.screen(p)))

    def test_track_polyline_keeps_teleport_and_continent_breaks(self):
        track = [Point(0, 0, 0), Point(0, 1, 0), Point(0, 1000, 0), Point(0, 1001, 0),
                 Point(1, 1002, 0), Point(0, 1003, 0), Point(0, 1004, 0)]
        view = Viewport(0, 0, 0, 1.)
        canvas = SimpleNamespace(view=view, track=track, pose=None, delete=Mock(), create_line=Mock())
        MapView.draw_pose(canvas)
        self.assertEqual(canvas.create_line.call_count, 3)
        for call, indices in zip(canvas.create_line.call_args_list, ((0, 1), (2, 3), (5, 6))):
            self.assertEqual(call.args, tuple(v for i in indices for v in view.screen(track[i])))

    def test_ui_batch_keeps_telemetry_but_does_not_redraw_every_frame(self):
        app = Application.__new__(Application)
        app.closed, app.events = False, queue.Queue()
        app.map, app.root = Mock(), Mock()
        samples = [telemetry(i, seq=i + 1) for i in range(250)]
        for t in samples:
            app.events.put({'kind': 'telemetry', 'telemetry': t})
        app.poll()
        self.assertEqual(app.map.update_pose.call_count, 200)
        app.map.draw_pose.assert_called_once()
        self.assertEqual(app.events.qsize(), 50)
        self.assertEqual(app.latest.seq, 200)
        app.poll()
        self.assertEqual(app.map.update_pose.call_count, 250)
        self.assertEqual(app.map.draw_pose.call_count, 2)
        self.assertEqual(app.latest.seq, 250)


class SettingsTests(unittest.TestCase):
    def test_key_usage_codes_and_normalisation(self):
        self.assertEqual([key_code(k) for k in ("W", "a", " D ", "UP", "LEFT")], [26, 4, 7, 82, 80])
        for value in ("", "F12", "CTRL", "AA"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                key_code(value)

    def test_validation_and_roundtrip_with_real_calibration(self):
        settings = Settings(forward_key="UP", left_key="LEFT", right_key="RIGHT", calibration=CALIBRATION.copy())
        settings.validate()
        with temporary_workspace() as directory:
            path = Path(directory) / "nested/config.json"
            settings.save(path)
            self.assertEqual(Settings.load(path), settings)
            self.assertFalse(path.with_suffix('.tmp').exists())
            raw = json.loads(path.read_text(encoding='utf-8'))
            raw['future_option'] = 'ignored'
            path.write_text(json.dumps(raw), encoding='utf-8')
            self.assertEqual(Settings.load(path), settings)
        self.assertEqual(Calibration(**settings.calibration), Calibration(2.5, 2.6, .03))

    def test_invalid_binding_and_endpoint_rejected(self):
        for changes in ({'left_key': 'W'}, {'host': ' '}, {'port': 0}, {'port': 65536}, {'window_title': ' '}, {'right_key': 'F12'}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                Settings(**changes).validate()

    def test_malformed_json_types_are_diagnostic_value_errors(self):
        for changes in ({'host': 123}, {'port': '8888'}, {'port': True}, {'window_title': None}, {'left_key': None}, {'allow_candidates': 'false'}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                Settings(**changes).validate()

    def test_bad_calibration_cannot_be_saved_as_valid(self):
        for calibration in ({}, [], {**CALIBRATION, 'left_rate_rad_s': 0.}, {**CALIBRATION, 'right_rate_rad_s': math.nan}, {**CALIBRATION, 'input_latency_s': -1.}, {**CALIBRATION, 'left_rate_rad_s': 'fast'}):
            with self.subTest(calibration=calibration), self.assertRaises(ValueError):
                Settings(calibration=calibration).validate()

    def test_apply_binding_or_window_change_invalidates_old_calibration(self):
        for keys, window, expect_clear in ((('W', 'A', 'D'), 'World of Warcraft', False),
                                           (('UP', 'LEFT', 'RIGHT'), 'World of Warcraft', True),
                                           (('W', 'A', 'D'), 'Forever', True)):
            with self.subTest(keys=keys, window=window):
                app = Application.__new__(Application)
                app.settings = Settings(calibration=CALIBRATION.copy())
                app.runtime = Mock()
                app.device_diagnostic_var = Mock()
                app.key_vars = [Mock(get=Mock(return_value=k)) for k in keys]
                app.window_var = Mock(get=Mock(return_value=window))
                app.host_var = Mock(get=Mock(return_value=app.settings.host))
                app.port_var = Mock(get=Mock(return_value=str(app.settings.port)))
                app.candidates_var = Mock(get=Mock(return_value=False))
                with patch.object(Settings, 'save'):
                    new = app._apply_settings()
                self.assertEqual(new.calibration is None, expect_clear)
                if expect_clear:
                    app.runtime.command.assert_called_once_with('configure', settings=new)
                else:
                    app.runtime.command.assert_not_called()


class ActiveUiGuardsTests(unittest.TestCase):
    def test_pending_and_live_execution_cannot_be_relabelled_by_preview_edits(self):
        for state in ACTIVE_STATES:
            with self.subTest(state=state):
                app = Application.__new__(Application)
                app.state = state
                app.log = Mock()
                app.map = Mock()
                app.pick(Point(0, 30., 40.))
                app.clear_start()
                app.preview()
                self.assertEqual(app.log.call_count, 3)
                self.assertEqual(app.map.mock_calls, [])


class ReplayTests(unittest.TestCase):
    def write_rows(self, path: Path, rows):
        path.write_text('\n'.join(json.dumps(row) for row in rows) + '\n', encoding='utf-8')

    def test_real_eventlog_roundtrip_and_measured_summary(self):
        with temporary_workspace() as directory:
            log = EventLog(Path(directory))
            planned = route()
            log.write('session', settings={'host': 'test'})
            log.write('route', route=planned)
            log.write('telemetry', telemetry=telemetry(0., seq=1))
            log.write('action', key=26, duration_ms=100)
            log.write('telemetry', telemetry=telemetry(3., seq=2))
            log.write('action', key=26, duration_ms=100)
            log.write('telemetry', telemetry=telemetry(10., seq=3))
            log.write('state', state='arrived', reason='test completion')
            log.close()
            replay = Replay.load(log.path)
            summary = replay.summary()
        self.assertEqual(summary, {'frames': 3, 'route_revisions': 1, 'actions': 2, 'tracked_yards': 10.,
                                   'final_state': 'arrived', 'final_reason': 'test completion', 'goal_error_yards': 0.})
        self.assertEqual(route_from_dict(next(r['route'] for r in replay.rows if r['kind'] == 'route')), planned)
        self.assertEqual(telemetry_from_dict(next(r['telemetry'] for r in replay.rows if r['kind'] == 'telemetry')), telemetry())

    def test_empty_and_continent_transition_do_not_invent_distance(self):
        self.assertEqual(Replay([]).summary()['tracked_yards'], 0)
        a, b = json_value(telemetry()), json_value(telemetry(2., seq=2))
        b['c'] = 1
        replay = Replay([{'kind': 'telemetry', 'telemetry': a}, {'kind': 'telemetry', 'telemetry': b}, {'kind': 'route', 'route': json_value(route())}])
        self.assertEqual(replay.summary()['tracked_yards'], 0)
        self.assertIsNone(replay.summary()['goal_error_yards'])

    def test_invalid_frames_and_session_resets_break_track_continuity(self):
        for middle in (telemetry(40., seq=2, flags=Flags.LOADING), telemetry(40., session=6, seq=1)):
            poses = [telemetry(0., seq=1), middle, telemetry(1., seq=3)]
            replay = Replay([{'kind': 'telemetry', 'telemetry': json_value(t)} for t in poses])
            self.assertEqual(replay.summary()['tracked_yards'], 0.)

    def test_parse_errors_include_line_number(self):
        with temporary_workspace() as directory:
            path = Path(directory) / 'bad.jsonl'
            for second in ('no-json', '[]', '{}'):
                path.write_text('{"kind":"session","t":0}\n' + second + '\n', encoding='utf-8')
                with self.subTest(second=second), self.assertRaisesRegex(ValueError, '2'):
                    Replay.load(path)

    def test_event_time_and_payload_rejected_before_async_playback(self):
        with temporary_workspace() as directory:
            path = Path(directory) / 'bad.jsonl'
            cases = [{'kind': 'state', 't': 'bad', 'state': 'paused'},
                     {'kind': 'state', 't': math.nan, 'state': 'paused'},
                     {'kind': 'telemetry', 't': .1},
                     {'kind': 'telemetry', 't': .1, 'telemetry': {**asdict(telemetry()), 'x': math.nan}},
                     {'kind': 'route', 't': .1, 'route': {'start': {'c': 0}}}]
            for bad in cases:
                self.write_rows(path, [{'kind': 'session', 't': 0}, bad])
                with self.subTest(bad=bad), self.assertRaisesRegex(ValueError, '2'):
                    Replay.load(path)


class PlannerControllerBoundaryTests(unittest.TestCase):
    def test_identical_endpoints_arrive_without_input_even_without_terrain(self):
        point = Point(0, -9458.437, 50.658)
        planned = Planner(DataBundle([], [], {})).plan(point, point)
        self.assertEqual(planned.segments, ())
        self.assertEqual(planned.distance, 0.)
        controller = Controller(Calibration(**CALIBRATION))
        controller.set_route(planned)
        pose = Telemetry(5, 1, 0, 1429, point.x, point.y, 0., 0., Flags.VALID, 1.)
        decision = controller.update(pose, 1.)
        self.assertEqual(decision.kind, 'arrived')
        self.assertIsNone(decision.key)
        self.assertIsNone(controller.pending)

    def test_real_planner_routes_pass_controller_continuity_and_keep_source_status(self):
        root = Path(__file__).resolve().parents[1]
        bundle = DataBundle.load(root / 'data')
        bundle.asset_root = None  # Verify original source labels on the legacy backend.
        source = {e.id: e for e in bundle.edges}
        for a, b, candidates in ((3766, 3908, False), (1108, 512, True)):
            planned = Planner(bundle).plan(bundle.nodes[a], bundle.nodes[b], allow_candidates=candidates)
            Controller(Calibration(**CALIBRATION)).set_route(planned)
            self.assertLess(planned.points[0].distance(planned.start), 1e-6)
            self.assertLess(planned.points[-1].distance(planned.goal), 1e-6)
            for segment in planned.segments:
                if segment.edge_id.startswith('road:'):
                    edge = source[segment.edge_id]
                    expected = 'reviewed' if edge.status in ('user_reviewed', 'user_correction') else 'candidate'
                    self.assertEqual(segment.reliability, expected)
                    self.assertEqual(segment.source, edge.source)
                    self.assertEqual(edge.kind, 'walk')


if __name__ == '__main__':
    unittest.main()
