"""Permanent recorded-start regression using real production Detour surfaces."""
from pathlib import Path
import json
import math
import unittest

from adrive.replay import route_from_dict
from tests.motion_simulation import ROOT, run_recorded_case


FIXTURE = Path(__file__).parent/'fixtures/motion_recorded_start.json'


class RecordedMotionFixtureTests(unittest.TestCase):
    def test_fixture_preserves_recorded_prefix_and_measured_bindings(self):
        fixture = json.loads(FIXTURE.read_text(encoding='utf-8'))
        route = route_from_dict(fixture['route'])
        self.assertGreaterEqual(route.distance, 60.)
        self.assertLess(route.distance, 70.)
        self.assertEqual(route.start, route.points[0])
        self.assertEqual(route.goal, route.points[-1])
        self.assertEqual(len(route.segments), 15)
        for a, b in zip(route.segments, route.segments[1:]):
            self.assertEqual(a.points[-1], b.points[0])
        self.assertEqual(fixture['provenance']['source_log'], '20261030-113853-682916.jsonl')
        self.assertAlmostEqual(fixture['provenance']['telemetry_event_s'], 47.7252019)
        self.assertEqual(fixture['navigation_replay_baseline']['identity'], '7a4606dd9c967264')
        self.assertEqual(len(fixture['navigation_replay_baseline']['native_library_sha256']), 64)
        self.assertEqual(tuple(fixture['bindings'].values()), (26, 20, 8, 4, 7, 84))
        self.assertAlmostEqual(fixture['telemetry']['facing'], 3.167095084188392)
        profile = fixture['calibration']
        self.assertNotEqual(profile['calibration']['left_rate_rad_s'], profile['calibration']['right_rate_rad_s'])
        self.assertNotEqual(profile['movement_calibration']['strafe_left_angle'], math.pi/2)


class RecordedMotionProductionTests(unittest.TestCase):
    def _assert_continuous_start(self, initial_walking):
        manifest = ROOT/'data/navigation/manifest.json'
        if not manifest.exists():
            self.skipTest('Production navigation manifest is absent; recorded Detour regression needs that pack.')
        # An existing but malformed, incompatible or incomplete pack must fail.
        # SurfaceValidator/NavMeshPack verify the actual library and loaded tiles;
        # no broad exception handler turns integrity or geometry errors into skip.
        fixture = json.loads(FIXTURE.read_text(encoding='utf-8'))
        result = run_recorded_case(fixture, walk_speed=5., max_step=.005, target_distance=40., initial_walking=initial_walking)
        summary = {name: result[name] for name in (
            'outcome', 'initial_walking', 'first_failure', 'travelled_yards', 'max_lateral_error', 'max_lateral_after_join',
            'full_actual_trace_certified', 'certified_chords', 'windows', 'max_window_ms',
            'expired_windows', 'pivot_windows', 'release_windows', 'game_walk_toggles',
            'distinct_one_shot_events', 'source_sha256', 'source_sha256_after')}
        diagnostic = json.dumps(summary, ensure_ascii=False)
        self.assertEqual(result['source_sha256'], result['source_sha256_after'], diagnostic)
        self.assertEqual(result['first_response']['kind'], 'window', diagnostic)
        self.assertEqual(result['outcome'], 'passed', diagnostic)
        self.assertGreaterEqual(result['travelled_yards'], 40., diagnostic)
        self.assertTrue(result['full_actual_trace_certified'], diagnostic)
        self.assertGreater(result['certified_chords'], 100, diagnostic)
        self.assertGreater(result['lateral_samples'], 100, diagnostic)
        # Startup alignment is reported separately; settled route following
        # retains the 0.3-yard target, and ALL actual traces require a sweep.
        self.assertLessEqual(result['max_lateral_after_join'], .30, diagnostic)
        self.assertLessEqual(result['max_window_ms'], 200, diagnostic)
        self.assertEqual(result['expired_windows'], 0, diagnostic)
        self.assertEqual(result['pivot_windows'], 0, diagnostic)
        self.assertEqual(result['release_windows'], 0, diagnostic)
        expected_toggles = 0 if initial_walking else 1
        self.assertEqual(result['game_walk_toggles'], expected_toggles, diagnostic)
        self.assertEqual(result['distinct_one_shot_events'], expected_toggles, diagnostic)

    def test_running_mode_starts_and_traverses_forty_yards(self):
        self._assert_continuous_start(initial_walking=False)

    def test_already_walking_does_not_toggle_back_to_running(self):
        self._assert_continuous_start(initial_walking=True)


if __name__ == '__main__':
    unittest.main()
