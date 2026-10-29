import math
import unittest

from adrive.models import Point, Route, Segment
from adrive.replay import Replay
from adrive.simulation import simulate


class ClosedLoopTests(unittest.TestCase):
    def test_pixels_to_controller_to_simulated_motion_and_replay(self):
        rows, result = simulate(use_pixels=True)
        self.assertTrue(result['arrived'])
        self.assertLessEqual(result['goal_error_yards'], 2)
        self.assertLess(result['maximum_cross_track_yards'], 1)
        summary = Replay(rows).summary()
        self.assertEqual(summary['final_state'], 'arrived')
        self.assertEqual(summary['actions'], result['actions'])
        self.assertLessEqual(summary['goal_error_yards'], 2)

    def test_quantized_reversal_with_short_segments(self):
        pts = [Point(0, 100, 100), Point(0, 110, 100), Point(0, 110, 103), Point(0, 100, 103)]
        route = Route(pts[0], pts[-1], tuple(Segment((a, b), 'test', 'terrain', str(i))
                    for i, (a, b) in enumerate(zip(pts, pts[1:]))))
        rows, result = simulate(route, speed_change=False)
        self.assertTrue(result['arrived'])
        positions = [Point(r['telemetry']['c'], r['telemetry']['x'], r['telemetry']['y'])
                     for r in rows if r['kind'] == 'telemetry']
        for corner in pts[1:-1]:
            self.assertLessEqual(min(p.distance(corner) for p in positions), .7)


if __name__ == '__main__':
    unittest.main()
