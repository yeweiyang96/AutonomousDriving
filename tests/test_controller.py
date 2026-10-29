from dataclasses import replace
import math
import unittest

from adrive.controller import Calibration, Controller
from adrive.models import Flags, Point, Route, Segment, Telemetry


def route(*coords):
    points = tuple(Point(0, x, y) for x, y in coords)
    segments = tuple(Segment((a, b), "fixture", "reviewed", f"edge-{i}") for i, (a, b) in enumerate(zip(points, points[1:])))
    return Route(points[0], points[-1], segments)


def sample(x=0.0, y=0.0, facing=0.0, now=10.0, seq=1, **kw):
    values = dict(session=7, seq=seq, c=0, ui_map=1429, x=x, y=y,
                  facing=facing, speed=7.0, flags=Flags.VALID, captured_at=now)
    values.update(kw)
    return Telemetry(**values)


def simulate(planned, initial_facing=0.0, speed_at=lambda step: 7.0, limit=3000):
    ctl = Controller(Calibration(math.pi, math.pi))
    ctl.set_route(planned)
    t = sample(planned.start.x, planned.start.y, initial_facing)
    trajectory = [t.point]
    decisions = []
    for step in range(limit):
        d = ctl.update(t, t.captured_at)
        decisions.append(d)
        if d.kind != "press":
            return ctl, t, trajectory, decisions
        elapsed = d.duration_ms / 1000
        heading, x, y = t.facing, t.x, t.y
        actual_speed = speed_at(step)
        if d.key == ctl.left_key:
            heading += math.pi * elapsed
        elif d.key == ctl.right_key:
            heading -= math.pi * elapsed
        else:
            x += math.cos(heading) * actual_speed * elapsed
            y += math.sin(heading) * actual_speed * elapsed
        t = sample(x, y, heading % math.tau, t.captured_at + elapsed + .02, t.seq + 1,
                   speed=actual_speed)
        assert ctl.on_action_complete(t, t.captured_at)
        trajectory.append(t.point)
    raise AssertionError("controller did not terminate")


class ControllerTests(unittest.TestCase):
    def controller(self, planned=None):
        c = Controller(Calibration(math.pi, math.pi))
        c.set_route(planned or route((0, 0), (100, 0)))
        return c

    def test_unknown_calibration_cannot_move(self):
        c = Controller()
        c.set_route(route((0, 0), (10, 0)))
        self.assertEqual(c.update(sample(), 10).kind, "stop")

    def test_waits_for_action_observation(self):
        c = self.controller()
        first = c.update(sample(), 10)
        self.assertEqual(first.kind, "press")
        self.assertEqual(c.update(sample(now=10.05, seq=2), 10.05).kind, "none")
        self.assertFalse(c.on_action_complete(sample(now=10.21, seq=1), 10.21))
        self.assertFalse(c.on_action_complete(sample(now=10.1, seq=2), 10.1))
        self.assertTrue(c.on_action_complete(sample(x=1.4, now=10.22, seq=2), 10.22))
        self.assertEqual(c.update(sample(x=1.4, now=10.22, seq=2), 10.22).kind, "press")

    def test_straight_arrival(self):
        ctl, t, points, ds = simulate(route((0, 0), (100, 0)))
        self.assertEqual(ctl.state, "arrived")
        self.assertLessEqual(t.point.distance(Point(0, 100, 0)), 2)
        self.assertTrue(all(abs(p.y) < .01 for p in points))
        self.assertTrue(all(20 <= d.duration_ms <= 200 for d in ds if d.kind == "press"))

    def test_two_right_angle_corners_preserved(self):
        planned = route((0, 0), (10, 0), (10, 10), (25, 10))
        ctl, t, points, _ = simulate(planned)
        self.assertEqual(ctl.state, "arrived")
        for corner in (Point(0, 10, 0), Point(0, 10, 10)):
            self.assertLess(min(p.distance(corner) for p in points), .61)
        self.assertLessEqual(t.point.distance(planned.goal), 2)

    def test_u_turn_does_not_finish_at_initial_goal(self):
        planned = route((0, 0), (10, 0), (0, 0))
        ctl, t, points, _ = simulate(planned)
        self.assertEqual(ctl.state, "arrived")
        self.assertGreater(max(p.x for p in points), 9.4)
        self.assertLessEqual(t.point.distance(planned.goal), 2)

    def test_facing_wrap_chooses_short_turn(self):
        c = self.controller(route((0, 0), (100, 12)))
        d = c.update(sample(facing=math.tau - .1), 10)
        self.assertEqual(d.key, c.left_key)
        self.assertLess(d.duration_ms, 100)

    def test_speed_change_finishes_without_large_overshoot(self):
        planned = route((0, 0), (40, 0), (40, 40))
        ctl, t, points, _ = simulate(planned, speed_at=lambda i: 7 if i < 8 else 14)
        self.assertEqual(ctl.state, "arrived")
        self.assertLessEqual(t.point.distance(planned.goal), 2)
        self.assertLess(max(p.x for p in points), 40.61)

    def test_many_short_edges_do_not_skip_to_final(self):
        coords = [(float(i), math.sin(i / 3) * 3) for i in range(50)]
        ctl, t, points, _ = simulate(route(*coords), limit=5000)
        self.assertEqual(ctl.state, "arrived")
        self.assertGreater(len(points), 50)
        self.assertEqual(ctl.waypoint_index, len(coords) - 2)

    def test_offroute_requests_replan_without_blocking_ground(self):
        c = self.controller()
        d = c.update(sample(y=16), 10)
        self.assertEqual(d.kind, "replan")
        self.assertIsNone(d.blocked_edge)
        self.assertEqual(c.replan_count, 1)

    def test_cross_track_correction_aligns_before_forward(self):
        c = self.controller()
        d = c.update(sample(x=10, y=4), 10)
        self.assertEqual(d.kind, "press")
        self.assertEqual(d.key, c.right_key)

    def test_stuck_blocks_edge_and_budget_survives_replanning(self):
        c = self.controller()
        t = sample()
        for attempt in range(3):
            while True:
                d = c.update(t, t.captured_at)
                if d.kind != "press":
                    break
                t = replace(t, seq=t.seq + 1, captured_at=t.captured_at + d.duration_ms / 1000 + .02)
                self.assertTrue(c.on_action_complete(t, t.captured_at))
            if attempt < 2:
                self.assertEqual(d.kind, "replan")
                self.assertEqual(d.blocked_edge, "edge-0")
                c.set_route(route((0, 0), (100, 0)), new_task=False)
            else:
                self.assertEqual(d.kind, "stop")
        self.assertEqual(c.replan_count, 2)
        self.assertIn("edge-0", c.blocked_edges)
        c.set_route(route((0, 0), (100, 0)))
        self.assertEqual(c.replan_count, 0)
        self.assertFalse(c.blocked_edges)

    def test_all_unsafe_flags_stop(self):
        for flag in Flags:
            if flag == Flags.VALID:
                continue
            with self.subTest(flag=flag):
                c = self.controller()
                self.assertEqual(c.update(sample(flags=Flags.VALID | flag), 10).kind, "stop")
        c = self.controller()
        self.assertEqual(c.update(sample(flags=Flags(0)), 10).kind, "stop")

    def test_stale_and_nonfinite_stop(self):
        for t in (sample(now=9.7), sample(x=math.nan), sample(facing=math.inf)):
            with self.subTest(t=t):
                self.assertEqual(self.controller().update(t, 10).kind, "stop")

    def test_session_change_during_action_stops(self):
        c = self.controller()
        c.update(sample(), 10)
        t = sample(now=10.22, seq=2, session=9)
        self.assertFalse(c.on_action_complete(t, 10.22))
        self.assertEqual(c.state, "fault")

    def test_discontinuous_route_rejected(self):
        a, b, g = Point(0, 0, 0), Point(0, 10, 0), Point(0, 30, 0)
        broken = Route(a, g, (Segment((a, b), "x", "reviewed", "1"),
                              Segment((Point(0, 20, 0), g), "x", "reviewed", "2")))
        with self.assertRaises(ValueError):
            self.controller(broken)

    def test_wrong_turn_keys_fail_instead_of_looping_forever(self):
        c = self.controller(route((0, 0), (0, 100)))
        t = sample()
        for _ in range(8):
            d = c.update(t, t.captured_at)
            self.assertEqual(d.kind, "press")
            t = replace(t, seq=t.seq + 1, captured_at=t.captured_at + d.duration_ms / 1000 + .02)
            self.assertTrue(c.on_action_complete(t, t.captured_at))
        self.assertEqual(c.update(t, t.captured_at).kind, "stop")

    def test_pause_is_latched(self):
        c = self.controller()
        c.pause("test")
        self.assertEqual(c.update(sample(), 10).kind, "none")

    def test_manual_resume_can_accept_reload_without_resetting_budget(self):
        c = self.controller()
        c.update(sample(), 10)
        c.pause("reload")
        c.replan_count = 1
        c.blocked_edges.add("old")
        c.reset_observation_session()
        c.set_route(route((0, 0), (100, 0)), new_task=False)
        self.assertEqual(c.update(sample(session=8), 10).kind, "press")
        self.assertEqual(c.replan_count, 1)
        self.assertIn("old", c.blocked_edges)

    def test_active_session_cannot_reset(self):
        c = self.controller()
        c.update(sample(), 10)
        with self.assertRaises(ValueError):
            c.reset_observation_session()

    def test_nonfinite_route_goal_rejected(self):
        for planned in (route((0, 0), (math.nan, 0)), route((math.inf, 0), (10, 0))):
            with self.assertRaises(ValueError):
                self.controller(planned)


if __name__ == "__main__":
    unittest.main()
