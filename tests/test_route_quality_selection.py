"""Global road preference checks; these never control a device or the game."""
from __future__ import annotations

import math
import threading
import unittest
from unittest.mock import Mock

from adrive.models import PlanningError, Point, Route, Segment
from adrive.navmesh import NavMeshPack


class RoadRouteSelectionTests(unittest.TestCase):
    def setUp(self):
        self.start = Point(0, 0.0, 0.0, 5.0)
        self.goal = Point(0, 700.0, 0.0, 7.0)
        self.pack = object.__new__(NavMeshPack)
        self.guided = self.route(7500.0, expanded=123)
        self.guide = self.route(7580.0, reliability="reviewed")
        self.direct = self.route(1000.0, expanded=17)
        self.pack._plan_geometry = Mock(return_value=self.direct)

    def route(self, length, *, expanded=0, reliability="navmesh"):
        # A symmetric dogleg gives the requested distance without assuming
        # the selection policy has a particular representation of a route.
        middle = Point(0, 350.0, math.sqrt((length / 2) ** 2 - 350.0 ** 2), 6.0)
        segment = Segment((self.start, middle, self.goal), "fixture", reliability,
                          "road:fixture" if reliability == "reviewed" else "nav:fixture")
        return Route(self.start, self.goal, (segment,), expanded)

    def select(self, *, route=None, guide=None, blocked=frozenset(), cancel=None,
               budget=500000):
        diagnostic = {"road_trace": []}
        selected = self.pack._select_road_route(
            route or self.guided, self.guide if guide is None else guide,
            blocked, cancel, budget, diagnostic)
        return selected, diagnostic["route_selection"]

    def test_ordinary_guide_does_not_trigger_probe_even_if_native_route_is_long(self):
        selected, decision = self.select(guide=self.route(1000.0, reliability="reviewed"))
        self.assertIs(selected, self.guided)
        self.assertFalse(decision["triggered"])
        self.pack._plan_geometry.assert_not_called()

    def test_no_guide_does_not_run_a_duplicate_direct_query(self):
        diagnostic = {"road_trace": []}
        selected = self.pack._select_road_route(
            self.guided, None, frozenset(), None, 500000, diagnostic)
        self.assertIs(selected, self.guided)
        self.assertFalse(diagnostic["route_selection"]["triggered"])
        self.pack._plan_geometry.assert_not_called()

    def test_short_guided_result_does_not_trigger_probe_for_long_coarse_guide(self):
        route = self.route(1000.0, expanded=123)
        selected, decision = self.select(route=route)
        self.assertIs(selected, route)
        self.assertFalse(decision["triggered"])
        self.pack._plan_geometry.assert_not_called()

    def test_distance_ratio_at_trigger_boundary_is_not_excessive(self):
        selected, decision = self.select(guide=self.route(1750.0, reliability="reviewed"))
        self.assertIs(selected, self.guided)
        self.assertFalse(decision["triggered"])
        self.pack._plan_geometry.assert_not_called()

    def test_short_absolute_excess_keeps_road_preference(self):
        self.start = Point(0, 0.0, 0.0, 5.0)
        self.goal = Point(0, 100.0, 0.0, 7.0)
        middle = Point(0, 50.0, math.sqrt(300.0 ** 2 - 50.0 ** 2), 6.0)
        guide = Route(self.start, self.goal,
                      (Segment((self.start, middle, self.goal), "fixture", "reviewed", "road:fixture"),))
        route = Route(self.start, self.goal,
                      (Segment(guide.points, "fixture", "navmesh", "nav:fixture"),), 123)
        selected, decision = self.select(route=route, guide=guide)
        self.assertIs(selected, route)
        self.assertFalse(decision["triggered"])
        self.pack._plan_geometry.assert_not_called()

    def test_significantly_shorter_complete_candidate_is_selected(self):
        selected, decision = self.select()
        self.assertEqual(selected.segments, self.direct.segments)
        self.assertEqual(selected.expanded, self.guided.expanded + self.direct.expanded)
        self.assertTrue(decision["triggered"])
        self.assertEqual(decision["selected"], "direct")
        self.assertAlmostEqual(decision["guided_length_xy"], self.guided.distance)
        self.assertAlmostEqual(decision["direct_length_xy"], self.direct.distance)

    def test_bounded_domain_does_not_excuse_insignificant_actual_saving(self):
        self.pack._plan_geometry.return_value = self.route(6500.0, expanded=29)
        selected, decision = self.select()
        self.assertEqual(selected.segments, self.guided.segments)
        self.assertEqual(selected.expanded, self.guided.expanded + 29)
        self.assertEqual(decision["selected"], "guided")

    def test_twenty_percent_saving_at_acceptance_boundary_is_selected(self):
        self.pack._plan_geometry.return_value = self.route(6000.0, expanded=29)
        selected, decision = self.select()
        self.assertEqual(selected.segments, self.pack._plan_geometry.return_value.segments)
        self.assertEqual(selected.expanded, self.guided.expanded + 29)
        self.assertEqual(decision["selected"], "direct")

    def test_probe_uses_exact_resolved_floors_blocks_and_remaining_budget(self):
        blocks = frozenset({"nav:fixture:0:1,2,3:4,5,6"})
        cancel = threading.Event()
        self.select(blocked=blocks, cancel=cancel, budget=1000)
        args, kwargs = self.pack._plan_geometry.call_args
        self.assertEqual(args[:3], (self.start, self.goal, None))
        self.assertIs(args[3], blocks)
        self.assertIs(args[4], cancel)
        self.assertEqual(args[5], 1000 - self.guided.expanded)
        self.assertIn("road_trace", args[6])
        self.assertAlmostEqual(kwargs["direct_limit"], .8 * self.guided.distance)

    def test_failed_budget_probe_is_charged_and_keeps_complete_guided_route(self):
        failure = PlanningError("budget", "fixture node pool exhaustion")
        failure.expanded = 31
        self.pack._plan_geometry.side_effect = failure
        selected, decision = self.select()
        self.assertEqual(selected.segments, self.guided.segments)
        self.assertEqual(selected.expanded, self.guided.expanded + failure.expanded)
        self.assertEqual(decision["selected"], "guided")

    def test_expected_disconnection_probe_failures_keep_guided_and_charge_work(self):
        for code in ("no_route", "blocked_navmesh", "road_scope_unconnected", "road_connection_unverified"):
            with self.subTest(code=code):
                failure = PlanningError(code, "fixture incomplete connection")
                failure.expanded = 11
                self.pack._plan_geometry.side_effect = failure
                selected, decision = self.select()
                self.assertEqual(selected.segments, self.guided.segments)
                self.assertEqual(selected.expanded, self.guided.expanded + 11)
                self.assertEqual(decision["selected"], "guided")

    def test_no_remaining_budget_does_not_start_another_query(self):
        selected, decision = self.select(budget=self.guided.expanded)
        self.assertEqual(selected, self.guided)
        self.assertEqual(decision["selected"], "guided")
        self.pack._plan_geometry.assert_not_called()

    def test_probe_cancellation_is_propagated(self):
        failure = PlanningError("cancelled", "fixture cancellation")
        self.pack._plan_geometry.side_effect = failure
        with self.assertRaises(PlanningError) as raised:
            self.select(cancel=threading.Event())
        self.assertIs(raised.exception, failure)

    def test_integrity_failure_is_not_hidden_by_an_existing_guided_route(self):
        failure = PlanningError("navmesh_integrity", "fixture damaged tile")
        self.pack._plan_geometry.side_effect = failure
        with self.assertRaises(PlanningError) as raised:
            self.select()
        self.assertIs(raised.exception, failure)


if __name__ == "__main__":
    unittest.main()
