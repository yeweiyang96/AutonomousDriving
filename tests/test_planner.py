from __future__ import annotations

from pathlib import Path
import threading
import unittest

from adrive.data import DataBundle, RoadEdge, CELL_SIZE, cell, cell_center
from adrive.models import Point, PlanningError
from adrive.planner import Planner


def encode_rle(data: bytes) -> bytes:
    output = bytearray()
    i = 0
    while i < len(data):
        j = i + 1
        while j < len(data) and data[j] == data[i] and j - i < 255:
            j += 1
        output.extend((j - i, data[i]))
        i = j
    return bytes(output)


def terrain_bundle(cells: dict[tuple[int, int], tuple[int, bool]], nodes=None, edges=None):
    tiles = {}
    for (r, col), (conn, water) in cells.items():
        key = r // 32 * 64 + col // 32
        tile = tiles.setdefault(key, bytearray(1152))
        index = (r % 32) * 32 + col % 32
        tile[index] = conn
        if water:
            tile[1024 + index // 8] |= 1 << (index % 8)
    return DataBundle(nodes or [], edges or [], {0: {k: encode_rle(v) for k, v in tiles.items()}})


def p(r: int, col: int) -> Point:
    return cell_center(0, r, col)


def road(i, a, b, status="user_reviewed", kind="walk"):
    return RoadEdge(f"road:{i}", a, b, status, kind, f"source-{i}")


class PlannerTests(unittest.TestCase):
    def test_long_wilderness_without_road(self):
        cells = {(100, col): (68, False) for col in range(100, 230)}
        route = Planner(terrain_bundle(cells)).plan(p(100, 100), p(100, 229))
        self.assertGreater(route.distance, 2000)
        self.assertEqual(route.start, p(100, 100))
        self.assertEqual(route.goal, p(100, 229))
        self.assertTrue(all(s.reliability == "terrain" for s in route.segments))

    def test_directed_terrain_does_not_become_bidirectional(self):
        cells = {(100, col): (4, False) for col in range(100, 105)}
        planner = Planner(terrain_bundle(cells))
        planner.plan(p(100, 100), p(100, 104))
        with self.assertRaises(PlanningError) as raised:
            planner.plan(p(100, 104), p(100, 100))
        self.assertEqual(raised.exception.code, "no_route")

    def test_water_requires_dry_detour(self):
        cells = {(r, c): (255, False) for r in range(99, 102) for c in range(100, 105)}
        cells[(100, 102)] = (255, True)
        route = Planner(terrain_bundle(cells)).plan(p(100, 100), p(100, 104))
        self.assertGreater(route.distance, CELL_SIZE * 4)
        self.assertNotIn((100, 102), [cell(point) for point in route.points])

    def test_water_barrier_has_no_fabricated_bridge(self):
        cells = {(100, col): (68, col == 102) for col in range(100, 105)}
        with self.assertRaises(PlanningError) as raised:
            Planner(terrain_bundle(cells)).plan(p(100, 100), p(100, 104))
        self.assertEqual(raised.exception.code, "no_route")

    def test_missing_terrain_is_not_relocated(self):
        cells = {(100, col): (68, False) for col in range(100, 105)}
        for start, goal, code in [(p(99, 100), p(100, 100), "start_unreachable"), (p(100, 100), p(100, 106), "goal_unreachable")]:
            with self.subTest(code=code), self.assertRaises(PlanningError) as raised:
                Planner(terrain_bundle(cells)).plan(start, goal)
            self.assertEqual(raised.exception.code, code)

    def test_diagonal_does_not_cut_blocked_corner(self):
        bundle = terrain_bundle({(100, 100): (8, False), (101, 101): (128, False)})
        self.assertFalse(bundle.line_clear(p(100, 100), p(101, 101)))
        with self.assertRaises(PlanningError):
            Planner(bundle).plan(p(100, 100), p(101, 101))

    def test_reviewed_roads_and_original_corner_preserved(self):
        nodes = [p(100, 100), p(110, 100), p(110, 110)]
        planner = Planner(DataBundle(nodes, [road(1, 0, 1), road(2, 1, 2)], {}))
        for start, end in ((nodes[0], nodes[2]), (nodes[2], nodes[0])):
            route = planner.plan(start, end)
            self.assertIn(nodes[1], route.points)
            self.assertEqual({s.reliability for s in route.segments}, {"reviewed"})

    def test_candidate_toggle_and_non_ground_exclusion(self):
        nodes = [p(100, 100), p(100, 105)]
        planner = Planner(DataBundle(nodes, [road(1, 0, 1, "road_proposal")], {}))
        with self.assertRaises(PlanningError):
            planner.plan(*nodes)
        route = planner.plan(*nodes, allow_candidates=True)
        self.assertEqual({s.reliability for s in route.segments}, {"candidate"})
        for kind in ("tunnel", "lift", "city_access"):
            with self.subTest(kind=kind), self.assertRaises(PlanningError):
                Planner(DataBundle(nodes, [road(1, 0, 1, kind=kind)], {})).plan(*nodes, allow_candidates=True)

    def test_reviewed_detour_preferred_to_short_candidate(self):
        nodes = [p(100, 100), p(110, 100), p(100, 101)]
        edges = [road(1, 0, 1), road(2, 1, 2), road(3, 0, 2, "editor_proposal")]
        route = Planner(DataBundle(nodes, edges, {})).plan(nodes[0], nodes[2], allow_candidates=True)
        self.assertIn(nodes[1], route.points)
        self.assertNotIn("road:3", [s.edge_id for s in route.segments])

    def test_road_and_terrain_can_join_between_disconnected_roads(self):
        cells = {(100, col): (68, False) for col in range(100, 108)}
        nodes = [p(100, 100), p(100, 102), p(100, 105), p(100, 107)]
        route = Planner(terrain_bundle(cells, nodes, [road(1, 0, 1), road(2, 2, 3)])).plan(nodes[0], nodes[3])
        ids = [s.edge_id for s in route.segments]
        self.assertIn("road:1", ids)
        self.assertIn("road:2", ids)
        self.assertIn("terrain", {s.reliability for s in route.segments})

    def test_blocked_road_falls_back_to_valid_terrain(self):
        cells = {(100, col): (68, False) for col in range(100, 105)}
        nodes = [p(100, 100), p(100, 104)]
        planner = Planner(terrain_bundle(cells, nodes, [road(1, 0, 1)]))
        route = planner.plan(*nodes, blocked_edges=frozenset(("road:1",)))
        self.assertEqual({s.reliability for s in route.segments}, {"terrain"})
        blocked = frozenset((route.segments[len(route.segments) // 2].edge_id, "road:1"))
        with self.assertRaises(PlanningError):
            planner.plan(*nodes, blocked_edges=blocked)

    def test_exact_endpoint_is_preserved_on_one_road_edge(self):
        nodes = [p(100, 100), p(100, 110)]
        start, goal = p(100, 102), p(100, 108)
        route = Planner(DataBundle(nodes, [road(1, 0, 1)], {})).plan(start, goal)
        self.assertEqual(route.points[0], start)
        self.assertEqual(route.points[-1], goal)
        self.assertAlmostEqual(route.distance, start.distance(goal))

    def test_offroad_endpoint_uses_estimated_connection_not_silent_snap(self):
        cells = {(r, col): (255, False) for r in range(99, 102) for col in range(100, 106)}
        nodes = [p(100, 100), p(100, 105)]
        start = Point(0, p(100, 102).x + 3, p(100, 102).y)
        goal = Point(0, p(100, 104).x + 2, p(100, 104).y)
        route = Planner(terrain_bundle(cells, nodes, [road(1, 0, 1)])).plan(start, goal)
        self.assertEqual(route.points[0], start)
        self.assertEqual(route.points[-1], goal)
        self.assertEqual(route.segments[0].reliability, "terrain")
        self.assertEqual(route.segments[-1].reliability, "terrain")

    def test_near_reviewed_road_over_water_preserves_both_endpoints(self):
        nodes = [p(100, 100), p(100, 105)]
        cells = {(100, col): (68, True) for col in range(100, 106)}
        start = Point(0, p(100, 102).x + 3, p(100, 102).y)
        goal = Point(0, p(100, 104).x - 2, p(100, 104).y)
        for status in ("user_reviewed", "user_correction"):
            bundle = terrain_bundle(cells, nodes, [road(1, 0, 1, status)])
            for a, b in ((start, goal), (goal, start)):
                with self.subTest(status=status, start=a):
                    route = Planner(bundle).plan(a, b)
                    self.assertEqual(route.points[0], a)
                    self.assertEqual(route.points[-1], b)
                    self.assertEqual(route.segments[0].reliability, "terrain")
                    self.assertEqual(route.segments[-1].reliability, "terrain")
                    self.assertEqual(route.segments[1].edge_id, "road:1")
                    self.assertAlmostEqual(route.distance, p(100, 102).distance(p(100, 104)) + 5)
                    self.assertIn("road endpoint access", route.segments[0].source)
                    self.assertFalse(bundle.line_clear(*route.segments[0].points))

    def test_road_water_access_stays_local_and_requires_reviewed_walk_edge(self):
        nodes = [p(100, 100), p(100, 105)]
        cells = {(100, col): (68, True) for col in range(100, 106)}
        start = Point(0, p(100, 102).x + 3, p(100, 102).y)
        cases = [
            ([], start, frozenset()),
            ([road(1, 0, 1, "editor_proposal")], start, frozenset()),
            ([road(1, 0, 1, kind="tunnel")], start, frozenset()),
            ([road(1, 0, 1)], Point(0, start.x + 3, start.y), frozenset()),
            ([road(1, 0, 1)], start, frozenset(("road:1",))),
        ]
        for edges, a, blocked in cases:
            with self.subTest(edges=edges, start=a, blocked=blocked), self.assertRaises(PlanningError) as raised:
                Planner(terrain_bundle(cells, nodes, edges)).plan(a, nodes[1], allow_candidates=True, blocked_edges=blocked)
            self.assertEqual(raised.exception.code, "start_unreachable")

    def test_failed_water_road_access_is_not_retried(self):
        nodes = [p(100, 100), p(100, 105)]
        cells = {(100, col): (68, True) for col in range(100, 106)}
        start = Point(0, p(100, 102).x + 3, p(100, 102).y)
        planner = Planner(terrain_bundle(cells, nodes, [road(1, 0, 1)]))
        route = planner.plan(start, nodes[1])
        with self.assertRaises(PlanningError) as raised:
            planner.plan(start, nodes[1], blocked_edges=frozenset((route.segments[0].edge_id,)))
        self.assertEqual(raised.exception.code, "start_unreachable")

    def test_road_water_access_respects_directed_cell_boundary(self):
        # The endpoint and road are 2 yards apart across a cell boundary.
        boundary = p(100, 100).x - CELL_SIZE / 2
        nodes = [Point(0, boundary - 1, p(101, col).y) for col in (100, 105)]
        start = Point(0, boundary + 1, p(100, 102).y)
        cells = {(101, col): (68, True) for col in range(100, 106)}
        cells[(100, 102)] = (16, True)  # Only south, toward the road.
        planner = Planner(terrain_bundle(cells, nodes, [road(1, 0, 1)]))
        self.assertEqual(planner.plan(start, nodes[1]).points[0], start)
        with self.assertRaises(PlanningError):
            planner.plan(nodes[1], start)
        # A known cell with the wrong direction, missing data, and a blocked
        # cell must all remain inaccessible even within the road tolerance.
        for info in ((1, True), (0, False), None):
            restricted = cells.copy()
            if info is None:
                del restricted[(100, 102)]
            else:
                restricted[(100, 102)] = info
            with self.subTest(info=info), self.assertRaises(PlanningError) as raised:
                Planner(terrain_bundle(restricted, nodes, [road(1, 0, 1)])).plan(start, nodes[1])
            self.assertEqual(raised.exception.code, "start_unreachable")

    def test_road_water_access_does_not_cut_blocked_corner(self):
        boundary_x = p(100, 100).x - CELL_SIZE / 2
        boundary_y = p(100, 100).y - CELL_SIZE / 2
        corner = Point(0, boundary_x - 1, boundary_y - 1)
        start = Point(0, boundary_x + 1, boundary_y + 1)
        nodes = [corner, Point(0, corner.x - 20, corner.y - 20)]
        cells = {(100, 100): (8, True), (101, 101): (128, True)}
        with self.assertRaises(PlanningError) as raised:
            Planner(terrain_bundle(cells, nodes, [road(1, 0, 1)])).plan(start, nodes[1])
        self.assertEqual(raised.exception.code, "start_unreachable")

    def test_different_continent_cancel_budget_and_invalid(self):
        planner = Planner(terrain_bundle({(100, c): (68, False) for c in range(100, 120)}))
        with self.assertRaises(PlanningError) as raised:
            planner.plan(p(100, 100), Point(1, 0, 0))
        self.assertEqual(raised.exception.code, "different_continent")
        cancel = threading.Event()
        cancel.set()
        with self.assertRaises(PlanningError) as raised:
            planner.plan(p(100, 100), p(100, 119), cancel=cancel)
        self.assertEqual(raised.exception.code, "cancelled")
        with self.assertRaises(PlanningError) as raised:
            planner.plan(p(100, 100), p(100, 119), max_expansions=2)
        self.assertEqual(raised.exception.code, "budget")
        with self.assertRaises(PlanningError) as raised:
            planner.plan(Point(0, float("nan"), 0), p(100, 119))
        self.assertEqual(raised.exception.code, "invalid_point")


@unittest.skipUnless((Path(__file__).resolve().parents[1] / "data/bundle.json").exists(), "Extract data first")
class RealDataRouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle = DataBundle.load(Path(__file__).resolve().parents[1] / "data")
        # These regressions describe the retained legacy road/grid backend.
        # Native collision geometry has separate integration/fixture tests.
        cls.bundle.asset_root = None

    def test_real_reviewed_edges_in_both_continents_and_directions(self):
        bundle = self.bundle
        planner = Planner(bundle)
        for c in (0, 1):
            edge = next(e for e in bundle.edges if e.kind == "walk" and e.status == "user_reviewed" and bundle.nodes[e.a].c == c and bundle.nodes[e.a].distance(bundle.nodes[e.b]) > 5)
            for a, b in ((edge.a, edge.b), (edge.b, edge.a)):
                with self.subTest(c=c, direction=(a, b)):
                    route = planner.plan(bundle.nodes[a], bundle.nodes[b], max_expansions=100000)
                    self.assertEqual(route.goal, bundle.nodes[b])
                    self.assertTrue(route.segments)
                    self.assertTrue(all(s.reliability == "reviewed" for s in route.segments))

    def test_cross_zone_routes_under_default_expansion_budget(self):
        bundle = self.bundle
        planner = Planner(bundle)
        # Fixed, source-backed outdoor endpoints; source digest tests guard the
        # index identities. These checks are graph evidence, not playtesting.
        for a, b in ((3766, 3908), (3738, 3586), (1108, 512)):
            with self.subTest(endpoints=(a, b)):
                route = planner.plan(bundle.nodes[a], bundle.nodes[b])
                self.assertLess(route.expanded, 500000)
                self.assertEqual(route.points[0], bundle.nodes[a])
                self.assertEqual(route.points[-1], bundle.nodes[b])
                self.assertNotIn("candidate", {s.reliability for s in route.segments})
                for left, right in zip(route.segments, route.segments[1:]):
                    self.assertLess(left.points[-1].distance(right.points[0]), 1e-6)

    def test_real_wilderness_route_without_road_graph(self):
        bundle = self.bundle
        planner = Planner(DataBundle([], [], bundle.terrain))
        route = planner.plan(bundle.nodes[1108], bundle.nodes[512])
        self.assertGreater(route.distance, 2000.)
        self.assertLess(route.expanded, 500000)
        self.assertTrue(all(s.reliability == "terrain" for s in route.segments))
        for segment in route.segments:
            a, b = segment.points
            self.assertTrue(bundle.line_clear(a, b))

    def test_stormwind_roadside_water_raster_regression(self):
        # Actual logged position from the roadside failure; goal transcribed
        # from the screenshot. This verifies planning, not in-game collision.
        start, goal = Point(0, -8967.125, 511.1875), Point(0, -10543.9, -1192.3)
        self.assertTrue(self.bundle.info(start.c, *cell(start))[1])
        for a, b in ((start, goal), (goal, start)):
            with self.subTest(start=a):
                route = Planner(self.bundle).plan(a, b)
                self.assertEqual(route.points[0], a)
                self.assertEqual(route.points[-1], b)
                access = route.segments[0] if a == start else route.segments[-1]
                self.assertAlmostEqual(access.distance, 2.180402004355254)
                self.assertEqual(access.reliability, "terrain")
                self.assertTrue(access.edge_id.startswith("access:road:4254:"))
                self.assertIn("road:4254", [s.edge_id for s in route.segments])
                self.assertNotIn("candidate", {s.reliability for s in route.segments})
                for left, right in zip(route.segments, route.segments[1:]):
                    self.assertLess(left.points[-1].distance(right.points[0]), 1e-6)


if __name__ == "__main__":
    unittest.main()
