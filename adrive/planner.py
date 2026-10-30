"""Production road-only planning and the retained legacy diagnostic planner.

The public entrypoint requires a collision-bound unified road pack. The private
_plan_coarse function remains only for historical source/geometry diagnostics;
its road/grid costs and labels never authorize production movement.
"""
from __future__ import annotations

from collections import defaultdict
import heapq
import itertools
import math
import threading

from .data import CELL_SIZE, DIRS, GRID_SIZE, REVIEWED, ROAD_ACCESS_RADIUS, DataBundle, cell, cell_center
from .models import PlanningError, Point, Route, Segment


def _grid_id(row: int, col: int) -> int:
    return -1 - row * GRID_SIZE - col


def _grid_cell(key: int) -> tuple[int, int]:
    return divmod(-1 - key, GRID_SIZE)


def _terrain_id(c: int, a: int, b: int) -> str:
    return f"terrain:{c}:{min(a, b)}:{max(a, b)}"


def _project(p: Point, a: Point, b: Point) -> tuple[float, Point]:
    dx, dy = b.x - a.x, b.y - a.y
    norm = dx * dx + dy * dy
    t = max(0., min(1., ((p.x - a.x) * dx + (p.y - a.y) * dy) / norm)) if norm else 0.
    return t, Point(p.c, a.x + t * dx, a.y + t * dy)


class Planner:
    def __init__(self, bundle: DataBundle):
        self.bundle = bundle

    def plan(self, start: Point, goal: Point, allow_candidates: bool = False,
             blocked_edges: frozenset[str] = frozenset(), cancel: threading.Event | None = None,
             max_expansions: int = 500000) -> Route:
        from .navmesh import load_pack
        from .roads import load_network, RoadPlanner
        if cancel and cancel.is_set():
            raise PlanningError('cancelled','规划已取消')
        if start.c != goal.c:
            raise PlanningError('different_continent','道路导航不能跨大陆')
        # Arrival with no movement creates no walking segment and needs no
        # fallback geometry. Retain this old controller/worker contract.
        if start == goal:
            return Route(start,goal,(),0)
        pack=load_pack(self.bundle.asset_root)
        if pack is None:
            raise PlanningError('navmesh_missing','正式道路导航需要有效碰撞包；未回退到地形估算')
        # allow_candidates is accepted only for old callers/configuration. All
        # published roads have one class; provenance never changes path cost.
        network=load_network(self.bundle.asset_root,pack)
        return RoadPlanner(pack,network).plan(start,goal,blocked_edges,cancel,max_expansions)

    def _plan_coarse(self, start: Point, goal: Point, allow_candidates: bool = False,
                     blocked_edges: frozenset[str] = frozenset(), cancel: threading.Event | None = None,
                     max_expansions: int = 500000) -> Route:
        if cancel and cancel.is_set():
            raise PlanningError("cancelled", "规划已取消")
        if start.c != goal.c:
            raise PlanningError("different_continent", "首版不能跨大陆行走")
        if start.c not in (0, 1):
            raise PlanningError("unsupported_continent", "首版仅支持东部王国和卡利姆多")
        if not all(math.isfinite(v) for p in (start, goal) for v in (p.x, p.y)):
            raise PlanningError("invalid_point", "坐标必须为有限数值")
        # No movement is needed, so unknown surrounding terrain cannot turn an
        # already-reached destination into a planning fault.
        if start.distance(goal) <= 1e-7:
            return Route(start, goal, (), 0)
        if max_expansions < 1:
            raise PlanningError("budget", "寻路展开预算已用尽")
        c, bundle = start.c, self.bundle
        points = list(bundle.nodes)
        start_id, goal_id = len(points), len(points) + 1
        points.extend((start, goal))
        # Each link is target, distance, unreviewed distance, source, class, ID.
        graph: dict[int, list[tuple[int, float, float, str, str, str]]] = defaultdict(list)
        participating: set[int] = set()

        def add(a: int, b: int, distance: float, source: str, reliability: str,
                edge_id: str, reverse: bool = True) -> None:
            if edge_id in blocked_edges:
                return
            unreviewed = 0. if reliability == "reviewed" else distance
            graph[a].append((b, distance, unreviewed, source, reliability, edge_id))
            if reverse:
                graph[b].append((a, distance, unreviewed, source, reliability, edge_id))

        eligible = [e for e in bundle.edges if e.kind == "walk" and e.id not in blocked_edges and
                    (e.status in REVIEWED or allow_candidates) and bundle.nodes[e.a].c == c]
        # Start/goal projections split the source edge. Both endpoints may lie on
        # one edge; sorted splits avoid a detour through an unrelated endpoint.
        for index, edge in enumerate(eligible):
            if index % 256 == 0 and cancel and cancel.is_set():
                raise PlanningError("cancelled", "规划已取消")
            a, b = points[edge.a], points[edge.b]
            reliability = "reviewed" if edge.status in REVIEWED else "candidate"
            cuts = [(0., edge.a), (1., edge.b)]
            for endpoint_id, endpoint in ((start_id, start), (goal_id, goal)):
                t, projected = _project(endpoint, a, b)
                distance = endpoint.distance(projected)
                # Bound access to the actual road; never relocate the endpoint.
                if distance > ROAD_ACCESS_RADIUS:
                    continue
                clear_forward = bundle.line_clear(endpoint, projected) if distance > 1e-7 else True
                clear_reverse = bundle.line_clear(projected, endpoint) if distance > 1e-7 else True
                forward_source = reverse_source = "terrain endpoint access"
                # Reviewed roads can cross water on bridges that the coarse
                # terrain raster cannot represent. Permit a short estimated
                # approach, still respecting every directional/corner check.
                # Candidate roads cannot override the dry-terrain requirement.
                if reliability == "reviewed":
                    if not clear_forward and bundle.line_clear(endpoint, projected, road_access=True):
                        clear_forward = True
                        forward_source = "reviewed road endpoint access estimate"
                    if not clear_reverse and bundle.line_clear(projected, endpoint, road_access=True):
                        clear_reverse = True
                        reverse_source = "reviewed road endpoint access estimate"
                if not (clear_forward or clear_reverse):
                    continue
                if t <= 1e-10:
                    node = edge.a
                elif t >= 1 - 1e-10:
                    node = edge.b
                else:
                    node = len(points)
                    points.append(projected)
                    cuts.append((t, node))
                connector_id = f"access:{edge.id}:{endpoint.x:.4f}:{endpoint.y:.4f}"
                if distance <= 1e-7:
                    add(endpoint_id, node, 0., edge.source, reliability, connector_id)
                else:
                    if clear_forward:
                        add(endpoint_id, node, distance, forward_source, "terrain", connector_id, reverse=False)
                    if clear_reverse:
                        add(node, endpoint_id, distance, reverse_source, "terrain", connector_id, reverse=False)
            cuts.sort()
            for (_, u), (_, v) in zip(cuts, cuts[1:]):
                add(u, v, points[u].distance(points[v]), edge.source, reliability, edge.id)
                participating.update((u, v))

        # Only points in the same known dry cell are joined to its centre.
        # No nearest-free-cell relocation, island escape, or missing-map jump.
        for node in participating | {start_id, goal_id}:
            point = points[node]
            row, col = cell(point)
            if not bundle.dry(c, row, col):
                continue
            grid_id = _grid_id(row, col)
            centre = cell_center(c, row, col)
            join_id = f"join:{c}:{node}" if node < len(bundle.nodes) else f"access:{c}:{point.x:.4f}:{point.y:.4f}"
            distance = point.distance(centre)
            if bundle.line_clear(point, centre):
                add(node, grid_id, distance, "terrain access", "terrain", join_id, reverse=False)
            if bundle.line_clear(centre, point):
                add(grid_id, node, distance, "terrain access", "terrain", join_id, reverse=False)

        if not graph.get(start_id):
            raise PlanningError("start_unreachable", "起点没有有效地面连接；未将起点移动到别处")
        if not graph.get(goal_id) and not any(link[0] == goal_id for links in graph.values() for link in links):
            raise PlanningError("goal_unreachable", "终点没有有效地面连接；未替换所选终点")
        def point_for(key: int) -> Point:
            return points[key] if key >= 0 else cell_center(c, *_grid_cell(key))

        def links(key: int):
            yield from graph.get(key, ())
            if key >= 0:
                return
            row, col = _grid_cell(key)
            conn, water = bundle.info(c, row, col)
            if water or not conn:
                return
            for direction, (dr, dc) in enumerate(DIRS):
                if not conn & (1 << direction):
                    continue
                nr, nc = row + dr, col + dc
                if not bundle.dry(c, nr, nc):
                    continue
                # Do not cut a blocked corner even if the diagonal bit exists.
                if dr and dc and not (bundle.can_step(c, row, col, nr, col) and
                                      bundle.can_step(c, row, col, row, nc) and
                                      bundle.can_step(c, nr, col, nr, nc) and
                                      bundle.can_step(c, row, nc, nr, nc)):
                    continue
                neighbor = _grid_id(nr, nc)
                edge_id = _terrain_id(c, key, neighbor)
                if edge_id in blocked_edges:
                    continue
                distance = CELL_SIZE * (math.sqrt(2) if dr and dc else 1.)
                yield neighbor, distance, distance, "terrain grid", "terrain", edge_id

        # h=(0, euclidean distance) is admissible with zero-risk roads. If there
        # are no reviewed roads, all travel adds risk == distance, so Euclidean
        # distance is also an admissible first-component heuristic.
        all_uncertain = not any(e.status in REVIEWED for e in eligible)
        serial = itertools.count()
        queue = [(start.distance(goal) if all_uncertain else 0., start.distance(goal), next(serial), start_id, 0., 0.)]
        scores = {start_id: (0., 0.)}
        previous: dict[int, tuple[int, tuple]] = {}
        expanded = 0
        while queue:
            _, _, _, key, risk, length = heapq.heappop(queue)
            if scores.get(key) != (risk, length):
                continue
            if key == goal_id:
                break
            if expanded >= max_expansions:
                raise PlanningError("budget", f"寻路超过 {max_expansions} 次展开；没有返回未完成路线")
            expanded += 1
            if expanded % 128 == 0 and cancel and cancel.is_set():
                raise PlanningError("cancelled", "规划已取消")
            for link in links(key):
                neighbor, distance, uncertain, *_ = link
                candidate = risk + uncertain, length + distance
                if candidate < scores.get(neighbor, (math.inf, math.inf)):
                    scores[neighbor] = candidate
                    previous[neighbor] = key, link
                    heuristic = point_for(neighbor).distance(goal)
                    heapq.heappush(queue, (candidate[0] + (heuristic if all_uncertain else 0.), candidate[1] + heuristic, next(serial), neighbor, *candidate))
        else:
            raise PlanningError("no_route", "地面数据中没有连通路线；不会补画直线")

        path = []
        key = goal_id
        while key != start_id:
            before, link = previous[key]
            _, distance, _, source, reliability, edge_id = link
            a, b = point_for(before), point_for(key)
            if distance > 1e-7:
                path.append(Segment((a, b), source, reliability, edge_id))
            key = before
        path.reverse()
        # Keep per-edge IDs for failed-edge exclusion and every original road
        # vertex. Terrain simplification is intentionally omitted in v1: this
        # conservative representation cannot accidentally skip a directed edge.
        return Route(start, goal, tuple(path), expanded)
