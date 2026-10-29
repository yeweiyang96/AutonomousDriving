"""Forward road following with conservative native length-domain queries.

Guide vertices are soft preferences. Every emitted segment comes from a
complete native query; XY corridor checks never create a walking segment.
This module does not infer an intended floor from a two-dimensional road.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from .models import PlanningError, Point, Route


@dataclass(frozen=True)
class FollowPolicy:
    anchor_spacing: float = 8.0
    lookahead: float = 60.0
    corridor: float = 3.0
    relaxed_corridor: float = 9.0
    detour_corridor: float = 24.0
    detour_lookahead: float = 120.0
    metric_spacing: float = 1.0
    backtrack_tolerance: float = 2.0
    local_budget: int = 4096
    rollback_chunks: int = 2
    rollback_span: float = 120.0


def _xy(point):
    return (point.x, point.y)


def _length(points):
    return sum(math.dist(_xy(a), _xy(b)) for a, b in zip(points, points[1:]))


def detour_metrics(points, reference):
    """Shared length acceptance for a native path and its remaining road guide.

    This is a preference-quality bound, not evidence that geometry is absent.
    It deliberately does not turn the local XY corridor into a hard obstacle.
    """
    route_length, reference_length = _length(points), _length(reference)
    limit = 2.5 * reference_length + 20.0
    return {'route_length_xy': route_length, 'reference_length_xy': reference_length,
            'length_limit_xy': limit, 'excessive_detour': route_length > limit}


def _sample(points, spacing):
    result = [_xy(points[0])]
    for a, b in zip(points, points[1:]):
        length = a.distance(b)
        count = max(1, math.ceil(length / spacing))
        if len(result) + count > 4096:
            raise ValueError('curve_metric_sample_limit')
        for k in range(1, count + 1):
            fraction = k / count
            result.append((a.x + fraction * (b.x-a.x), a.y + fraction * (b.y-a.y)))
    return result


def discrete_frechet(a, b):
    """Ordered bidirectional curve distance for already sampled XY curves."""
    previous = [math.inf] * len(b)
    for i, p in enumerate(a):
        row = [math.inf] * len(b)
        for j, q in enumerate(b):
            before = 0.0 if i == j == 0 else min(previous[j], row[j-1] if j else math.inf,
                                               previous[j-1] if j else math.inf)
            row[j] = max(before, math.dist(p, q))
        previous = row
    return previous[-1]


def progress_backtrack(samples, reference):
    """Nearest XY progress is supplementary; it is not used to trim geometry."""
    spans, distance = [], 0.0
    for a, b in zip(reference, reference[1:]):
        length = a.distance(b)
        if length:
            spans.append((a, b, length, distance))
            distance += length
    previous, backward = None, 0.0
    for x, y in samples:
        closest = None
        for a, b, length, origin in spans:
            dx, dy = b.x-a.x, b.y-a.y
            t = max(0.0, min(1.0, ((x-a.x)*dx + (y-a.y)*dy) / (length*length)))
            d2 = (x-a.x-t*dx)**2 + (y-a.y-t*dy)**2
            progress = origin + t*length
            candidate = (d2, abs(progress-previous) if previous is not None else progress, progress)
            if closest is None or candidate < closest:
                closest = candidate
        if closest is not None:
            current = closest[2]
            if previous is not None:
                backward += max(0.0, previous-current)
            previous = current
    return backward


def corridor_metrics(points, reference, policy):
    route_samples = _sample(points, policy.metric_spacing)
    road_samples = _sample(reference, policy.metric_spacing)
    return {'ordered_distance_xy': discrete_frechet(route_samples, road_samples),
            'backtrack_xy': progress_backtrack(route_samples, reference),
            'route_length_xy': _length(points), 'reference_length_xy': _length(reference)}


def _anchors(guide, start, goal, policy):
    points, road, source = [start], [], []
    for segment_index, segment in enumerate(guide.segments):
        is_road = segment.reliability in ('reviewed', 'candidate')
        if not is_road:
            continue
        for vertex_index, vertex in enumerate(segment.points):
            before = points[-1]
            distance = before.distance(vertex)
            if distance <= 1e-7:
                continue
            along_road = vertex_index > 0
            steps = max(1, math.ceil(distance / policy.anchor_spacing)) if along_road else 1
            for step in range(1, steps + 1):
                t = step / steps
                z = (before.z + t*(vertex.z-before.z)) if before.z is not None and vertex.z is not None else None
                points.append(Point(vertex.c, before.x + t*(vertex.x-before.x), before.y + t*(vertex.y-before.y), z))
                road.append(along_road)
                source.append(segment_index)
    if points[-1].distance(goal) > 1e-7:
        points.append(goal)
        road.append(False)
        source.append(None)
    else:
        points[-1] = goal
    arclength = [0.0]
    for a, b in zip(points, points[1:]):
        arclength.append(arclength[-1] + a.distance(b))
    return points, road, source, arclength


def _choices(index, arc, span):
    last = index + 1
    while last + 1 < len(arc) and arc[last + 1] - arc[index] <= span:
        last += 1
    targets = [last]
    for fraction in (.5, .25):
        limit = arc[index] + (arc[last]-arc[index])*fraction
        target = max(index+1, next((i-1 for i in range(index+1, last+1) if arc[i] > limit), last))
        targets.append(target)
    targets.append(index+1)
    return sorted(set(targets), reverse=True)


def follow_guide(mesh, start, goal, guide, max_expansions=500000, *, trace=None, policy=None):
    policy = policy or FollowPolicy()
    trace = [] if trace is None else trace
    anchors, road_flags, sources, arc = _anchors(guide, start, goal, policy)
    current, index, expanded, queries, output = start, 0, 0, 0, []
    height_cache = {}
    query_limit = max(128, math.ceil(arc[-1] / 4) + 32)
    history, forbidden, rollback_count = [], set(), 0
    rollback_frontier = None

    def targets(point):
        if point.z is not None:
            return [point]
        key = (point.c, point.x, point.y)
        if key not in height_cache:
            height_cache[key] = mesh.heights(point)
        return [Point(point.c, point.x, point.y, z)
                for z in sorted(height_cache[key], key=lambda z: abs(z-current.z))]

    while index < len(anchors)-1:
        if mesh.cancelled():
            raise PlanningError('cancelled', '规划已取消')
        connected = None
        attempt_cache = {}
        tiers = [('road', policy.corridor, policy.lookahead),
                 ('road_tolerance', policy.relaxed_corridor, policy.lookahead),
                 ('local_detour', policy.detour_corridor, policy.detour_lookahead)]
        for mode, width, lookahead in tiers:
            choices = _choices(index, arc, lookahead)
            # When already off the preferred corridor, reconnect to the first
            # reachable forward point instead of extending a long excursion.
            if mode == 'local_detour':
                choices.reverse()
            for target_index in choices:
                if target_index in forbidden:
                    continue
                waypoint = anchors[target_index]
                reference = [current, *anchors[index+1:target_index+1]]
                on_road = any(road_flags[index:target_index])
                for target in targets(waypoint):
                    key = (target_index, target.z)
                    if key not in attempt_cache:
                        if expanded >= max_expansions or queries >= query_limit:
                            raise PlanningError('budget', '道路引导搜索预算已用尽；没有返回部分路线')
                        budget = min(policy.local_budget, max_expansions-expanded) if on_road else max_expansions-expanded
                        mesh.debug_context = {'stage': 'road_forward', 'mode': mode,
                                              'anchor_from': index, 'anchor_to': target_index,
                                              'expanded_before': expanded, 'plan_budget': max_expansions}
                        queries += 1
                        try:
                            # The existing final length check applies to every
                            # guide connection, including endpoint access.
                            length_limit = detour_metrics(reference, reference)['length_limit_xy']
                            parts, count = mesh.path(current, target, budget,
                                                     max_length_xy=length_limit)
                            expanded += count
                            points = list(Route(current, target, tuple(parts), count).points)
                            metadata = {'status': 'native_complete', 'native_expanded': count,
                                        'search_length_limit_xy': length_limit}
                            lengths = detour_metrics(points, reference)
                            metadata.update(lengths)
                            if lengths['excessive_detour']:
                                metadata.update(status='excessive_detour')
                            elif on_road:
                                metadata.update(corridor_metrics(points, reference, policy))
                            else:
                                metadata.update(ordered_distance_xy=0.0, backtrack_xy=0.0)
                            attempt_cache[key] = (parts, target, metadata)
                        except PlanningError as exc:
                            expanded += getattr(exc, 'expanded', 0)
                            if exc.code not in ('no_route', 'road_scope_unconnected', 'navmesh_outside', 'navmesh_height', 'budget', 'blocked_navmesh'):
                                raise
                            attempt_cache[key] = (None, target, {'status': exc.code,
                                'native_expanded': getattr(exc, 'expanded', 0),
                                'search_length_limit_xy': length_limit,
                                'native_debug': getattr(exc, 'native_debug', None),
                                'global_unreachable_proven': False})
                        except ValueError as exc:
                            attempt_cache[key] = (None, target, {'status': 'metric_limit', 'detail': str(exc)})
                    parts, resolved_target, info = attempt_cache[key]
                    accepted = (parts is not None and info['status'] == 'native_complete'
                                and info['ordered_distance_xy'] <= width
                                and info['backtrack_xy'] <= policy.backtrack_tolerance)
                    trace.append({'from_anchor': index, 'to_anchor': target_index, 'source_segment': sources[target_index-1],
                                  'mode': mode, 'corridor_width': width, 'target': [target.x, target.y, target.z],
                                  'accepted': accepted, **info})
                    if accepted:
                        connected = (target_index, parts, resolved_target, width)
                        break
                if connected:
                    break
            if connected:
                break
        if connected is None:
            if history and rollback_count < policy.rollback_chunks:
                old_index = index
                before_index, before_point, before_size, _ = history[-1]
                frontier = old_index if rollback_frontier is None else rollback_frontier
                if arc[frontier] - arc[before_index] <= policy.rollback_span:
                    history.pop()
                    forbidden.add(old_index)
                    index, current = before_index, before_point
                    del output[before_size:]
                    rollback_count += 1
                    rollback_frontier = frontier
                    trace.append({'mode': 'rollback_soft_anchor', 'abandoned_anchor': old_index,
                                  'resume_anchor': index, 'native_searches_remain_charged': True})
                    continue
            error = PlanningError('road_guidance_unreachable', '道路附近无法生成完整且向前的步行连接；没有返回部分路线')
            error.expanded = expanded
            error.prefix_segments = tuple(output)
            error.resume_point = current
            error.remaining_reference = tuple(anchors[index+1:])
            error.remaining_reference_range = {
                'anchor_from': index, 'anchor_to': len(anchors)-1,
                'source_segment_indices': sorted({s for s in sources[index:] if s is not None}),
            }
            raise error
        next_index, parts, target, accepted_width = connected
        retained_checkpoints = []
        # A successful query can still revisit the preceding soft anchor.
        # Compare complete native replacements, never splice an unchecked line.
        eligible_history = [i for i,h in enumerate(history) if arc[next_index]-arc[h[0]] <= policy.rollback_span]
        reconnect_history = sorted({eligible_history[0], eligible_history[-1]}) if eligible_history else []
        for history_index in reconnect_history:
            prior_index, prior_point, prior_size, prior_width = history[history_index]
            if arc[next_index]-arc[prior_index] > policy.rollback_span:
                continue
            old_parts = [*output[prior_size:], *parts]
            old_points = list(Route(prior_point, target, tuple(old_parts)).points)
            old_xy = _length(old_points)
            if old_xy-prior_point.distance(target) <= .5:
                continue
            if expanded >= max_expansions or queries >= query_limit:
                raise PlanningError('budget', '道路引导搜索预算已用尽；没有返回部分路线')
            mesh.debug_context = {'stage': 'road_reconnect', 'anchor_from': prior_index,
                                  'anchor_to': next_index, 'expanded_before': expanded,
                                  'plan_budget': max_expansions}
            queries += 1
            try:
                # An accepted reconnect must be shorter than old_xy - .5.
                # Use that existing threshold as a conservative search domain.
                reconnect_limit = old_xy - .5
                replacement, count = mesh.path(prior_point, target,
                                               min(policy.local_budget, max_expansions-expanded),
                                               max_length_xy=reconnect_limit)
                expanded += count
                new_points = list(Route(prior_point, target, tuple(replacement)).points)
                reference = [prior_point, *anchors[prior_index+1:next_index+1]]
                metrics = corridor_metrics(new_points, reference, policy)
                old_3d = sum(math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)) for a,b in zip(old_points, old_points[1:]))
                new_3d = sum(math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)) for a,b in zip(new_points, new_points[1:]))
                width = min(policy.relaxed_corridor, max(accepted_width, prior_width,
                                                        *(h[3] for h in history[history_index:])))
                better = (metrics['route_length_xy'] < old_xy-.5 and new_3d < old_3d-.5
                          and metrics['ordered_distance_xy'] <= width
                          and metrics['backtrack_xy'] <= policy.backtrack_tolerance)
                trace.append({'mode': 'native_reconnect', 'from_anchor': prior_index, 'to_anchor': next_index,
                              'accepted': better, 'corridor_width': width, 'replaced_length_xy': old_xy,
                              'replaced_chunk_count': len(history)-history_index+1,
                              'replaced_length_3d': old_3d, 'replacement_length_3d': new_3d,
                              'native_expanded': count, 'search_length_limit_xy': reconnect_limit, **metrics})
                if better:
                    # A merged chunk must not erase every bridge-approach
                    # checkpoint. Retain only real native polygon-boundary
                    # points on the replacement, in forward order. No old
                    # off-path coordinate or interpolated point is reused.
                    last_position = 0
                    for old_index, _, _, _ in history[history_index+1:]:
                        candidates = [(replacement[k].points[0].distance(anchors[old_index]), k)
                                      for k in range(last_position+1, len(replacement))]
                        if not candidates:
                            break
                        distance, position = min(candidates)
                        if distance <= width:
                            retained_checkpoints.append((old_index, replacement[position].points[0],
                                                         prior_size+position, width))
                            last_position = position
                    trace[-1]['retained_native_checkpoints'] = len(retained_checkpoints)
                    history = history[:history_index]
                    del output[prior_size:]
                    index, current = prior_index, prior_point
                    parts, accepted_width = replacement, width
                    break
            except PlanningError as exc:
                expanded += getattr(exc, 'expanded', 0)
                if exc.code not in ('no_route', 'road_scope_unconnected', 'navmesh_outside', 'navmesh_height', 'budget', 'blocked_navmesh'):
                    raise
                trace.append({'mode': 'native_reconnect', 'accepted': False, 'status': exc.code,
                              'from_anchor': prior_index, 'to_anchor': next_index,
                              'native_expanded': getattr(exc, 'expanded', 0),
                              'search_length_limit_xy': reconnect_limit,
                              'native_debug': getattr(exc, 'native_debug', None),
                              'global_unreachable_proven': False})
            except ValueError as exc:
                trace.append({'mode': 'native_reconnect', 'accepted': False, 'status': 'metric_limit', 'detail': str(exc)})
        history.append((index, current, len(output), accepted_width))
        history.extend(retained_checkpoints)
        index = next_index
        output.extend(parts)
        current = parts[-1].points[-1] if parts else target
        if rollback_frontier is not None and index > rollback_frontier:
            rollback_count, rollback_frontier = 0, None
    return Route(start, goal, tuple(output), expanded)
