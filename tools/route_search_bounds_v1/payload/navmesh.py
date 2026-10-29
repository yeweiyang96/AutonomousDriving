"""Walking navigation with policy-bounded road searches and explicit diagnostics.

Offline, height-aware static-collision navigation using the local Detour pack.

No connection to the game/server. Missing, incompatible or incomplete data never
falls back silently to an unchecked straight line or the old coarse grid.
"""
from __future__ import annotations

from contextlib import contextmanager
from datetime import datetime, timezone
import ctypes as C
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path
import struct
import threading

from .data import TILE_SIZE, WORLD_TOP
from .models import PlanningError, Point, Route, Segment
from .road_corridor import detour_metrics, follow_guide

ERRORS = {
    1: ('navmesh_format', '精细导航数据格式不兼容，请重新生成数据包'),
    2: ('navmesh_memory', '精细导航内存不足'),
    3: ('navmesh_outside', '所选位置不在可步行表面内；未将坐标移动到别处'),
    4: ('navmesh_height', '所选位置有多层地面，请选择起点或终点高度'),
    5: ('no_route', '静态碰撞网格中没有完整步行路线；没有返回部分路线'),
    6: ('budget', '精细导航搜索或数据预算已用尽'),
    7: ('cancelled', '规划已取消'),
    8: ('blocked_navmesh', '路线经过被排除的连接或非步行连接'),
}
CALLBACK = C.CFUNCTYPE(C.c_int)
NATIVE_BUDGET_REASONS = {
    0: 'not_set',
    1: 'expansion_limit',
    2: 'node_pool_exhausted',
    3: 'corridor_capacity',
    4: 'path_output_capacity',
    5: 'surface_query_failed',
    6: 'surface_polygon_capacity',
    7: 'surface_height_capacity',
}


def _budget_error(source, **details):
    """Add observable context while preserving the existing budget error code."""
    payload = json.dumps({'source': source, **details}, ensure_ascii=False,
                         separators=(',', ':'))
    return PlanningError(ERRORS[6][0], ERRORS[6][1] + ' | NAV_DEBUG ' + payload)


def checked(result, source='native', **details):
    if result == 6:
        raise _budget_error(source, rc=result, **details)
    if result:
        raise PlanningError(*ERRORS.get(result, ('navmesh_error', '精细导航失败')))


class Native:
    def __init__(self, path):
        self.dll = C.CDLL(str(path.resolve()))
        ptr, ints, floats, refs = C.c_void_p, C.POINTER(C.c_int), C.POINTER(C.c_float), C.POINTER(C.c_uint64)
        specs = {
            'ad_create': ([ptr, C.c_int, C.POINTER(ptr)], C.c_int),
            'ad_destroy': ([ptr], None),
            'ad_add': ([ptr, ptr, C.c_int], C.c_int),
            'ad_surfaces': ([ptr, floats, floats, C.c_int, ints], C.c_int),
            'ad_poly_key': ([ptr, C.c_uint64, ints], C.c_int),
            'ad_block': ([ptr, ints, ints], C.c_int),
            'ad_path': ([ptr, floats, floats, C.c_int, CALLBACK, floats, refs, C.c_int, ints, ints], C.c_int),
        }
        for name, (args, result) in specs.items():
            fn = getattr(self.dll, name)
            fn.argtypes, fn.restype = args, result
        # Optional: the old DLL still works, but reports diagnostics as unavailable.
        self.budget_reason = getattr(self.dll, 'ad_budget_reason', None)
        if self.budget_reason is not None:
            self.budget_reason.argtypes = [ptr, C.POINTER(C.c_uint32)]
            self.budget_reason.restype = C.c_int
        self.bounded_path = getattr(self.dll, 'ad_path_bounded_v3', None)
        if self.bounded_path is not None:
            self.bounded_path.argtypes = [ptr, floats, floats, C.c_int, C.c_float,
                                           CALLBACK, floats, refs, C.c_int, ints, ints]
            self.bounded_path.restype = C.c_int


def vector(point):
    return (C.c_float * 3)(point.x, point.y, point.z if point.z is not None else math.nan)


def validate_point(point):
    if (point.c not in (0, 1) or not all(math.isfinite(v) and abs(v) <= WORLD_TOP for v in (point.x, point.y))
            or point.z is not None and (not math.isfinite(point.z) or abs(point.z) > 20000)):
        raise PlanningError('navmesh_outside', '坐标超出精细导航数据范围')


class Mesh:
    def __init__(self, pack, c, cancel=None):
        self.pack, self.c, self.cancel = pack, c, cancel
        self.handle = C.c_void_p()
        data = pack.read(pack.maps[c])
        checked(pack.native.dll.ad_create(data, len(data), C.byref(self.handle)), source='native.ad_create', continent=c)
        self.loaded, self.bytes, self.keys = set(), 0, {}
        self.debug_context = {'stage': 'surface_query'}

    def close(self):
        if self.handle:
            self.pack.native.dll.ad_destroy(self.handle)
            self.handle = C.c_void_p()

    def cancelled(self):
        return bool(self.cancel and self.cancel.is_set())

    def load(self, keys):
        for key in sorted(keys):
            if self.cancelled():
                raise PlanningError(*ERRORS[7])
            entry = self.pack.tiles.get((self.c, *key))
            if key in self.loaded or entry is None:
                continue
            if self.bytes + entry['bytes'] > self.pack.max_bytes:
                raise _budget_error(
                    'python.tile_load', **self.debug_context,
                    continent=self.c, next_tile=list(key),
                    loaded_tiles=len(self.loaded), loaded_bytes=self.bytes,
                    next_tile_bytes=entry['bytes'], max_bytes=self.pack.max_bytes)
            data = self.pack.read(entry)
            checked(self.pack.native.dll.ad_add(self.handle, data, len(data)), source='native.ad_add', tile=list(key), **self.debug_context)
            self.bytes += len(data)
            self.loaded.add(key)

    def native_budget_details(self):
        fn = getattr(self.pack.native, 'budget_reason', None)
        if fn is None:
            return {'native_reason': 'unavailable_in_loaded_dll'}
        try:
            status = C.c_uint32()
            reason = fn(self.handle, C.byref(status))
            return {
                'native_reason': NATIVE_BUDGET_REASONS.get(reason, 'unknown'),
                'native_reason_code': reason,
                'native_status': f'0x{status.value:08x}',
            }
        except Exception as exc:
            # Diagnostic failures must not replace the original budget failure.
            return {'native_reason': 'diagnostic_read_failed',
                    'diagnostic_error': type(exc).__name__}

    def heights(self, point):
        result, count = (C.c_float * 32)(), C.c_int()
        rc = self.pack.native.dll.ad_surfaces(self.handle, vector(point), result, len(result), C.byref(count))
        if rc == 3:
            return []
        native_details = self.native_budget_details() if rc == 6 else {}
        checked(rc, source='native.ad_surfaces', **self.debug_context, **native_details)
        return list(result[:count.value])

    def key(self, ref):
        if ref not in self.keys:
            key = (C.c_int * 3)()
            checked(self.pack.native.dll.ad_poly_key(self.handle, ref, key), source='native.ad_poly_key', **self.debug_context)
            self.keys[ref] = ','.join(map(str, key))
        return self.keys[ref]

    def block(self, edges):
        prefix = f'nav:{self.pack.identity}:{self.c}:'
        for edge in edges:
            if not edge.startswith(prefix):
                continue
            try:
                a, b = edge[len(prefix):].split(':')
                ka, kb = tuple(map(int, a.split(','))), tuple(map(int, b.split(',')))
                if len(ka) != 3 or len(kb) != 3:
                    continue
            except ValueError:
                continue
            checked(self.pack.native.dll.ad_block(self.handle, (C.c_int * 3)(*ka), (C.c_int * 3)(*kb)), source='native.ad_block', **self.debug_context)

    def path(self, start, goal, budget, *, max_length_xy=None):
        if self.cancelled():
            raise PlanningError(*ERRORS[7])
        if max_length_xy is not None:
            if (not math.isfinite(max_length_xy) or max_length_xy < 0
                    or max_length_xy < start.distance(goal)-.01):
                raise PlanningError('navmesh_search_bound', '道路搜索长度限制无效')
            if self.pack.native.bounded_path is None:
                raise PlanningError('navmesh_capability', '当前 DLL 缺少道路范围搜索接口，请完成 DLL 更新后重新启动导航程序')
        cap = 65535
        values, refs, count, expanded = (C.c_float * (cap * 3))(), (C.c_uint64 * cap)(), C.c_int(), C.c_int()
        callback = CALLBACK(lambda: int(self.cancelled()))
        if max_length_xy is None:
            source = 'native.ad_path'
            rc = self.pack.native.dll.ad_path(self.handle, vector(start), vector(goal), budget, callback,
                                              values, refs, cap, C.byref(count), C.byref(expanded))
        else:
            source = 'native.ad_path_bounded_v3'
            rc = self.pack.native.bounded_path(self.handle, vector(start), vector(goal), budget,
                                               max_length_xy, callback, values, refs, cap,
                                               C.byref(count), C.byref(expanded))
        native_details = self.native_budget_details() if rc == 6 else {}
        search_details = ({'search_length_limit_xy': max_length_xy,
                           'search_scope': 'loaded_walk_graph_with_conservative_length_domain',
                           'global_unreachable_proven': False}
                          if max_length_xy is not None else {})
        try:
            if rc == 10 and max_length_xy is not None:
                payload = {'source': source, 'rc': rc, **self.debug_context, **search_details,
                           'passed_budget': budget, 'call_expanded': expanded.value,
                           'continent': self.c, 'loaded_tiles': len(self.loaded),
                           'loaded_bytes': self.bytes, 'max_bytes': self.pack.max_bytes,
                           'start': [start.x, start.y, start.z], 'goal': [goal.x, goal.y, goal.z]}
                error = PlanningError('road_scope_unconnected',
                    '当前已加载网格内，道路长度约束限定的搜索范围没有完整步行连接' +
                    ' | NAV_DEBUG ' + json.dumps(payload, ensure_ascii=False, separators=(',', ':')))
                error.search_scope = payload
                raise error
            checked(rc, source=source, **self.debug_context, **native_details, **search_details,
                    passed_budget=budget, call_expanded=expanded.value,
                    output_count=count.value, output_capacity=cap,
                    continent=self.c, loaded_tiles=len(self.loaded),
                    loaded_bytes=self.bytes, max_bytes=self.pack.max_bytes,
                    start=[start.x, start.y, start.z],
                    goal=[goal.x, goal.y, goal.z])
        except PlanningError as exc:
            exc.expanded = expanded.value
            exc.native_debug = {'source': source, 'rc': rc, **self.debug_context,
                                **native_details, **search_details, 'passed_budget': budget,
                                'call_expanded': expanded.value, 'continent': self.c,
                                'loaded_tiles': len(self.loaded), 'loaded_bytes': self.bytes,
                                'start': [start.x, start.y, start.z], 'goal': [goal.x, goal.y, goal.z]}
            raise
        points = [Point(self.c, *values[i * 3:i * 3 + 3]) for i in range(count.value)]
        if not points or start.distance(points[0]) > .02 or goal.distance(points[-1]) > .02:
            raise PlanningError('navmesh_endpoint', '精细路线未到达原始起终点')
        # Float32 geometry is allowed; the user's XY endpoints remain exact.
        points[0] = Point(self.c, start.x, start.y, points[0].z)
        points[-1] = Point(self.c, goal.x, goal.y, points[-1].z)
        segments = []
        for i, (a, b) in enumerate(zip(points, points[1:])):
            if a.distance(b) <= 1e-5:
                if abs(a.z - b.z) > .8:
                    raise PlanningError('navmesh_step', '路线包含无法步行通过的高度突变')
                continue
            edge_id = f'nav:{self.pack.identity}:{self.c}:{self.key(refs[i])}:{self.key(refs[i+1])}'
            segments.append(Segment((a, b), 'navmesh: static terrain + WMO/M2 collision', 'navmesh', edge_id))
        return segments, expanded.value


class NavMeshPack:
    def __init__(self, manifest):
        self.root = manifest.parent.resolve()
        raw = json.loads(manifest.read_text(encoding='utf-8'))
        if raw.get('schema') != 1 or raw.get('kind') != 'adrive-ground-navmesh' or raw.get('mmap_version') != 16:
            raise ValueError('Unsupported navigation pack')
        if raw['settings']['walkable_climb_yards'] > .8 or raw['settings']['agent_radius_yards'] < .5:
            raise ValueError('Unsupported navigation agent settings')
        self.raw, self.identity, self.build = raw, raw['identity'], raw['client']['version']
        self.record_diagnostics = True
        self.last_route_diagnostics = None
        self.tiles = {(t['c'], t['row'], t['col']): t for t in raw['tiles']}
        self.maps = {t['c']: t for t in raw['maps']}
        if len(self.tiles) != len(raw['tiles']) or set(self.maps) != {0, 1}:
            raise ValueError('Incomplete or duplicate navigation index')
        for t in raw['tiles']:
            if t['c'] not in (0, 1) or not 0 <= t['row'] < 64 or not 0 <= t['col'] < 64:
                raise ValueError('Invalid navigation tile coordinate')
        for entry in [*raw['maps'], *raw['tiles'], raw['library']]:
            if not (self.root / entry['path']).resolve().is_relative_to(self.root):
                raise ValueError('Navigation path escapes pack')
        self.read(raw['library'])
        self.native = Native(self.root / raw['library']['path'])
        self.max_bytes = 384 * 1024 * 1024

    def read(self, entry):
        try:
            data = (self.root / entry['path']).read_bytes()
        except OSError as exc:
            raise PlanningError('navmesh_missing', f'精细导航文件缺失：{entry["path"]}') from exc
        if len(data) != entry['bytes'] or hashlib.sha256(data).hexdigest() != entry['sha256']:
            raise PlanningError('navmesh_integrity', f'精细导航文件校验失败：{entry["path"]}')
        return data

    @staticmethod
    def around(point, radius=1):
        row, col = math.floor((WORLD_TOP-point.x)/TILE_SIZE), math.floor((WORLD_TOP-point.y)/TILE_SIZE)
        return {(r, c) for r in range(max(0,row-radius), min(64,row+radius+1))
                for c in range(max(0,col-radius), min(64,col+radius+1))}

    @contextmanager
    def mesh(self, c, cancel=None):
        mesh = Mesh(self, c, cancel)
        try:
            yield mesh
        finally:
            mesh.close()

    def heights(self, point):
        validate_point(point)
        with self.mesh(point.c) as mesh:
            mesh.load(self.around(point))
            return mesh.heights(point)

    def plan(self, start, goal, guide=None, blocked_edges=frozenset(), cancel=None, max_expansions=500000):
        def point(p):
            return {'c': p.c, 'x': p.x, 'y': p.y, 'z': p.z}

        def segments(items, limit):
            return [{'points': [point(p) for p in s.points], 'source': s.source,
                     'reliability': s.reliability, 'edge_id': s.edge_id}
                    for s in items[:limit]]

        diagnostic = {'schema': 1, 'planner_version': 'road-corridor-v3-bounded',
                      'started_at_utc': datetime.now(timezone.utc).isoformat(),
                      'navigation_identity': self.identity,
                      'native_library_sha256': self.raw['library']['sha256'],
                      'start': point(start), 'goal': point(goal),
                      'blocked_edges': sorted(blocked_edges), 'max_expansions': max_expansions,
                      'guide': None if guide is None else {
                          'start': point(guide.start), 'goal': point(guide.goal),
                          'segments': segments(guide.segments, 10000),
                          'truncated': len(guide.segments) > 10000},
                      'road_trace': [], 'complete': not guide or len(guide.segments) <= 10000}
        self.last_route_diagnostics = diagnostic
        try:
            route = self._plan_geometry(start, goal, guide, blocked_edges, cancel,
                                        max_expansions, diagnostic)
            if cancel and cancel.is_set():
                raise PlanningError('cancelled', '规划已取消')
            diagnostic['result'] = {'status': 'success', 'start': point(route.start),
                                    'goal': point(route.goal), 'expanded': route.expanded,
                                    'length_xy': route.distance,
                                    'length_3d': sum(math.dist((a.x,a.y,a.z), (b.x,b.y,b.z))
                                                     for a,b in zip(route.points,route.points[1:])),
                                    'segments': segments(route.segments, 20000),
                                    'truncated': len(route.segments) > 20000}
            diagnostic['complete'] = diagnostic['complete'] and len(route.segments) <= 20000
            return route
        except Exception as exc:
            diagnostic['result'] = {'status': 'error', 'code': getattr(exc, 'code', type(exc).__name__),
                                    'message': str(exc), 'expanded': getattr(exc, 'expanded', None)}
            raise
        finally:
            diagnostic['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
            if len(diagnostic['road_trace']) > 2000:
                diagnostic['road_trace_count'] = len(diagnostic['road_trace'])
                diagnostic['road_trace'] = diagnostic['road_trace'][:1000] + diagnostic['road_trace'][-1000:]
                diagnostic['complete'] = False
            if getattr(self, 'record_diagnostics', False):
                self._save_route_diagnostics(diagnostic)

    def _save_route_diagnostics(self, diagnostic):
        # Diagnostics never change route success or trigger device activity.
        # One atomically replaced local snapshot bounds retained data.
        temporary = None
        try:
            project = self.root.parent.parent.resolve()
            directory = project / 'runtime' / 'route-quality'
            if not directory.resolve().is_relative_to(project):
                raise ValueError('Diagnostic directory escapes the project')
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / 'last-plan.json'
            if not path.resolve().is_relative_to(directory.resolve()):
                raise ValueError('Diagnostic path escapes its directory')
            diagnostic['report_path'] = str(path)
            data = (json.dumps(diagnostic, ensure_ascii=False, separators=(',', ':'), allow_nan=False)+'\n').encode('utf-8')
            if len(data) > 8*1024*1024:
                summary = {k:v for k,v in diagnostic.items() if k not in ('guide','road_trace','result')}
                summary.update(complete=False, diagnostic_limit_bytes=8*1024*1024,
                               result={k:v for k,v in diagnostic['result'].items() if k!='segments'})
                data = (json.dumps(summary, ensure_ascii=False, separators=(',', ':'), allow_nan=False)+'\n').encode('utf-8')
            handle, name = tempfile.mkstemp(prefix='.route-', suffix='.tmp', dir=directory)
            temporary = Path(name)
            with os.fdopen(handle, 'wb') as stream:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        except (OSError, ValueError, TypeError) as exc:
            diagnostic['report_write_error'] = type(exc).__name__ + ': ' + str(exc)
        finally:
            if temporary is not None:
                try:
                    temporary.unlink(missing_ok=True)
                except OSError:
                    pass

    def _plan_geometry(self, start, goal, guide, blocked_edges, cancel, max_expansions, diagnostic):
        validate_point(start)
        validate_point(goal)
        if start.c != goal.c:
            raise PlanningError('different_continent', '精细导航不能跨大陆')
        with self.mesh(start.c, cancel) as mesh:
            # Keep actual road vertices as preferences, not the old 16.7-yard
            # terrain cell centres. Wilderness paths use the fine mesh directly.
            reference = [start]
            road_segments = [s for s in guide.segments if s.reliability in ('reviewed', 'candidate')] if guide else []
            for segment in road_segments:
                for point in segment.points:
                    if reference[-1].distance(point) > 1e-5:
                        reference.append(point)
            if reference[-1].distance(goal) > 1e-5 or len(reference) == 1:
                reference.append(goal)
            else:
                reference[-1] = goal
            keys = set()
            # Load the corridor around the reference route, including boundaries.
            for a, b in zip(reference, reference[1:]):
                steps = max(1, math.ceil(a.distance(b)/(TILE_SIZE/2)))
                for i in range(steps+1):
                    t = i/steps
                    keys.update(self.around(Point(a.c, a.x+(b.x-a.x)*t, a.y+(b.y-a.y)*t)))
            if not keys:
                keys = self.around(start)
            mesh.debug_context = {'stage': 'initial_load', 'plan_budget': max_expansions,
                                  'expanded_before': 0}
            mesh.load(keys)
            mesh.block(blocked_edges)
            mesh.debug_context = {'stage': 'endpoints', 'plan_budget': max_expansions,
                                  'expanded_before': 0}
            resolved = []
            for label, p in (('起点', start), ('终点', goal)):
                hs = mesh.heights(p)
                if not hs:
                    if p.z is not None and mesh.heights(Point(p.c, p.x, p.y)):
                        raise PlanningError('navmesh_height_mismatch', f'{label}高度与可步行地面不符，请确认角色高度或所选楼层')
                    raise PlanningError(ERRORS[3][0], label + '不在可步行表面内；未将坐标移动到别处')
                if p.z is None and len(hs)>1:
                    raise PlanningError(ERRORS[4][0], label + '有多层地面，请确认角色高度或选择对应楼层')
                if p.z is not None:
                    hs.sort(key=lambda h:abs(h-p.z))
                    if len(hs)>1 and abs(abs(hs[0]-p.z)-abs(hs[1]-p.z)) < .35:
                        raise PlanningError(*ERRORS[4])
                resolved.append(Point(p.c,p.x,p.y,hs[0]))
            start,goal = resolved
            reference[0], reference[-1] = start, goal
            if start.distance(goal) <= 1e-7 and abs(start.z-goal.z) < .35:
                return Route(start, goal, (), 0)
            expanded, prefix, fallback_start = 0, (), start
            fallback_reference = None
            if road_segments:
                try:
                    return follow_guide(mesh, start, goal, guide, max_expansions,
                                        trace=diagnostic['road_trace'])
                except PlanningError as exc:
                    if exc.code != 'road_guidance_unreachable':
                        raise
                    expanded = exc.expanded
                    prefix, fallback_start = exc.prefix_segments, exc.resume_point
                    fallback_reference = (fallback_start, *exc.remaining_reference)
                    diagnostic['road_trace'].append({
                        'mode': 'remaining_native_fallback', 'reason': exc.code,
                        'retained_prefix_segments': len(prefix),
                        'resume_point': {'c': fallback_start.c, 'x': fallback_start.x,
                                         'y': fallback_start.y, 'z': fallback_start.z},
                        'remaining_reference_range': exc.remaining_reference_range,
                        'expanded_before': expanded})
            # This is the same length acceptance used after a completed fallback.
            # The native domain is a necessary geometric superset, not a new
            # permission to cross water or a proof about unloaded tiles.
            fallback_limit = (detour_metrics(fallback_reference, fallback_reference)['length_limit_xy']
                              if fallback_reference is not None else None)
            for radius in (1, 2, 4):
                if mesh.cancelled():
                    error = PlanningError('cancelled', '规划已取消')
                    error.expanded = expanded
                    raise error
                if expanded >= max_expansions:
                    error = PlanningError('budget', '道路引导搜索预算已用尽；没有返回部分路线')
                    error.expanded = expanded
                    raise error
                mesh.debug_context = {'stage': 'fallback_load', 'radius': radius,
                                      'plan_budget': max_expansions,
                                      'expanded_before': expanded}
                if radius > 1:
                    extra = {(r+dr,c+dc) for r,c in keys for dr in range(-radius,radius+1)
                             for dc in range(-radius,radius+1) if 0<=r+dr<64 and 0<=c+dc<64}
                    mesh.load(extra)
                    mesh.block(blocked_edges)
                mesh.debug_context['stage'] = 'fallback_path'
                try:
                    parts, count = mesh.path(fallback_start, goal, max_expansions-expanded,
                                             max_length_xy=fallback_limit)
                except PlanningError as exc:
                    expanded += getattr(exc, 'expanded', 0)
                    if exc.code == 'road_scope_unconnected':
                        diagnostic['road_trace'].append({
                            'mode': 'bounded_remaining_fallback', 'radius': radius,
                            'status': 'road_scope_unconnected', 'accepted': False,
                            'native_expanded': getattr(exc, 'expanded', 0), 'expanded_total': expanded,
                            'search_length_limit_xy': fallback_limit,
                            'scope': getattr(exc, 'search_scope', {}), 'global_unreachable_proven': False})
                        if radius == 4:
                            error = PlanningError('road_connection_unverified',
                                '道路参考连接未验证：在已加载网格和道路长度约束限定的搜索范围内，'
                                '无法形成完整步行连接；请检查道路连接与导航表面数据。'
                                ' | NAV_DEBUG ' + json.dumps(getattr(exc, 'search_scope', {}),
                                                           ensure_ascii=False, separators=(',', ':')))
                            error.expanded = expanded
                            raise error from exc
                    if exc.code not in ('no_route', 'road_scope_unconnected') or radius == 4:
                        exc.expanded = expanded
                        raise
                    continue
                # Charge a completed native query exactly once before judging
                # its quality. Rejected candidates still consume the budget.
                expanded += count
                if mesh.cancelled():
                    error = PlanningError('cancelled', '规划已取消')
                    error.expanded = expanded
                    raise error
                if fallback_reference is not None:
                    points = Route(fallback_start, goal, tuple(parts)).points
                    quality = detour_metrics(points, fallback_reference)
                    accepted = not quality['excessive_detour']
                    decision = {'mode': 'remaining_native_fallback_quality',
                                'radius': radius, 'accepted': accepted,
                                'status': 'accepted_native_detour' if accepted else 'excessive_detour',
                                'native_expanded': count, 'expanded_total': expanded,
                                **quality}
                    if not accepted:
                        decision['candidate_point_count'] = len(points)
                        # Keep complete small candidates for diagnosis only;
                        # never emit an unchecked or truncated walking path.
                        decision['candidate_points'] = ([[p.x, p.y, p.z] for p in points]
                                                        if len(points) <= 2048 else [])
                        decision['candidate_geometry_omitted'] = len(points) > 2048
                    diagnostic['road_trace'].append(decision)
                    if not accepted:
                        error = PlanningError('road_connection_unverified',
                            '道路参考连接未验证：剩余参考路线约 %.1f 码，原生绕行约 %.1f 码；'
                            '未采用这条明显绕远的路线，请检查该处选点与道路或桥面的导航网格。' %
                            (quality['reference_length_xy'], quality['route_length_xy']))
                        error.expanded = expanded
                        error.fallback_quality = quality
                        raise error
                return Route(start, goal, (*prefix, *parts), expanded)


def load_pack(asset_root):
    if asset_root is None:
        return None
    manifest = asset_root / 'navigation/manifest.json'
    if not manifest.exists():
        return None
    try:
        return NavMeshPack(manifest)
    except PlanningError:
        raise
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        raise PlanningError('navmesh_invalid', f'精细导航包不可用：{exc}') from exc
