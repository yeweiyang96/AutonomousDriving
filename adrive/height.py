"""Resolve player ground height from XY on the capture owner thread.

The current addon supplies no altitude. Legacy altitude frames are rejected.
Track all feasible floors through observed motion and bounded walk connections.
Never use a planned route or a region name as proof of the player's floor.
"""
from dataclasses import replace
import math
import time

from .models import Flags, PlanningError, Point
from .navmesh import Mesh, load_pack, validate_point
from .regions import RegionContext


def height_status(sample):
    if sample.height_source == 'navmesh':
        return f'角色高度 {sample.z:.2f} 码（由当前位置唯一地面推定）'
    if sample.height_source == 'trajectory':
        return f'角色地面 {sample.z:.2f} 码（由连续轨迹与地面连通关系推定）'
    if sample.height_source == 'ambiguous':
        return '正在辨认所在楼层；请短距离步行，客户端会根据轨迹自动判断'
    if sample.height_source == 'unavailable':
        return '当前位置无法确定角色高度，请查看地面数据或采集诊断'
    if sample.z is None:
        return '精细导航缺少可靠的角色高度，无法确定所在楼层'
    return f'角色高度 {sample.z:.2f} 码（插件上报）'


class TerrainHeightResolver:
    """Lazy pack + one bounded 3x3 neighborhood, owned by a single worker."""

    def __init__(self, asset_root, *, cancel=None, pack_loader=load_pack, mesh_factory=Mesh):
        self.asset_root, self.cancel = asset_root, cancel
        self._pack_loader, self._mesh_factory = pack_loader, mesh_factory
        self._loaded, self._pack, self._mesh = False, None, None
        self._area, self._load_error, self._area_error = None, None, None
        self.error = ''
        self._previous = None
        self._candidates = ()
        self._last_result = None
        self._regions = RegionContext(asset_root)
        self._anchors = {}
        from .road_floors import RoadFloorHints
        self._road_hints = RoadFloorHints(asset_root)

    def _surfaces(self, point):
        validate_point(point)
        if not self._loaded:
            self._loaded = True
            try:
                self._pack = self._pack_loader(self.asset_root)
            except (PlanningError, OSError, ValueError) as exc:
                self._load_error = str(exc)
        if self._load_error:
            raise ValueError(self._load_error)
        if self._pack is None:
            raise ValueError('没有安装精细地面数据')
        keys = frozenset(self._pack.around(point))
        area = (point.c, keys)
        if area != self._area:
            self._close_mesh()
            self._area, self._area_error = area, None
            try:
                size = sum(self._pack.tiles[(point.c, *k)]['bytes'] for k in keys
                           if (point.c, *k) in self._pack.tiles)
                if size > 64 * 1024 * 1024:
                    raise ValueError('角色附近地面数据超过 64 MiB 预算')
                self._mesh = self._mesh_factory(self._pack, point.c, self.cancel)
                self._mesh.load(keys)
            except (PlanningError, OSError, ValueError) as exc:
                self._area_error = str(exc)
                if self._mesh is not None:
                    self._mesh.close()
                    self._mesh = None
        if self._area_error:
            raise ValueError(self._area_error)
        return self._mesh.heights(point)

    def resolve(self, sample):
        self.error = ''
        # Already-enriched log samples retain their original provenance.
        if sample.height_source != 'raw':
            return sample
        if sample.z is not None:
            raise ValueError('不再支持插件高度，请更新到仅输出水平位置的插件')
        result = replace(sample, z=None, raw_z=None, height_source='unavailable', floor_candidates=())
        unsafe = Flags.LOADING | Flags.DEAD | Flags.FLYING | Flags.SWIMMING | Flags.TRANSPORT | Flags.TAXI
        if not sample.flags & Flags.VALID or sample.flags & unsafe or sample.environment & 8:
            self._reset_track()
            return result
        previous = self._previous
        if previous and sample.session == previous.session and sample.seq == previous.seq:
            # A duplicate must not become a fresh geometric observation.
            return replace(self._last_result, captured_at=previous.captured_at)
        continuous = False
        if previous:
            dt = sample.captured_at - previous.captured_at
            continuous = (sample.session == previous.session and sample.c == previous.c
                          and 0 < (sample.seq - previous.seq) % 2**32 < 2**31
                          and 0 < dt <= .75
                          and sample.point.distance(previous.point) <= max(3., max(sample.speed, previous.speed)*dt*1.8+1))
        try:
            point = Point(sample.c, sample.x, sample.y)
            heights = tuple(h for h in self._surfaces(point) if math.isfinite(h))
            anchors = {h: replace(point,z=h) for h in heights}
            # XY telemetry is rounded to 1/16 yd. At polygon seams the
            # observed point may be just outside, although its quantization
            # cell intersects ground. Keep original XY; use a witness only
            # inside that cell to test connectivity, never a nearest-cell snap.
            for dx,dy in ((-.03125,0),(.03125,0),(0,-.03125),(0,.03125),
                          (-.03125,-.03125),(-.03125,.03125),(.03125,-.03125),(.03125,.03125)):
                witness=Point(sample.c,sample.x+dx,sample.y+dy)
                for h in self._surfaces(witness):
                    if math.isfinite(h) and not any(abs(h-old)<.35 for old in anchors):
                        anchors[h]=replace(witness,z=h)
            heights=tuple(anchors)
            # Dedicated validated roads order work, but cannot discard a
            # plausible surface or authorize the observer's current floor.
            hints=self._road_hints.heights(point,self._pack)
            if hints:heights=tuple(sorted(heights,key=lambda h:min(abs(h-z) for z in hints)))
            result = replace(result, region_name=self._regions.describe(sample, self._pack.build))
            source = 'navmesh'
            proven = True
            if continuous and self._candidates and heights:
                heights, proven = self._follow(previous, sample, heights, anchors)
                source = 'trajectory'
            if len(heights) == 1 and proven:
                result = replace(result, z=heights[0], height_source=source, floor_candidates=heights)
            elif heights:
                result = replace(result, height_source='ambiguous', floor_candidates=heights)
            else:
                self.error = '当前位置没有与已观察轨迹相符的可步行地面'
            self._previous, self._candidates, self._last_result = sample, heights, result
            self._anchors = {h:anchors[h] for h in heights}
        except (PlanningError, OSError, ValueError) as exc:
            self.error = str(exc)
            self._reset_track()
        return result

    def _follow(self, previous, sample, heights, anchors):
        distance = previous.point.distance(sample.point)
        if distance < .001:
            keep = tuple(h for h in heights if any(abs(h-old)<.1 for old in self._candidates))
            return keep, self._last_result.z is not None or len(keep) > 1
        # Actual small movements cannot contain a detour to a distant stairway.
        # Bound both XY and 3D path lengths and total work on the capture worker.
        limit = distance * 1.8 + .4
        deadline = time.monotonic() + .020
        if len(heights) * len(self._candidates) > 64:
            return heights, False
        keep, uncertain = [], False
        for h in heights:
            connected, unknown = False, False
            for old in self._candidates:
                if abs(h-old) > limit:
                    continue
                if time.monotonic() >= deadline:
                    unknown = True
                    continue
                a, b = self._anchors[old], anchors[h]
                try:
                    parts, _ = self._mesh.path(a, b, 1024, max_length_xy=limit)
                    # Native surface projection must not silently select a nearby floor.
                    if parts and (abs(parts[0].points[0].z-old)>.1 or abs(parts[-1].points[-1].z-h)>.1):
                        unknown = True
                        continue
                    length = sum(math.hypot(p.distance(q), p.z-q.z) for part in parts
                                 for p,q in zip(part.points,part.points[1:]))
                    if parts and length <= limit:
                        connected = True
                        break
                except PlanningError as exc:
                    if exc.code not in ('no_route', 'road_scope_unconnected', 'navmesh_outside'):
                        unknown = True
            if connected or unknown:
                keep.append(h)
                uncertain |= not connected
        return tuple(keep), not uncertain

    def _reset_track(self):
        self._previous = self._last_result = None
        self._candidates = ()
        self._anchors = {}

    def _close_mesh(self):
        if self._mesh is not None:
            self._mesh.close()
            self._mesh = None
        self._area = None

    def close(self):
        self._close_mesh()
        self._reset_track()
