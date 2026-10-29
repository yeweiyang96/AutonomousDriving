"""Resolve missing/placeholder player heights on the capture owner thread.

Some Forever captures report a constant zero from UnitPosition. Zero is still
a valid wire value, but cannot select a floor reliably in that capture stream.
Infer only the unique walkable surface at the original XY; never snap sideways
or use a planned route to decide which floor the player occupies.
"""
from dataclasses import replace
import math

from .models import Flags, PlanningError, Point
from .navmesh import Mesh, load_pack, validate_point


def height_status(sample):
    if sample.height_source == 'navmesh':
        return f'角色高度 {sample.z:.2f} 码（由当前位置唯一地面推定）'
    if sample.height_source == 'ambiguous':
        return '当前位置有多层地面，插件未提供可靠高度，无法确定角色所在楼层'
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
            self.close()
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
        if sample.z is not None and sample.z != 0:
            return replace(sample, raw_z=sample.z, height_source='client')
        result = replace(sample, z=None, raw_z=sample.z, height_source='unavailable')
        if not sample.flags & Flags.VALID:
            return result
        try:
            heights = self._surfaces(Point(sample.c, sample.x, sample.y))
            if len(heights) == 1 and math.isfinite(heights[0]):
                return replace(result, z=heights[0], height_source='navmesh')
            if len(heights) > 1:
                return replace(result, height_source='ambiguous')
            self.error = '原始水平坐标处没有可步行地面'
        except (PlanningError, OSError, ValueError) as exc:
            self.error = str(exc)
        return result

    def close(self):
        if self._mesh is not None:
            self._mesh.close()
            self._mesh = None
        self._area = None
