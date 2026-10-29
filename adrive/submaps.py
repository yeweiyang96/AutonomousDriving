"""Transparent submaps in the same world-yard coordinate system as the base map."""
from __future__ import annotations
from dataclasses import dataclass
import json
import math
from pathlib import Path


def smooth(value):
    t=min(1.,max(0.,value))
    return t*t*(3-2*t)


@dataclass(frozen=True)
class OverlayTile:
    area: str
    c: int
    n: int
    col: int
    row: int
    north: float
    west: float
    size: float
    path: Path


class Submaps:
    def __init__(self, areas=(), *, error='', build=''):
        self.areas=list(areas)
        self.error,self.build=error,build

    @classmethod
    def from_bundle(cls,bundle):
        path=bundle.asset_root/'submaps/manifest.json' if bundle.asset_root else None
        if path is None or not path.exists():return cls()
        try:return cls.load(path)
        except (OSError,ValueError,TypeError,KeyError) as exc:
            return cls(error=f'子地图不可用：{exc}')

    @classmethod
    def load(cls,path):
        return cls.from_manifest(path.parent,json.loads(path.read_text(encoding='utf-8')))

    @classmethod
    def from_manifest(cls,root,raw):
        root=root.resolve()
        if raw['schema']!=1 or raw['kind']!='wow-world-submap-overlays' or raw['tile_pixels']!=512:
            raise ValueError('子地图格式不兼容')
        if not raw['areas'] or len(raw['areas'])!=raw['area_count']:
            raise ValueError('子地图清单不完整')
        areas=[];ids=set();total=0
        for a in raw['areas']:
            if not isinstance(a['id'],str) or a['id'] in ids:raise ValueError('子地图编号重复')
            ids.add(a['id'])
            ppy=a['pixels_per_yard']
            if (a['c'] not in (0,1) or a['kind'] not in ('city','cave','micro') or
                not all(isinstance(v,(int,float)) and math.isfinite(v) for v in
                        (ppy,a['north'],a['west'],*a['bounds'])) or
                not .01<=ppy<=16 or len(a['bounds'])!=4 or
                not all(type(a[k]) is int and 0<a[k]<=32768 for k in ('width','height'))):
                raise ValueError('子地图坐标无效')
            x0,y0,x1,y1=a['bounds']
            if not x0<x1 or not y0<y1:raise ValueError('子地图边界无效')
            tiles={};levels=set()
            for t in a['tiles']:
                n,col,row=(t[k] for k in ('n','col','row'))
                if (any(type(v)is not int for v in (n,col,row)) or n not in (1,2,4,8,16,32,64) or
                    not 0<=col<math.ceil(a['width']/(512*n)) or not 0<=row<math.ceil(a['height']/(512*n))):
                    raise ValueError('子地图图片坐标无效')
                filename=(root/t['path']).resolve()
                if not filename.is_relative_to(root) or filename.suffix.lower()!='.webp':
                    raise ValueError('子地图图片路径无效')
                key=(n,col,row)
                if key in tiles:raise ValueError('子地图图片重复')
                size=512*n/ppy
                tiles[key]=OverlayTile(a['id'],a['c'],n,col,row,a['north']-row*size,a['west']-col*size,size,filename)
                levels.add(n)
            if not tiles or 1 not in levels:raise ValueError('子地图缺少原始精度')
            total+=len(tiles)
            areas.append(dict(a,tiles=tiles,levels=sorted(levels)))
        if total!=raw['tile_count']:raise ValueError('子地图图片数量不完整')
        return cls(areas,build=raw['build']['version'])

    @staticmethod
    def opacity(a,view):
        x0,y0,x1,y1=a['bounds']
        fill=min(view.width/((y1-y0)*view.scale),view.height/((x1-x0)*view.scale))
        return smooth((4-fill)/(4-1.8))*smooth((view.scale-.12)/.25)

    def visible(self,view):
        west=view.west+view.width/2/view.scale
        east=view.west-view.width/2/view.scale
        north=view.north+view.height/2/view.scale
        south=view.north-view.height/2/view.scale
        result=[]
        for a in self.areas:
            if a['c']!=view.c:continue
            if (a['west']<east or a['west']-a['width']/a['pixels_per_yard']>west or
                a['north']<south or a['north']-a['height']/a['pixels_per_yard']>north):continue
            opacity=self.opacity(a,view)
            if opacity<1/64:continue
            target=a['pixels_per_yard']/view.scale
            n=max((n for n in a['levels'] if n<=target),default=1)
            # Return current and coarse fallback levels; MapView avoids decoding
            # hidden intermediate levels and removes fallback once fine is ready.
            for (level,col,row),tile in a['tiles'].items():
                if level!=n and level!=a['levels'][-1]:continue
                if tile.west>=east and tile.west-tile.size<=west and tile.north>=south and tile.north-tile.size<=north:
                    result.append((tile,round(opacity*32)/32,level==n))
        return result

    def zoom_limit(self,view,point):
        limit=4.
        for a in self.areas:
            x0,y0,x1,y1=a['bounds']
            if a['c']==point.c and x0<=point.x<=x1 and y0<=point.y<=y1:
                limit=max(limit,min(32.,.8*min(view.width/(y1-y0),view.height/(x1-x0))))
        return limit
