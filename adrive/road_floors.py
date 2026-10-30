"""Small road-height hints for the capture worker, never floor evidence."""
from collections import OrderedDict,defaultdict
import hashlib
import json
import math


class RoadFloorHints:
    def __init__(self,asset_root):
        self.root=asset_root;self.loaded=False;self.index=None;self.cache=OrderedDict()
    def heights(self,point,pack):
        if self.root is None:return ()
        try:
            if not self.loaded:
                self.loaded=True;root=self.root/'roads';pointer=json.loads((root/'manifest.json').read_bytes())
                if pointer.get('kind')=='adrive-road-pointer':
                    target=(root/pointer['path']).resolve()
                    if not target.is_relative_to(root.resolve()):return ()
                    root=target.parent
                    entry=pointer['floor_index'];p=(self.root/'roads'/entry['path']).resolve()
                else:
                    entry=pointer['floor_index'];p=(root/entry['path']).resolve()
                if not p.is_relative_to(root.resolve()):return ()
                data=p.read_bytes()
                if hashlib.sha256(data).hexdigest()!=entry['sha256']:return ()
                index=json.loads(data)
                if index['navigation_identity']!=pack.identity or index['native_library_sha256']!=pack.raw['library']['sha256']:return ()
                self.index=index;self.directory=root
            if self.index is None:return ()
            row,col=next(iter(pack.around(point,0)));key=f'{point.c}-{row}-{col}'
            if key not in self.cache:
                entry=self.index['shards'].get(key)
                if not entry or entry['bytes']>8*1024*1024:return ()
                path=(self.directory/entry['path']).resolve()
                if not path.is_relative_to(self.directory.resolve()):return ()
                data=path.read_bytes()
                if len(data)!=entry['bytes'] or hashlib.sha256(data).hexdigest()!=entry['sha256']:return ()
                bins=defaultdict(set)
                for edge in json.loads(data):
                    for x,y,z in edge['points']:bins[(math.floor(x/16),math.floor(y/16))].add((x,y,z))
                self.cache[key]=bins
                while len(self.cache)>2:self.cache.popitem(last=False)
            bins=self.cache[key];bx,by=math.floor(point.x/16),math.floor(point.y/16);near=[]
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    near.extend((math.hypot(x-point.x,y-point.y),z) for x,y,z in bins.get((bx+dx,by+dy),())
                                if math.hypot(x-point.x,y-point.y)<=8)
            return tuple(z for _,z in sorted(near)[:16])
        except (OSError,ValueError,KeyError,TypeError,AttributeError):return ()
