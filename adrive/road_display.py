"""Independent, asynchronous road artwork. Never loads an execution graph/DLL.

The optional display asset and every viewport/LOD query stay in an isolated
process, so artwork CPU work cannot hold the capture interpreter's GIL. The
parent receives only compact visible XY arrays. Neither path authorizes movement.
"""
from __future__ import annotations

from collections import defaultdict, OrderedDict
from concurrent.futures import ThreadPoolExecutor
from array import array
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import pickle
import subprocess
import sys
import threading


@dataclass(frozen=True)
class DisplayLine:
    id: str
    c: int
    kind: str
    points: tuple


@dataclass(frozen=True)
class DisplayFrame:
    c: int
    bounds: tuple
    scale: float
    lines: tuple

    def covers(self,c,bounds,scale):
        a,b,d,e=self.bounds;x,y,u,v=bounds
        return self.c==c and abs(self.scale-scale)<1e-10 and a<=x<=y<=b and d<=u<=v<=e


@dataclass(frozen=True)
class PackedPoints:
    """Compact visible XY coordinates; avoids unpacking the full source graph."""
    values: array
    def __len__(self):return len(self.values)//2
    def __iter__(self):
        for i in range(0,len(self.values),2):yield (self.values[i],self.values[i+1])
    def __getitem__(self,index):
        if isinstance(index,slice):return tuple(self)[index]
        if index<0:index+=len(self)
        if not 0<=index<len(self):raise IndexError(index)
        return self.values[index*2],self.values[index*2+1]


def _normalise(rows):
    result=[];seen=set()
    for number,row in enumerate(rows):
        c=row['c'];kind=row.get('kind','road')
        if c not in (0,1) or kind not in ('road','connector'):continue
        pts=[]
        for point in row['points']:
            if len(point)<2:raise ValueError('Invalid display road coordinate')
            p=tuple(float(v) for v in point[:2])
            if len(point)>2 and point[2] is not None:p+=(float(point[2]),)
            if len(p)<2 or not all(math.isfinite(v) for v in p):raise ValueError('Invalid display road coordinate')
            if not pts or p!=pts[-1]:pts.append(p)
        if len(pts)<2:continue
        pts=tuple(pts);key=(c,kind,min(pts,tuple(reversed(pts))))
        if key in seen:continue
        seen.add(key);result.append(DisplayLine(str(row.get('id',number)),c,kind,pts))
    return result


def merge_lines(rows):
    """Join degree-two artwork endpoints; no inferred topological connections."""
    lines=_normalise(rows);adj=defaultdict(list)
    def endpoint(line,p):return (line.c,line.kind,p)
    for i,line in enumerate(lines):
        adj[endpoint(line,line.points[0])].append(i);adj[endpoint(line,line.points[-1])].append(i)
    unused=set(range(len(lines)));out=[]
    def walk(i,start):
        first=lines[i];pts=[];ids=[];node=start
        while i in unused:
            unused.remove(i);line=lines[i];following=line.points if endpoint(line,line.points[0])==node else tuple(reversed(line.points))
            pts.extend(following if not pts else following[1:]);ids.append(line.id)
            node=endpoint(line,following[-1])
            if len(adj[node])!=2:break
            choices=[j for j in adj[node] if j in unused]
            if not choices:break
            i=choices[0]
        out.append(DisplayLine('|'.join(ids),first.c,first.kind,tuple(pts)))
    for node,indices in adj.items():
        if len(indices)!=2:
            for i in indices:
                if i in unused:walk(i,node)
    while unused:
        i=min(unused);walk(i,endpoint(lines[i],lines[i].points[0]))
    return out


def simplify(points,tolerance):
    """RDP in world XY, solely for subpixel drawing; endpoints stay exact."""
    if len(points)<3:return points
    keep={0,len(points)-1};todo=[(0,len(points)-1)];threshold=tolerance*tolerance
    while todo:
        lo,hi=todo.pop();a,b=points[lo],points[hi];dx,dy=b[0]-a[0],b[1]-a[1];norm=dx*dx+dy*dy
        greatest=threshold;at=None
        for i in range(lo+1,hi):
            p=points[i];t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/norm)) if norm else 0.
            d=(p[0]-a[0]-t*dx)**2+(p[1]-a[1]-t*dy)**2
            if d>greatest:greatest,at=d,i
        if at is not None:keep.add(at);todo.extend(((lo,at),(at,hi)))
    return tuple(points[i] for i in sorted(keep))


class RoadDisplayIndex:
    def __init__(self,rows):
        self.lines=merge_lines(rows);self.boxes=[];self.bins=defaultdict(list);self.wide=defaultdict(list)
        self.continents=defaultdict(list);self.cache=OrderedDict();self.cached_points=0
        for i,line in enumerate(self.lines):
            xs=[p[0] for p in line.points];ys=[p[1] for p in line.points]
            box=(min(xs),max(xs),min(ys),max(ys));self.boxes.append(box);self.continents[line.c].append(i)
            ax,bx,ay,by=(math.floor(v/512) for v in box)
            if (bx-ax+1)*(by-ay+1)>256:self.wide[line.c].append(i);continue
            for x in range(ax,bx+1):
                for y in range(ay,by+1):self.bins[(line.c,x,y)].append(i)

    def query(self,c,bounds,scale,cancel=None):
        margin=120/scale;x0,x1,y0,y1=bounds;area=(x0-margin,x1+margin,y0-margin,y1+margin)
        ax,bx,ay,by=(math.floor(v/512) for v in area)
        if (bx-ax+1)*(by-ay+1)>len(self.bins)*2:
            candidates=self.continents[c]
        else:
            candidates=set(self.wide[c])
            for x in range(ax,bx+1):
                for y in range(ay,by+1):candidates.update(self.bins.get((c,x,y),()))
        level=math.floor(math.log2(.75/scale));tolerance=2.**level;out=[]
        for n,i in enumerate(sorted(candidates)):
            if cancel and n%128==0 and cancel.is_set():return None
            a,b,d,e=self.boxes[i]
            if b<area[0] or a>area[1] or e<area[2] or d>area[3] or max(b-a,e-d)*scale<.5:continue
            key=(i,level);line=self.lines[i]
            if key not in self.cache:
                points=simplify(line.points,tolerance);self.cache[key]=points;self.cached_points+=len(points)
                while self.cached_points>100000 and len(self.cache)>1:
                    _,old=self.cache.popitem(last=False);self.cached_points-=len(old)
            else:self.cache.move_to_end(key)
            out.append(DisplayLine(str(i),line.c,line.kind,self.cache[key]))
        return DisplayFrame(c,area,scale,tuple(out))


def _published_index(request):
    source=Path(request['path']);data=source.read_bytes()
    if len(data)!=request['bytes'] or hashlib.sha256(data).hexdigest()!=request['sha256']:
        raise ValueError('Road display asset integrity failure')
    raw=json.loads(data)
    if raw.get('schema')!=1 or raw.get('kind')!='adrive-road-display':raise ValueError('Unsupported road artwork')
    rows=raw['roads']
    if len(rows)>100000 or sum(len(r['points']) for r in rows)>2000000:raise ValueError('Road artwork exceeds display budget')
    return RoadDisplayIndex(rows)


def serve():
    """One isolated artwork owner: JSON/index/LOD never run beside capture."""
    read=sys.stdin.buffer;write=sys.stdout.buffer
    def reply(value):pickle.dump(value,write,protocol=pickle.HIGHEST_PROTOCOL);write.flush()
    try:
        request=pickle.load(read)
        if 'display' in request:index=_published_index(request['display'])
        else:
            rows=request.get('rows')
            if rows is None:
                raw=json.loads(Path(request['bundle']).read_bytes());nodes=raw['nodes'];rows=[]
                for e in raw['edges']:
                    a,b=nodes[e['a']],nodes[e['b']]
                    if e['kind']=='walk' and a[0] in (0,1):rows.append(dict(id=e['id'],c=a[0],points=[a[1:3],b[1:3]]))
            index=RoadDisplayIndex(rows)
        reply(('ready',len(index.lines)))
        while True:
            request=pickle.load(read)
            frame=index.query(*request)
            lines=tuple(DisplayLine(line.id,line.c,line.kind,PackedPoints(array('d',
                (v for p in line.points for v in p[:2])))) for line in frame.lines)
            reply(('frame',DisplayFrame(frame.c,frame.bounds,frame.scale,lines)))
    except EOFError:return
    except Exception as exc:
        try:reply(('error',str(exc)))
        except (OSError,BrokenPipeError):pass


class _DisplayProcess:
    def __init__(self):
        self.process=subprocess.Popen([sys.executable,'-m','adrive.road_display','--serve'],
            cwd=Path(__file__).resolve().parents[1],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    def call(self,request,expected):
        pickle.dump(request,self.process.stdin,protocol=pickle.HIGHEST_PROTOCOL);self.process.stdin.flush()
        response,value=pickle.load(self.process.stdout)
        if response!=expected:raise ValueError(value)
        return value
    def start(self,request):return self.call(request,'ready')
    def query(self,*request):return self.call(request,'frame')
    def stop(self):
        if self.process.poll() is None:self.process.kill()
    def close(self):
        self.stop();self.process.wait()
        self.process.stdin.close();self.process.stdout.close()


class RoadDisplaySource:
    def __init__(self,bundle):
        self.bundle=bundle;self.frame=None;self.error='';self.origin='';self.line_count=0
        self._client=None;self._child=None;self._closed=threading.Event()
        self._pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='road-artwork')
        self._load_future=self._pool.submit(self._load);self._query_future=None;self._desired=None;self._generation=0

    @property
    def loading(self):return self._load_future is not None
    @property
    def pending(self):return self.loading or self._query_future is not None

    def _load(self):
        if self._closed.is_set():return None
        root=self.bundle.asset_root
        try:
            if root is not None:
                directory=Path(root)/'roads';manifest=directory/'manifest.json'
                # Old execution manifests can be tens of megabytes. They are
                # never decoded for artwork: the existing bundle is enough.
                if manifest.exists() and manifest.stat().st_size<=1024*1024:
                    raw=json.loads(manifest.read_bytes());entry=raw.get('display')
                    if entry:
                        path=(directory/entry['path']).resolve()
                        if not path.is_relative_to(directory.resolve()):raise ValueError('Road artwork escapes pack')
                        if not 0<entry['bytes']<=128*1024*1024:raise ValueError('Road artwork exceeds byte budget')
                        self._child=_DisplayProcess()
                        if self._closed.is_set():self._child.close();return None
                        count=self._child.start(dict(display={**entry,'path':str(path)}))
                        self.origin='published';return self._child,count
        except (OSError,ValueError,KeyError,TypeError,EOFError) as exc:
            self.error=str(exc)
            if self._child:self._child.close()
        if self._closed.is_set():return None
        if self.bundle.source_path:request=dict(bundle=str(self.bundle.source_path))
        else:
            rows=[]
            for e in self.bundle.edges:
                a,b=self.bundle.nodes[e.a],self.bundle.nodes[e.b]
                if e.kind=='walk' and a.c in (0,1):rows.append(dict(id=e.id,c=a.c,points=((a.x,a.y),(b.x,b.y))))
            request=dict(rows=rows)
        self._child=_DisplayProcess()
        if self._closed.is_set():self._child.close();return None
        count=self._child.start(request);self.origin='original_bundle';return self._child,count

    @staticmethod
    def viewport(view):
        dx=view.height/(2*view.scale);dy=view.width/(2*view.scale)
        return view.c,(view.north-dx,view.north+dx,view.west-dy,view.west+dy),view.scale

    def request(self,view):
        request=self.viewport(view)
        if request!=self._desired:self._desired=request;self._generation+=1
        if self.frame and self.frame.covers(*request):return
        self._launch()

    def _launch(self):
        if self._closed.is_set() or self._client is None or self._query_future is not None or self._desired is None:return
        if self.frame and self.frame.covers(*self._desired):return
        self._query_generation=self._generation
        self._query_future=self._pool.submit(self._client.query,*self._desired)

    def poll(self):
        changed=False
        if self._load_future is not None and self._load_future.done():
            try:
                result=self._load_future.result()
                if result:self._client,self.line_count=result
            except Exception as exc:self.error=str(exc)
            self._load_future=None;changed=True;self._launch()
        if self._query_future is not None and self._query_future.done():
            try:frame=self._query_future.result()
            except Exception as exc:self.error=str(exc);frame=None
            self._query_future=None
            if frame is not None and self._query_generation==self._generation:self.frame=frame;changed=True
            elif self._query_generation!=self._generation:self._launch()
        return changed

    def close(self):
        if self._closed.is_set():return
        self._closed.set()
        if self._child:
            self._child.stop()
            self._pool.submit(self._child.close)
        self._pool.shutdown(wait=False,cancel_futures=False)


if __name__=='__main__':
    if len(sys.argv)==2 and sys.argv[1]=='--serve':
        from adrive.road_display import serve as run_server
        run_server();raise SystemExit(0)
    raise SystemExit('Internal road display worker')
