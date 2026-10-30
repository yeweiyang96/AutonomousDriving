"""Version-bound road graph, lazy geometry shards and road-only planning.

Only endpoint access and explicit graph connectors can leave a road. Connectors
retain their complete collision-verified geometry and are rechecked in a local
corridor, never replaced by an unrestricted remainder query. Source review
labels never influence route cost.
"""
from __future__ import annotations
from collections import defaultdict, OrderedDict
import hashlib
import heapq
import itertools
import json
import math
from pathlib import Path

from .models import Point, Route, PlanningError
from .navmesh import Mesh


def length(points):
    return sum(a.distance(b) for a,b in zip(points,points[1:]))


def _edge_box(edge,a,b):
    """Full geometry bounds in new packs; conservative legacy edge envelope."""
    bounds=edge.get('bounds')
    if bounds is None:
        return (min(a.x,b.x)-24.01,max(a.x,b.x)+24.01,
                min(a.y,b.y)-24.01,max(a.y,b.y)+24.01)
    if (not isinstance(bounds,(list,tuple)) or len(bounds)!=4
            or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in bounds)
            or bounds[0]>bounds[1] or bounds[2]>bounds[3]):
        raise PlanningError('roads_invalid','道路完整几何边界无效')
    if any(not (bounds[0]-.02<=p.x<=bounds[1]+.02 and bounds[2]-.02<=p.y<=bounds[3]+.02) for p in (a,b)):
        raise PlanningError('roads_invalid','道路端点不在其几何边界内')
    return tuple(bounds)


def _edge_factor(edge):
    kind=edge.get('kind','road')
    if kind not in ('road','connector'):raise PlanningError('roads_invalid','道路连接类型无效')
    expected=2 if kind=='connector' else 1
    value=edge.get('cost_factor',expected)
    if isinstance(value,bool) or not isinstance(value,(int,float)) or value!=expected:
        raise PlanningError('roads_invalid','道路连接代价无效')
    return value


def _edge_length(edge,reverse=False):
    value=edge.get('reverse_length',edge['length']) if reverse else edge['length']
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
        raise PlanningError('roads_invalid','道路方向长度无效')
    return value


def _partial_length(edge,start,end,total):
    ratio=_edge_length(edge,True)/total if start>end and total else 1.
    return abs(end-start)*ratio


def _height_at(a,b,t):
    if t<=1e-12:return a.z
    if t>=1.-1e-12:return b.z
    return a.z+(b.z-a.z)*t if a.z is not None and b.z is not None else None


def projection(point, points):
    best=None;along=0.
    for i,(a,b) in enumerate(zip(points,points[1:])):
        dx,dy=b.x-a.x,b.y-a.y;norm=dx*dx+dy*dy
        t=max(0.,min(1.,((point.x-a.x)*dx+(point.y-a.y)*dy)/norm)) if norm else 0.
        q=Point(a.c,a.x+dx*t,a.y+dy*t,_height_at(a,b,t))
        candidate=(point.distance(q),along+a.distance(b)*t,i,q)
        if best is None or candidate[:2]<best[:2]:best=candidate
        along+=a.distance(b)
    return best


def slice_line(points, start, end):
    reverse=start>end
    if reverse:start,end=end,start
    out=[];along=0.
    for a,b in zip(points,points[1:]):
        d=a.distance(b)
        if along+d>=start and along<=end:
            for x in (max(start,along),min(end,along+d)):
                t=(x-along)/d if d else 0.
                p=Point(a.c,a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,_height_at(a,b,t))
                if not out or out[-1]!=p:out.append(p)
        along+=d
    if reverse:out.reverse()
    return out


class RoadNetwork:
    def __init__(self, manifest, pack):
        self.root=manifest.parent.resolve();data=manifest.read_bytes()
        self.raw=json.loads(data)
        if self.raw.get('kind')=='adrive-road-pointer':
            target=(self.root/self.raw['path']).resolve()
            if not target.is_relative_to(self.root):raise PlanningError('roads_invalid','道路清单路径越界')
            data=target.read_bytes()
            if hashlib.sha256(data).hexdigest()!=self.raw['sha256']:raise PlanningError('roads_integrity','道路清单校验失败')
            self.root=target.parent;self.raw=json.loads(data)
        if self.raw.get('schema')!=1 or self.raw.get('kind')!='adrive-unified-roads':
            raise PlanningError('roads_invalid','统一路网格式不兼容')
        if (self.raw['navigation_identity']!=pack.identity or self.raw['build']!=pack.build
                or self.raw['native_library_sha256']!=pack.raw['library']['sha256']):
            raise PlanningError('roads_mismatch','道路包与碰撞导航或查询库不匹配')
        if pack.native.corridor_path is None:
            raise PlanningError('navmesh_capability','当前查询库缺少道路走廊接口')
        self.identity=self.raw['identity'];self.nodes=[Point(*p) for p in self.raw['nodes']]
        self.edges=self.raw['edges'];self.adj=defaultdict(list);self.geometry=OrderedDict();self.by_tile=defaultdict(list)
        self._geometry_sizes={};self._geometry_bytes=0;self._geometry_limit=16*1024*1024
        self._validation_cache=OrderedDict();self._validation_sizes={};self._validation_bytes=0;self._validation_limit=8*1024*1024
        self._edge_boxes=[];entry_groups=defaultdict(list)
        for i,e in enumerate(self.edges):
            if not (0<=e['a']<len(self.nodes) and 0<=e['b']<len(self.nodes)) or not math.isfinite(e['length']) or e['length']<=0:
                raise PlanningError('roads_invalid','道路索引无效')
            if self.nodes[e['a']].c!=self.nodes[e['b']].c:raise PlanningError('roads_invalid','道路跨大陆')
            _edge_factor(e)
            _edge_length(e,True)
            self._edge_boxes.append(_edge_box(e,self.nodes[e['a']],self.nodes[e['b']]))
            self.adj[e['a']].append((e['b'],i,False));self.adj[e['b']].append((e['a'],i,True))
            self.by_tile[e['shard']].append(i)
            if e.get('kind','road')=='road':entry_groups[e['shard']].append(i)
        # A tile-level spatial index lets endpoint access expand outwards
        # without imposing a radius or putting every road in the query heap.
        # The bounds cover the whole polyline, including bends far from the
        # endpoint chord. Explicit connectors cannot become endpoint entries.
        self.entry_tiles=[]
        for indices in entry_groups.values():
            boxes=[self._edge_boxes[i] for i in indices]
            self.entry_tiles.append((self.nodes[self.edges[indices[0]]['a']].c,
                (min(b[0] for b in boxes),max(b[1] for b in boxes),
                 min(b[2] for b in boxes),max(b[3] for b in boxes)),indices))
        self.components={}
        for u in range(len(self.nodes)):
            if u in self.components:continue
            todo=[u];self.components[u]=u
            while todo:
                v=todo.pop()
                for w,_,_ in self.adj[v]:
                    if w not in self.components:self.components[w]=u;todo.append(w)
    def entry_candidates(self,p,check,blocked_edges=frozenset()):
        """Globally ordered lower bounds, lazily expanding nearby tile batches."""
        def lower(box):
            ax,bx,ay,by=box
            return math.hypot(max(ax-p.x,p.x-bx,0),max(ay-p.y,p.y-by,0))
        tiles=[(lower(box),k,indices) for k,(c,box,indices) in enumerate(self.entry_tiles) if c==p.c]
        heapq.heapify(tiles);pending=[]
        while tiles or pending:
            check()
            # Equal bounds must be expanded first so chord/index tie breaks
            # stay deterministic across shard boundaries.
            if tiles and (not pending or tiles[0][0]<=pending[0][0]):
                _,_,indices=heapq.heappop(tiles)
                for i in indices:
                    check()
                    e=self.edges[i]
                    if e['id'] in blocked_edges or e.get('kind','road')!='road':continue
                    a,b=self.nodes[e['a']],self.nodes[e['b']]
                    # A large curved road's box may contain the endpoint even
                    # while all of its pavement is far away. Refine only this
                    # spatial batch to a true access-length lower bound before
                    # spending a native query on it.
                    distance=projection(p,self.points(i))[0]
                    heapq.heappush(pending,(distance,projection(p,[a,b])[0],i))
            else:
                yield heapq.heappop(pending)
    def _geometry_record(self,i):
        edge=self.edges[i];key=edge['shard']
        if key not in self.geometry:
            entry=self.raw['shards'][key];data=self._read_entry(entry,self._geometry_limit,'道路分片')
            self.geometry[key]={r['id']:r for r in json.loads(data)}
            self._geometry_sizes[key]=len(data);self._geometry_bytes+=len(data)
            while len(self.geometry)>32 or self._geometry_bytes>self._geometry_limit:
                old,_=self.geometry.popitem(last=False);self._geometry_bytes-=self._geometry_sizes.pop(old)
        else:self.geometry.move_to_end(key)
        return self.geometry[key][edge['id']]
    def points(self,i,reverse=False):
        edge=self.edges[i];key=edge['shard'];record=self._geometry_record(i)
        c=self.nodes[edge['a']].c
        independent_reverse=reverse and record.get('reverse_points') is not None
        values=record['reverse_points'] if independent_reverse else record['points']
        pts=[Point(c,*p) for p in values]
        a,b=self.nodes[edge['a']],self.nodes[edge['b']]
        pts[0],pts[-1]=(b,a) if independent_reverse else (a,b)
        if self.raw.get('validation'):
            row=self.validation(key).get(edge['id'])
            if not row or row['entry']!=[a.c,a.x,a.y,a.z] or row['exit']!=[b.c,b.x,b.y,b.z]:
                raise PlanningError('roads_invalid','道路楼层锚点与几何不匹配')
            if any(row[k]['status']!='passed' for k in ('forward','reverse')):
                raise PlanningError('roads_invalid','道路未完成双向碰撞验证')
        return list(reversed(pts)) if reverse and not independent_reverse else pts
    def reference(self,i,start=None,end=None):
        """The unchanged source corridor, cropped only for an actual partial leg."""
        edge=self.edges[i];record=self._geometry_record(i);c=self.nodes[edge['a']].c
        reference=[Point(c,*p) for p in record['reference']] if record.get('reference') is not None else self.points(i)
        if len(reference)<2:raise PlanningError('roads_invalid','道路共同参考走廊不完整')
        if start is None and end is None:return reference
        a,b=self.nodes[edge['a']],self.nodes[edge['b']]
        start=a if start is None else start;end=b if end is None else end
        def same(p,q):
            return p.distance(q)<=.001 and (p.z is None or q.z is None or abs(p.z-q.z)<.05)
        if same(start,a) and same(end,b):return reference
        if same(start,b) and same(end,a):return list(reversed(reference))
        left=projection(start,reference)[1];right=projection(end,reference)[1]
        result=slice_line(reference,left,right)
        # A zero-width along interval still defines the original local ribbon
        # around this point. Never recenter it on a derived avoidance path.
        if len(result)==1:return [result[0],result[0]]
        return result
    def _read_entry(self,entry,limit,label):
        if not isinstance(entry['bytes'],int) or not 0<=entry['bytes']<=limit:
            raise PlanningError('budget',label+'超过有界分片缓存预算')
        path=(self.root/entry['path']).resolve()
        if not path.is_relative_to(self.root):raise PlanningError('roads_invalid',label+'路径越界')
        try:
            with path.open('rb') as stream:data=stream.read(limit+1)
        except OSError as exc:raise PlanningError('roads_missing',label+'缺失') from exc
        if len(data)!=entry['bytes'] or hashlib.sha256(data).hexdigest()!=entry['sha256']:
            raise PlanningError('roads_integrity',label+'完整性校验失败')
        return data
    def validation(self,key):
        binding=self.raw.get('validation')
        if not binding or binding.get('kind')!='native-road-corridor-v1':
            raise PlanningError('roads_invalid','道路包缺少逐条碰撞验证记录')
        if (binding['navigation_identity']!=self.raw['navigation_identity']
                or binding['native_library_sha256']!=self.raw['native_library_sha256']
                or binding['edge_count']!=len(self.edges)):
            raise PlanningError('roads_mismatch','道路碰撞验证记录与路网不匹配')
        if key not in self._validation_cache:
            entry=binding['shards'][key];data=self._read_entry(entry,self._validation_limit,'道路碰撞验证记录')
            self._validation_cache[key]={r['id']:r for r in json.loads(data)}
            self._validation_sizes[key]=len(data);self._validation_bytes+=len(data)
            while len(self._validation_cache)>8 or self._validation_bytes>self._validation_limit:
                old,_=self._validation_cache.popitem(last=False);self._validation_bytes-=self._validation_sizes.pop(old)
        else:self._validation_cache.move_to_end(key)
        return self._validation_cache[key]


def load_network(asset_root, pack):
    if asset_root is None:raise PlanningError('roads_missing','缺少经过验证的统一道路包')
    path=Path(asset_root)/'roads/manifest.json'
    if not path.exists():raise PlanningError('roads_missing','缺少经过验证的统一道路包')
    try:
        network=RoadNetwork(path,pack)
        if not network.raw.get('validation'):raise PlanningError('roads_invalid','统一道路包缺少碰撞验证记录')
        return network
    except PlanningError:raise
    except (OSError,ValueError,KeyError,TypeError) as exc:
        raise PlanningError('roads_invalid',f'统一道路包不可用：{exc}') from exc


class RoadPlanner:
    def __init__(self,pack,network):self.pack,self.network=pack,network
    def plan(self,start,goal,blocked_edges=frozenset(),cancel=None,max_expansions=500000):
        from .navmesh import validate_point
        validate_point(start);validate_point(goal)
        if start.c!=goal.c:raise PlanningError('different_continent','道路导航不能跨大陆')
        network=self.network;spent=0;mesh=Mesh(self.pack,start.c,cancel)
        trace=[];diagnostic=dict(schema=1,planner_version='unified-roads-v1',navigation_identity=self.pack.identity,
            native_library_sha256=self.pack.raw['library']['sha256'],road_identity=network.identity,
            start=vars(start),goal=vars(goal),blocked_edges=sorted(blocked_edges),max_expansions=max_expansions,
            road_trace=trace,complete=True,guide=None)
        self.pack.last_route_diagnostics=diagnostic
        def check():
            if cancel and cancel.is_set():raise PlanningError('cancelled','规划已取消')
            if spent>=max_expansions:raise PlanningError('budget','道路导航搜索预算已用尽')
        def load(points):
            keys=set()
            for a,b in zip(points,points[1:]):
                steps=max(1,math.ceil(a.distance(b)/200))
                for i in range(steps+1):keys.update(self.pack.around(Point(a.c,a.x+(b.x-a.x)*i/steps,a.y+(b.y-a.y)*i/steps)))
            if len(points)==1:keys=self.pack.around(points[0])
            mesh.load(keys);mesh.block(blocked_edges)
        def query(a,b,reference=None,width=24.,limit=None):
            nonlocal spent
            check();load(reference if reference is not None else [a,b])
            try:
                parts,count=mesh.path(a,b,min(max_expansions-spent,limit) if limit else max_expansions-spent,road_reference=reference,corridor_width=width,
                                     max_length_xy=None if reference else max(a.distance(b)*4+50,50))
                spent+=count
                if not parts and a.distance(b)>1e-5:
                    raise PlanningError('navmesh_endpoint','碰撞查询未输出完整连接')
                return tuple(parts)
            except PlanningError as exc:
                spent+=getattr(exc,'expanded',0);raise
        def resolve(p,label):
            load([p]);hs=mesh.heights(p)
            if not hs:raise PlanningError('navmesh_outside',label+'不在可步行表面内')
            if p.z is None and label=='起点' and len(hs)!=1:
                raise PlanningError('navmesh_height','起点楼层不明，请先确定角色楼层')
            if p.z is not None:
                hs=sorted(hs,key=lambda z:abs(z-p.z))
                if abs(hs[0]-p.z)>2.5:
                    raise PlanningError('navmesh_height',label+'的确认高度与当前位置碰撞地面不匹配')
                if len(hs)>1 and abs(abs(hs[0]-p.z)-abs(hs[1]-p.z))<.35:
                    raise PlanningError('navmesh_height',label+'高度无法区分重叠楼层')
                hs=hs[:1]
            return [Point(p.c,p.x,p.y,h) for h in hs[:8]]
        def entries(p,allowance=131072,towards_endpoint=False):
            # Sort lower bounds without a radius cutoff; exact geometry and
            # complete native access are checked incrementally, at most 8 pass.
            queue=network.entry_candidates(p,check,blocked_edges)
            result=[];seen=set();represented=set();rejected=defaultdict(int);began=spent
            for lower,_,i in queue:
                check()
                if len(result)>=8:break
                if spent-began>=allowance:break
                if result and lower*2>min(e['cost'] for e in result)+48:break
                e=network.edges[i];pts=network.points(i)
                _,along,at,q=projection(p,pts)
                tangent=pts[at+1].x-pts[at].x,pts[at+1].y-pts[at].y
                direction=round((math.atan2(tangent[1],tangent[0])%math.pi)/(math.pi/4))%4
                sources=e.get('sources') or (e['id'],)
                # Whole connected components can contain many parallel roads
                # with different onward connections. Coalesce only pieces of
                # the same original road, direction and local height band.
                family=(sources[0],direction,round(q.z/3.))
                if family in represented or rejected[family]>=3:continue
                signature=(round(q.x,1),round(q.y,1),round(q.z,1))
                if signature in seen:continue
                seen.add(signature)
                try:
                    a,b=(q,p) if towards_endpoint else (p,q)
                    access=query(a,b,limit=min(allowance-(spent-began),65535,max(8192,int(p.distance(q)*32))))
                    result.append(dict(edge=i,along=along,point=q,parts=access,cost=sum(s.distance for s in access)*2,points=pts))
                    represented.add(family)
                except PlanningError as exc:
                    if exc.code not in ('no_route','road_scope_unconnected','blocked_navmesh','navmesh_outside','navmesh_height','navmesh_endpoint','budget'):raise
                    rejected[family]+=1
            # No absolute distance cutoff: a truly remote endpoint may have a
            # long necessary approach. Once its nearest verified access exists,
            # other entries may only add local (24 yd) exposure. Otherwise an
            # endpoint already on a road could disguise a cross-country bypass
            # as a long "access" leg to a road beside the destination.
            if result:
                best=min(e['cost'] for e in result)
                result=[e for e in result if e['cost']<=best+48.]
            return result
        cache={};excluded=set(blocked_edges)
        try:
            check();origin=resolve(start,'起点')[0];goals=resolve(goal,'终点')
            if origin.distance(goal)<1e-7 and any(abs(origin.z-p.z)<.35 for p in goals):
                route=Route(origin,origin,(),0)
            else:
                starts=entries(origin);ends=[]
                for target in goals:
                    for item in entries(target,allowance=min(65535,max(1,(max_expansions-spent)//len(goals))),towards_endpoint=True):item['target']=target;ends.append(item)
                ends=sorted(ends,key=lambda e:e['cost'])[:8]
                if not starts or not ends:raise PlanningError('road_access_unverified','限定预算内未验证起终点接路；未改走野外')
                # Each graph traversal and each failed native query shares one
                # ledger. Alternatives exclude the failed road, never go wild.
                while True:
                    check();serial=itertools.count();heap=[];scores={-1:0};came={};links=defaultdict(list)
                    def add(u,v,cost,payload):links[u].append((v,cost,payload))
                    for s in starts:
                        e=network.edges[s['edge']]
                        if e['id'] in excluded:continue
                        for v,pos in ((e['a'],0.),(e['b'],length(s['points']))):
                            add(-1,v,s['cost']+_partial_length(e,s['along'],pos,length(s['points'])),('start',s,pos))
                    for t in ends:
                        e=network.edges[t['edge']]
                        if e['id'] in excluded:continue
                        for u,pos in ((e['a'],0.),(e['b'],length(t['points']))):
                            add(u,-2,t['cost']+_partial_length(e,pos,t['along'],length(t['points'])),('end',t,pos))
                    for s in starts:
                        for t in ends:
                            if s['edge']==t['edge'] and network.edges[s['edge']]['id'] not in excluded:
                                e=network.edges[s['edge']]
                                add(-1,-2,s['cost']+_partial_length(e,s['along'],t['along'],length(s['points']))+t['cost'],('same',s,t))
                    heapq.heappush(heap,(0,next(serial),-1))
                    while heap:
                        score,_,u=heapq.heappop(heap)
                        if score!=scores.get(u):continue
                        if u==-2:break
                        check();spent+=1
                        options=list(links[u])
                        for v,i,reverse in network.adj.get(u,()):
                            e=network.edges[i]
                            if e['id'] not in excluded:options.append((v,_edge_length(e,reverse)*_edge_factor(e),('road',i,reverse)))
                        for v,cost,payload in options:
                            new=score+cost
                            if new<scores.get(v,math.inf):
                                scores[v]=new;came[v]=(u,payload);heapq.heappush(heap,(new,next(serial),v))
                    if -2 not in came:raise PlanningError('road_connection_unverified','已验证道路不连通；未采用野外备用路线')
                    chain=[];u=-2
                    while u!=-1:u,payload=came[u];chain.append(payload)
                    chain.reverse();output=[];selected_goal=None;failed=None
                    guide=[];access_cost=0.;connector_length=0.;connector_extra=0.;connector_count=0
                    for payload in chain:
                        if payload[0]=='road':
                            _,i,reverse=payload;e=network.edges[i];edge_kind=e.get('kind','road')
                            edge_length=_edge_length(e,reverse)
                            guide.append(dict(edge=e['id'],kind=edge_kind,reverse=reverse,length_xy=edge_length))
                            if edge_kind=='connector':
                                connector_count+=1;connector_length+=edge_length;connector_extra+=edge_length*(_edge_factor(e)-1)
                        elif payload[0] in ('start','end'):
                            kind,item,pos=payload;access_cost+=item['cost'];guide.append(dict(kind=kind,edge_kind='road',edge=network.edges[item['edge']]['id'],
                                along=item['along'],node_along=pos,access_point=vars(item['point'])))
                        else:
                            _,s,t=payload;access_cost+=s['cost']+t['cost'];guide.append(dict(kind='same',edge_kind='road',edge=network.edges[s['edge']]['id'],
                                start_along=s['along'],goal_along=t['along']))
                    diagnostic['guide']=dict(road_identity=network.identity,links=guide,weighted_cost=scores[-2],
                        length_xy=scores[-2]-access_cost/2-connector_extra,endpoint_access_xy=access_cost/2,
                        connector_count=connector_count,connector_length_xy=connector_length,
                        connector_weighted_extra_xy=connector_extra,
                        road_length_xy=scores[-2]-access_cost-connector_extra-connector_length)
                    def road(i,pts):
                        nonlocal failed
                        if len(pts)<2 or length(pts)<1e-6:return ()
                        key=(i,tuple(pts));e=network.edges[i]
                        if key in cache:return cache[key]
                        if e.get('kind','road')=='connector':
                            try:
                                parts=query(pts[0],pts[-1]);cache[key]=parts;return parts
                            except PlanningError as exc:
                                if exc.code not in ('no_route','road_scope_unconnected','blocked_navmesh'):raise
                                trace.append(dict(mode='rejected_connector',edge=e['id'],code=exc.code,
                                    message=str(exc),expanded_total=spent))
                                failed=e['id'];return ()
                        reference=network.reference(i,pts[0],pts[-1]) if hasattr(network,'reference') else pts
                        for width in (3.,9.,24.):
                            try:
                                parts=query(pts[0],pts[-1],reference,width);cache[key]=parts;return parts
                            except PlanningError as exc:
                                if exc.code not in ('no_route','road_scope_unconnected','blocked_navmesh'):raise
                                trace.append(dict(mode='rejected_road_corridor',edge=e['id'],width=width,
                                    kind=e.get('kind','road'),code=exc.code,message=str(exc),expanded_total=spent))
                        # A derived avoidance path can project to a source
                        # sub-arc that is too short at a bend. The last retry
                        # stays inside this same immutable source road's full
                        # 24 yd corridor, never an unrestricted remainder.
                        if hasattr(network,'reference'):
                            complete=network.reference(i)
                            if reference!=complete and reference!=list(reversed(complete)):
                                try:
                                    parts=query(pts[0],pts[-1],complete,24.)
                                    trace.append(dict(mode='accepted_full_source_corridor',edge=e['id'],width=24.,expanded_total=spent))
                                    cache[key]=parts;return parts
                                except PlanningError as exc:
                                    if exc.code not in ('no_route','road_scope_unconnected','blocked_navmesh'):raise
                                    trace.append(dict(mode='rejected_full_source_corridor',edge=e['id'],width=24.,
                                        code=exc.code,message=str(exc),expanded_total=spent))
                        failed=e['id'];return ()
                    for payload in chain:
                        kind=payload[0]
                        if kind=='road':
                            _,i,reverse=payload;output.extend(road(i,network.points(i,reverse)))
                        elif kind=='start':
                            _,s,pos=payload;output.extend(s['parts']);output.extend(road(s['edge'],slice_line(s['points'],s['along'],pos)))
                        elif kind=='end':
                            _,t,pos=payload;output.extend(road(t['edge'],slice_line(t['points'],pos,t['along'])))
                            output.extend(t['parts']);selected_goal=t['target']
                        else:
                            _,s,t=payload;output.extend(s['parts']);output.extend(road(s['edge'],slice_line(s['points'],s['along'],t['along'])))
                            output.extend(t['parts']);selected_goal=t['target']
                        if failed:break
                    if failed:
                        excluded.add(failed);trace.append(dict(mode='exclude_failed_road',edge=failed,expanded_total=spent));continue
                    route=Route(origin,selected_goal,tuple(output),spent);break
            if cancel and cancel.is_set():raise PlanningError('cancelled','规划已取消')
            reference_length=(diagnostic.get('guide') or {}).get('length_xy',0.)
            diagnostic['route_selection']=dict(selected='road_only',road_identity=network.identity,expanded_total=spent,
                guide_length_xy=reference_length,route_guide_ratio=route.distance/reference_length if reference_length else 1.,
                connector_count=(diagnostic.get('guide') or {}).get('connector_count',0),
                connector_length_xy=(diagnostic.get('guide') or {}).get('connector_length_xy',0.))
            diagnostic['result']=dict(status='success',start=vars(route.start),goal=vars(route.goal),length_xy=route.distance,
                expanded=spent,segments=[dict(points=[vars(p) for p in s.points],source=s.source,reliability=s.reliability,edge_id=s.edge_id) for s in route.segments])
            return route
        except PlanningError as exc:
            exc.expanded=spent;diagnostic['result']=dict(status='error',code=exc.code,message=str(exc),expanded=spent);raise
        finally:
            mesh.close()
            if self.pack.record_diagnostics:self.pack._save_route_diagnostics(diagnostic)
