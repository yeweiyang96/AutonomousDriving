"""Conservative whole-road overlap matching against an existing source graph.

This module proposes correspondences, not walk permissions. It never changes
source geometry or joins nodes by position. The caller must collision-validate
the returned entry/exit connections before suppressing or replacing a road.
"""
from __future__ import annotations

from collections import defaultdict
import heapq
import itertools
import math

from .models import Point

EPS=1e-8


def _length(points):
    return sum(a.distance(b) for a,b in zip(points,points[1:]))


def _interpolate(a,b,t):
    z=a.z+(b.z-a.z)*t if a.z is not None and b.z is not None else None
    return Point(a.c,a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,z)


def _samples(points,spacing=4.):
    if not points:return
    yield points[0]
    for a,b in zip(points,points[1:]):
        n=max(1,math.ceil(a.distance(b)/spacing))
        for k in range(1,n+1):yield _interpolate(a,b,k/n)


def _slice(points,start,end):
    reverse=start>end
    if reverse:start,end=end,start
    out=[];along=0.
    for a,b in zip(points,points[1:]):
        d=a.distance(b)
        if d>EPS and along+d>=start-EPS and along<=end+EPS:
            for pos in (max(start,along),min(end,along+d)):
                q=_interpolate(a,b,max(0.,min(1.,(pos-along)/d)))
                if not out or out[-1]!=q:out.append(q)
        along+=d
    if reverse:out.reverse()
    return out


def _tangent(points,at_end=False):
    pairs=list(zip(points,points[1:]))
    if at_end:pairs.reverse()
    for a,b in pairs:
        d=a.distance(b)
        if d>EPS:return ((b.x-a.x)/d,(b.y-a.y)/d)
    return None


class _Polyline:
    """Cached segment offsets, with a local index for long curved roads."""
    def __init__(self,points):
        self.points=points;self.segments=[];self.length=0.;self.grid=None
        for a,b in zip(points,points[1:]):
            d=a.distance(b)
            if d>EPS:self.segments.append((a,b,self.length,d))
            self.length+=d
        if len(self.segments)>24:
            self.grid=defaultdict(set)
            for i,(a,b,_,d) in enumerate(self.segments):
                n=max(1,math.ceil(d/8))
                for k in range(n+1):
                    x,y=a.x+(b.x-a.x)*k/n,a.y+(b.y-a.y)*k/n
                    bx,by=math.floor(x/16),math.floor(y/16)
                    for dx in (-1,0,1):
                        for dy in (-1,0,1):self.grid[(bx+dx,by+dy)].add(i)

    def nearest(self,p,limit=12.,tangent=None):
        best=None
        indices=self.grid.get((math.floor(p.x/16),math.floor(p.y/16)),()) if self.grid is not None else range(len(self.segments))
        for i in indices:
            a,b,along,d=self.segments[i]
            if p.c!=a.c:continue
            ux,uy=(b.x-a.x)/d,(b.y-a.y)/d
            if tangent is not None and abs(ux*tangent[0]+uy*tangent[1])<.7:continue
            t=max(0.,min(1.,((p.x-a.x)*ux+(p.y-a.y)*uy)/d));q=_interpolate(a,b,t)
            if p.z is not None and q.z is not None and abs(p.z-q.z)>2.5:continue
            distance=p.distance(q)
            if distance>limit+EPS:continue
            item=(distance,along+d*t,q)
            if best is None or item[:2]<best[:2]:best=item
        return best


class OverlapIndex:
    """Match one complete road to a connected chain in a preferred road graph.

    Graph adjacency comes exclusively from SourceRoad.a/b. A road with absent
    endpoint IDs is isolated from other roads, even if their coordinates touch.
    ``find`` returns a proposal dictionary or None; ``last_stats`` explains a
    conservative rejection or exhausted matching budget.
    """
    def __init__(self,preferred_roads,*,corridor=8.,endpoint_radius=12.,cell_size=64.,
                 max_expansions=4096,max_candidates=4096):
        if not 0<corridor<=endpoint_radius<=12 or cell_size<=0:
            raise ValueError('Invalid overlap geometry limits')
        self.roads=list(preferred_roads);self.corridor=float(corridor)
        self.endpoint_radius=float(endpoint_radius);self.cell_size=float(cell_size)
        self.max_expansions=max_expansions;self.max_candidates=max_candidates
        self.lines=[_Polyline(r.points) for r in self.roads];self.grid=defaultdict(set)
        self.node_ids=[];self.last_stats={}
        for i,(road,line) in enumerate(zip(self.roads,self.lines)):
            a=getattr(road,'a',None);b=getattr(road,'b',None)
            self.node_ids.append((a if a is not None else ('private',i,'a'),b if b is not None else ('private',i,'b')))
            radius=math.ceil(endpoint_radius/cell_size)
            for p in _samples(road.points,cell_size/2):
                bx,by=math.floor(p.x/cell_size),math.floor(p.y/cell_size)
                for dx in range(-radius,radius+1):
                    for dy in range(-radius,radius+1):self.grid[(p.c,bx+dx,by+dy)].add(i)

    def find(self,road):
        self.last_stats=dict(candidate_roads=0,expanded=0,reason='no_match')
        if len(road.points)<2:return None
        candidate=_Polyline(road.points)
        if candidate.length<=EPS:return None
        if self.max_expansions<1:self.last_stats['reason']='budget';return None
        local=set()
        for p in _samples(road.points,min(8.,self.cell_size/2)):
            local.update(self.grid.get((p.c,math.floor(p.x/self.cell_size),math.floor(p.y/self.cell_size)),()))
            if len(local)>self.max_candidates:
                self.last_stats.update(candidate_roads=len(local),reason='candidate_budget');return None
        self.last_stats['candidate_roads']=len(local)
        ends=[]
        for p,tangent in ((road.points[0],_tangent(road.points)),(road.points[-1],_tangent(road.points,True))):
            matches=[]
            for i in local:
                projection=self.lines[i].nearest(p,self.endpoint_radius,tangent)
                if projection is not None:
                    distance,along,q=projection
                    matches.append(dict(edge_id=self.roads[i].id,along=along,point=q,distance=distance,index=i))
            ends.append(sorted(matches,key=lambda m:(m['distance'],m['edge_id'],m['along']))[:8])
        if not ends[0] or not ends[1]:self.last_stats['reason']='missing_endpoint';return None

        allowed={}
        def interval(i,start,end):
            key=(i,start,end)
            if key not in allowed:
                points=_slice(self.roads[i].points,start,end)
                if len(points)<2 or abs(start-end)<=EPS:allowed[key]=()
                else:
                    good=all(candidate.nearest(p,self.corridor) is not None for p in _samples(points))
                    allowed[key]=points if good else None
            return allowed[key]

        start_node=('overlap_virtual','start');goal_node=('overlap_virtual','goal')
        graph=defaultdict(list)
        def add(a,b,i,begin,end,*,entry=None,exit=None):
            pts=interval(i,begin,end)
            if pts is None:return
            payload=dict(piece=(dict(edge_id=self.roads[i].id,from_along=begin,to_along=end) if pts else None),
                points=pts,entry=entry,exit=exit)
            penalty=(entry['distance'] if entry else 0)+(exit['distance'] if exit else 0)
            graph[a].append((b,abs(end-begin)+penalty,payload))

        for i in local:
            a,b=self.node_ids[i];total=self.lines[i].length
            add(a,b,i,0.,total);add(b,a,i,total,0.)
        for match in ends[0]:
            i=match['index'];a,b=self.node_ids[i]
            add(start_node,a,i,match['along'],0.,entry=match)
            add(start_node,b,i,match['along'],self.lines[i].length,entry=match)
        for match in ends[1]:
            i=match['index'];a,b=self.node_ids[i]
            add(a,goal_node,i,0.,match['along'],exit=match)
            add(b,goal_node,i,self.lines[i].length,match['along'],exit=match)
        for a in ends[0]:
            for b in ends[1]:
                if a['index']==b['index']:add(start_node,goal_node,a['index'],a['along'],b['along'],entry=a,exit=b)

        serial=itertools.count();queue=[(0.,next(serial),start_node)];score={start_node:0.};came={}
        while queue:
            cost,_,node=heapq.heappop(queue)
            if cost!=score.get(node):continue
            if node==goal_node:break
            if self.last_stats['expanded']>=self.max_expansions:
                self.last_stats['reason']='budget';return None
            self.last_stats['expanded']+=1
            for target,extra,payload in graph[node]:
                new=cost+extra
                if new<score.get(target,math.inf):
                    score[target]=new;came[target]=(node,payload);heapq.heappush(queue,(new,next(serial),target))
        if goal_node not in came:self.last_stats['reason']='disconnected';return None
        chain=[];node=goal_node
        while node!=start_node:node,payload=came[node];chain.append(payload)
        chain.reverse();points=[];entry=exit=None;pieces=[]
        for payload in chain:
            if payload['entry'] is not None:entry=payload['entry']
            if payload['exit'] is not None:exit=payload['exit']
            if payload['piece'] is not None:pieces.append(payload['piece'])
            if points and payload['points']:
                a,b=points[-1],payload['points'][0]
                if a.distance(b)>1e-3 or (a.z is not None and b.z is not None and abs(a.z-b.z)>2.5):
                    self.last_stats['reason']='junction_geometry_mismatch';return None
            for p in payload['points']:
                if not points or points[-1]!=p:points.append(p)
        if not entry or not exit or len(points)<2:self.last_stats['reason']='empty_interval';return None
        existing=_Polyline(points)
        forward=[];reverse=[]
        sample_list=list(_samples(road.points))
        for k,p in enumerate(sample_list):
            limit=self.endpoint_radius if k in (0,len(sample_list)-1) else self.corridor
            match=existing.nearest(p,limit)
            if match is None:self.last_stats['reason']='incomplete_coverage';return None
            forward.append(match[0])
        for p in _samples(points):
            match=candidate.nearest(p,self.corridor)
            if match is None:self.last_stats['reason']='incomplete_coverage';return None
            reverse.append(match[0])
        coverage_forward=sum(d<=self.corridor+EPS for d in forward)/len(forward)
        coverage_reverse=sum(d<=self.corridor+EPS for d in reverse)/len(reverse)
        if min(coverage_forward,coverage_reverse)<.9:self.last_stats['reason']='low_coverage';return None
        for a,b in ((_tangent(road.points),_tangent(points)),(_tangent(road.points,True),_tangent(points,True))):
            if a is None or b is None or a[0]*b[0]+a[1]*b[1]<.2:
                self.last_stats['reason']='direction';return None
        connection_length=entry['distance']+exit['distance']
        total=candidate.length+connection_length
        self.last_stats['reason']='matched'
        clean=lambda m:{k:v for k,v in m.items() if k!='index'}
        return dict(entry=clean(entry),exit=clean(exit),path=pieces,existing_points=points,
            existing_length=existing.length,candidate_length=candidate.length,
            candidate_connection_length=connection_length,candidate_total_length=total,
            coverage_forward=coverage_forward,coverage_reverse=coverage_reverse,
            max_distance=max(forward+reverse),rms_distance=math.sqrt(sum(d*d for d in forward+reverse)/(len(forward)+len(reverse))),
            keep_existing=total>existing.length*.95+EPS,expanded=self.last_stats['expanded'],
            candidate_roads=len(local),collision_verified=False)


def find_overlap(road,preferred_roads,index=None):
    """Convenience wrapper; reuse an OverlapIndex for bulk source imports."""
    return (index if index is not None else OverlapIndex(preferred_roads)).find(road)
