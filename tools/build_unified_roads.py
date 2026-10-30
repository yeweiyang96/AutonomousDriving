"""Build a candidate road pack. Never writes the active road/navigation manifest."""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from adrive.data import DataBundle
from adrive.models import Point, PlanningError
from adrive.navmesh import NavMeshPack, Mesh
from adrive.road_sources import source_roads


def encode(value):
    return (json.dumps(value,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n').encode('utf-8')


def sample(points, spacing=8.):
    out=[points[0]]
    for a,b in zip(points,points[1:]):
        for k in range(1,max(1,math.ceil(a.distance(b)/spacing))+1):
            n=max(1,math.ceil(a.distance(b)/spacing));t=k/n
            p=Point(a.c,a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,
                    a.z+(b.z-a.z)*t if a.z is not None and b.z is not None else None)
            if out[-1].distance(p)>.001:out.append(p)
    return out


def length_xy(points):
    return sum(a.distance(b) for a,b in zip(points,points[1:]))


class Validator:
    def __init__(self, pack):
        self.pack=pack;self.mesh=None;self.c=None;self.projections={}
        original=pack.read
        cached=lru_cache(maxsize=64)(lambda path: original(self.entries[path]))
        self.entries={e['path']:e for e in [*pack.raw['maps'],*pack.raw['tiles']]}
        pack.read=lambda entry: cached(entry['path'])
    def at(self, points):
        c=points[0].c;keys=set()
        for p in points:keys.update(self.pack.around(p))
        missing=keys-(self.mesh.loaded if self.mesh and self.c==c else set())
        size=sum(self.pack.tiles.get((c,*k),{}).get('bytes',0) for k in missing)
        if self.mesh is None or self.c!=c or self.mesh.bytes+size>128*1024*1024:
            self.close();self.mesh=Mesh(self.pack,c);self.c=c
        self.mesh.load(keys);return self.mesh
    def close(self):
        if self.mesh:self.mesh.close()
        self.mesh=None
    def project(self,p):
        key=(p.c,p.x,p.y,p.z)
        if key in self.projections:return self.projections[key]
        mesh=self.at([p]);hs=mesh.heights(p)
        if p.z is not None and hs:
            hs.sort(key=lambda h:abs(h-p.z))
            if len(hs)==1 or abs(hs[1]-p.z)-abs(hs[0]-p.z)>.35:hs=hs[:1]
        if len(hs)==1:result=Point(p.c,p.x,p.y,hs[0])
        elif hs:raise PlanningError('floor_unverified','Source road has multiple surfaces')
        else:
            options=mesh.project_road(p)
            if not options:raise PlanningError('surface_missing','No source surface within 9 yd')
            best=min(p.distance(q) for q in options)
            options=[q for q in options if p.distance(q)<=best+.02]
            if len(options)!=1:raise PlanningError('floor_unverified','Projected source has multiple surfaces')
            result=options[0]
        self.projections[key]=result;return result
    def path(self,a,b,reference,budget=4096):
        mesh=self.at(reference);spent=0;last=None
        for width in (3.,9.,24.):
            try:
                parts,count=mesh.path(a,b,budget-spent,road_reference=reference,corridor_width=width)
                spent+=count
                if not parts and a.distance(b)>1e-5:
                    raise PlanningError('navmesh_endpoint','Nonzero road has no complete native output')
                return parts,spent,width
            except PlanningError as exc:
                spent+=getattr(exc,'expanded',0);last=exc
                if exc.code not in ('road_scope_unconnected','no_route','blocked_navmesh') or spent>=budget:break
        last.expanded=spent;raise last


def build_fragmented_diagnostic(args):
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    bundle=DataBundle.load(ROOT/'data');roads,sources,excluded=source_roads(ROOT,bundle)
    pack=NavMeshPack(args.manifest.resolve());pack.record_diagnostics=False
    if pack.native.corridor_path is None or pack.native.road_project is None:raise ValueError('Explicit candidate DLL lacks road ABI')
    validator=Validator(pack);nodes=[];index={};edges=[];edge_index={};report=[];counts=Counter();expanded=0
    def node(p):
        key=(p.c,round(p.x,2),round(p.y,2),round(p.z,2))
        if key not in index:index[key]=len(nodes);nodes.append([p.c,p.x,p.y,p.z])
        return index[key]
    def emit(a,b,points,provenance,width,source):
        u,v=node(a),node(b)
        if u==v:return
        key=(min(u,v),max(u,v));length=sum(p.distance(q) for p,q in zip(points,points[1:]))
        if length<=1e-5:return
        old=edge_index.get(key)
        row=dict(a=u,b=v,length=length,points=[[p.x,p.y,p.z] for p in points],sources=[source],
                 provenance=[provenance],width=width)
        if old is not None:
            previous=edges[old]
            if length<previous['length']*.95:
                row['sources']+=previous['sources'];row['provenance']+=previous['provenance'];edges[old]=row;counts['updated']+=1
            else:previous['sources'].append(source);previous['provenance'].append(provenance);counts['deduplicated']+=1
        else:edge_index[key]=len(edges);edges.append(row)
    roads.sort(key=lambda r:(r.points[0].c,*sorted(pack.around(r.points[0],0))[0],r.provenance['source']!='existing',r.id))
    started=time.monotonic()
    try:
        for number,road in enumerate(roads):
            pts=sample(road.points);valid=0;fail=Counter();spent=0;max_offset=0
            resolved=[]
            for p in pts:
                try:
                    q=validator.project(p);resolved.append(q);max_offset=max(max_offset,p.distance(q))
                except PlanningError as exc:resolved.append(None);fail[exc.code]+=1
            for i,(a,b) in enumerate(zip(resolved,resolved[1:])):
                if a is None or b is None:continue
                if spent>=500000:fail['budget']+=1;continue
                try:
                    parts,count,width=validator.path(a,b,[a,b],min(4096,500000-spent));spent+=count
                    path=[parts[0].points[0],*(s.points[-1] for s in parts)] if parts else [a,b]
                    if sum(p.distance(q) for p,q in zip(path,path[1:]))>max(a.distance(b)*2.5+20,25):
                        fail['large_detour']+=1;continue
                    # An undirected published road must pass in both directions
                    # against the exact geometry that will be stored.
                    reverse,count2,_=validator.path(b,a,list(reversed(path)),min(4096,500000-spent))
                    spent+=count2
                    emit(a,b,path,road.provenance,width,road.id);valid+=1
                except PlanningError as exc:spent+=getattr(exc,'expanded',0);fail[exc.code]+=1
            # A 2-D source line can run beside a bridge rather than on its deck.
            # If local projection did not complete it, align the whole existing
            # source corridor instead of forcing every noisy sample as an
            # endpoint. This is a geometry-wide rule, never a tile patch.
            if valid<len(pts)-1 and not fail['floor_unverified'] and 1<len(pts)<200 and length_xy(pts)<=1000:
                try:
                    m=validator.at(pts)
                    ends=[sorted(m.project_road(p),key=lambda q:p.distance(q))[:4] for p in (pts[0],pts[-1])]
                    trials=[]
                    for a in ends[0]:
                        for b in ends[1]:
                            if spent>=65536:break
                            try:
                                parts,count,width=validator.path(a,b,pts,min(8192,65536-spent));spent+=count
                                path=[parts[0].points[0],*(s.points[-1] for s in parts)] if parts else [a,b]
                                d=length_xy(path)
                                if d<=length_xy(pts)*2.5+20:
                                    score=pts[0].distance(a)+pts[-1].distance(b)+abs(d-length_xy(pts))*.1
                                    trials.append((score,path,width))
                            except PlanningError as exc:spent+=getattr(exc,'expanded',0)
                    if trials:
                        _,aligned,width=min(trials,key=lambda item:item[0]);fine=sample(aligned)
                        for a,b in zip(fine,fine[1:]):
                            try:
                                parts,count,w=validator.path(a,b,[a,b]);spent+=count
                                path=[parts[0].points[0],*(s.points[-1] for s in parts)] if parts else [a,b]
                                validator.path(b,a,list(reversed(path)))
                                emit(a,b,path,{**road.provenance,'alignment':'whole_source_corridor'},w,road.id)
                                counts['aligned_pieces']+=1
                            except PlanningError:pass
                except PlanningError:pass
            expanded+=spent
            status='validated' if valid==len(pts)-1 and valid else 'partial' if valid else 'unverified'
            counts[status]+=1
            report.append(dict(id=road.id,source=road.provenance,status=status,pieces=max(0,len(pts)-1),validated=valid,
                               issues=dict(fail),expanded=spent,projection_max_xy=max_offset))
            if number%100==0:
                print(json.dumps(dict(processed=number+1,total=len(roads),edges=len(edges),status=dict(counts),seconds=round(time.monotonic()-started))),flush=True)
        # Only terminal road ends may acquire a short new connection. The
        # counterpart may be mid-road. Every connection is native-validated.
        degree=Counter()
        for e in edges:degree[e['a']]+=1;degree[e['b']]+=1
        parents=list(range(len(nodes)))
        def component(u):
            while parents[u]!=u:parents[u]=parents[parents[u]];u=parents[u]
            return u
        def union(u,v):parents[component(u)]=component(v)
        for e in edges:union(e['a'],e['b'])
        bins=defaultdict(list)
        for i,row in enumerate(nodes):bins[(row[0],math.floor(row[1]/25),math.floor(row[2]/25))].append(i)
        terminal=[i for i in range(len(nodes)) if degree[i]==1]
        for k,u in enumerate(terminal):
            a=Point(*nodes[u]);bx,by=math.floor(a.x/25),math.floor(a.y/25);near=[]
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    for v in bins[(a.c,bx+dx,by+dy)]:
                        if component(u)==component(v):continue
                        b=Point(*nodes[v]);d=a.distance(b)
                        if d<=25 and abs(a.z-b.z)<=max(1,d):near.append((math.hypot(d,a.z-b.z),v))
            for _,v in sorted(near)[:4]:
                if component(u)==component(v):continue
                b=Point(*nodes[v])
                try:
                    parts,count,width=validator.path(a,b,[a,b]);expanded+=count
                    path=[parts[0].points[0],*(s.points[-1] for s in parts)] if parts else [a,b]
                    if not 1e-5<sum(p.distance(q) for p,q in zip(path,path[1:]))<=25:continue
                    validator.path(b,a,list(reversed(path)))
                    emit(a,b,path,dict(source='verified_gap',max_length=25),width,f'gap:{u}:{v}');union(u,v);counts['gap_connections']+=1
                except PlanningError:pass
            if k%1000==0:print(json.dumps(dict(gap_progress=k,total=len(terminal),counts=dict(counts))),flush=True)
    finally:validator.close()
    shards=defaultdict(list);records=[]
    for i,e in enumerate(edges):
        e['id']=f'unified:{i}';p=Point(*nodes[e['a']]);r,c=next(iter(pack.around(p,0)));key=f'{p.c}-{r}-{c}'
        geometry=e.pop('points');shards[key].append(dict(id=e['id'],points=geometry));e['shard']=key;records.append(e)
    shard_entries={}
    for key,rows in shards.items():
        data=encode(rows);path=out/'tiles'/f'{key}.json';path.parent.mkdir(exist_ok=True);path.write_bytes(data)
        shard_entries[key]=dict(path=path.relative_to(out).as_posix(),bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
    manifest=dict(schema=1,kind='adrive-unified-roads',navigation_identity=pack.identity,build=pack.build,
        native_library_sha256=pack.raw['library']['sha256'],sources=sources,nodes=nodes,edges=records,shards=shard_entries,
        policy=dict(road_weight=1,access_weight=2,max_entries=8,max_gap_yards=25,corridor_widths=[3,9,24],water=False),
        counts=dict(counts),created_at=datetime.now(timezone.utc).isoformat())
    identity=hashlib.sha256(encode(manifest)).hexdigest()[:16];manifest['identity']=identity
    (out/'manifest.json').write_bytes(encode(manifest))
    (out/'audit.json').write_bytes(encode(dict(navigation_identity=pack.identity,native_library_sha256=manifest['native_library_sha256'],
        sources=sources,source_count=len(roads),excluded=excluded,roads=report,counts=dict(counts),expanded=expanded,
        scope='Every published piece is a complete native walking query; unverified source portions are excluded')))
    from tools.finalize_unified_roads import finalize
    identity=finalize(out)
    from tools.bind_unified_roads import bind
    identity=bind(out,args.manifest.resolve())
    print('ROAD_CANDIDATE_READY',out,identity,flush=True)


def build(args):
    from tools.build_topology_roads import build as build_graph
    return build_graph(args)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    build(parser.parse_args())
