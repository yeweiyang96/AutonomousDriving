"""Preserve source graph junctions; collision paths are edge geometry, never new junctions."""
from collections import Counter,defaultdict
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import time

from adrive.data import DataBundle
from adrive.models import Point,PlanningError
from adrive.navmesh import NavMeshPack
from adrive.road_sources import SourceRoad,source_roads
from adrive.roads import projection,slice_line,length
from tools.build_unified_roads import Validator,encode,sample,ROOT


def path_points(parts,a,b):
    if not parts:
        if a.distance(b)>1e-5:raise PlanningError('navmesh_endpoint','Nonzero connection has no complete native path')
        return [a,b]
    return [parts[0].points[0],*(s.points[-1] for s in parts)]


class GraphBuilder:
    def __init__(self,pack):
        self.pack=pack;self.validator=Validator(pack);self.nodes=[];self.node_keys=[];self.node_index={}
        self.node_aliases=[];self.node_polygons=defaultdict(set)
        self.edges=[];self.endpoint_cache={};self.node_meta={};self.audit=[];self.counts=Counter();self.expanded=0
        self.edge_parts={}
    def candidates(self,key,p):
        if key in self.endpoint_cache:return self.endpoint_cache[key]
        mesh=self.validator.at([p]);heights=mesh.heights(p)
        if p.z is not None:heights=[h for h in heights if abs(h-p.z)<=2.5]
        values=[Point(p.c,p.x,p.y,h) for h in heights]
        if not values:values=mesh.project_road(p)
        if p.z is not None:values=[v for v in values if abs(v.z-p.z)<=2.5]
        values=sorted(values,key=lambda q:(p.distance(q),abs(q.z-p.z) if p.z is not None else q.z))[:8]
        self.endpoint_cache[key]=values
        return values
    def query(self,a,b,reference,budget=32768,connector=False):
        if not connector and a.distance(b)<.001 and abs(a.z-b.z)<.05 and length(reference)>2:
            from adrive.road_overlap import _slice
            half=length(reference)/2;pivot=_slice(reference,half,half)[0]
            if pivot.distance(a)<.001:
                along=0.;choices=[]
                for p,q in zip(reference,reference[1:]):along+=p.distance(q);choices.append((q.distance(a),along,q))
                _,half,pivot=max(choices,key=lambda item:item[0])
            mesh=self.validator.at(sample(reference,100));began=self.expanded;last=None
            for z in sorted(mesh.heights(pivot),key=lambda z:abs(z-a.z))[:8]:
                mid=Point(pivot.c,pivot.x,pivot.y,z)
                try:
                    left,f=self.query(a,mid,_slice(reference,0.,half),budget=max(0,budget-(self.expanded-began)))
                    right,r=self.query(mid,b,_slice(reference,half,length(reference)),budget=max(0,budget-(self.expanded-began)))
                    return [*left,*right[1:]],dict(status='passed',expanded=self.expanded-began,
                        corridor_width=max(f['corridor_width'],r['corridor_width']),entry_polygon=f['entry_polygon'],
                        exit_polygon=r['exit_polygon'],output_length_xy=length(left)+length(right))
                except PlanningError as exc:last=exc
            raise last or PlanningError('surface_missing','Closed source road has no verifiable middle surface')
        # Samples are for loading/query bounds, never graph nodes.
        mesh=self.validator.at(sample(reference,spacing=100));spent=0;last=None
        for width in ((None,) if connector else (3.,9.,24.)):
            remaining=budget-spent
            if remaining<=0:break
            try:
                if connector:
                    parts,count=mesh.path(a,b,min(8192,remaining),max_length_xy=max(50.,a.distance(b)*4+50))
                else:
                    parts,count=mesh.path(a,b,min(8192,remaining),road_reference=reference,corridor_width=width)
                spent+=count;self.expanded+=count
                pts=path_points(parts,a,b)
                if length(pts)<=1e-5:raise PlanningError('degenerate','Connection has no walking distance')
                return pts,dict(status='passed',expanded=spent,corridor_width=width,
                    entry_polygon=parts[0].edge_id.split(':')[-2],exit_polygon=parts[-1].edge_id.split(':')[-1],
                    output_length_xy=length(pts))
            except PlanningError as exc:
                count=getattr(exc,'expanded',0);spent+=count;self.expanded+=count;last=exc
                if exc.code not in ('road_scope_unconnected','no_route','blocked_navmesh','budget'):break
        if last is None:last=PlanningError('budget','Source edge validation budget exhausted')
        last.expanded=spent;raise last
    def node(self,key,p,metadata=None):
        # Same original junction and same collision anchor have one identity.
        # Nearby points from unrelated roads are never silently welded.
        identity=(key,p.x,p.y,p.z)
        if identity not in self.node_index:
            self.node_index[identity]=len(self.nodes);self.nodes.append(p);self.node_keys.append(key)
            self.node_aliases.append({key})
            self.node_meta[len(self.nodes)-1]=metadata or {}
        return self.node_index[identity]
    def emit(self,road,a,b,pts,forward,reverse,kind='road',u=None,v=None,reverse_geometry=None,reference=None):
        u=self.node(road.a,a,dict(portal=bool(road.metadata.get('bridge_allowed_a',True)),declared_join=road.metadata.get('join_a',False),
            source_height=road.metadata.get('height_a'),
            special=road.metadata.get('special','normal'),raw_key=road.a)) if u is None else u
        v=self.node(road.b,b,dict(portal=bool(road.metadata.get('bridge_allowed_b',True)),declared_join=road.metadata.get('join_b',False),
            source_height=road.metadata.get('height_b'),
            special=road.metadata.get('special','normal'),raw_key=road.b)) if v is None else v
        if length(pts)<=1e-5:return None
        self.node_polygons[u].add(forward['entry_polygon']);self.node_polygons[v].add(forward['exit_polygon'])
        edge=dict(id=f'graph:{len(self.edges)}',a=u,b=v,length=length(pts),kind=kind,cost_factor=2 if kind=='connector' else 1,
            points=pts,reverse_points=reverse_geometry or list(reversed(pts)),reference=list(reference if reference is not None else road.points),
            display_id=road.id,display_reference=list(road.points),
            sources=[road.id],provenance=[road.provenance],metadata=road.metadata,
            raw_a=road.a,raw_b=road.b,forward=forward,reverse=reverse,active=True)
        self.edges.append(edge);return edge
    def validate(self,road):
        akey,bkey=road.a or road.id+':a',road.b or road.id+':b';road.a,road.b=akey,bkey
        reasons=Counter();success=0;began=self.expanded
        try:
            aa=self.candidates(akey,road.points[0]);bb=self.candidates(bkey,road.points[-1])
            if not aa or not bb:raise PlanningError('surface_missing','No endpoint surface found')
            trials=sorted(((a.distance(road.points[0])+b.distance(road.points[-1]),a,b) for a in aa for b in bb),key=lambda r:r[0])
            for _,a,b in trials:
                if self.expanded-began>=131072:reasons['budget']+=1;break
                # The source corridor is immutable and shared by both
                # directions. A forward obstacle detour is never its centre.
                reference=list(road.points)
                try:
                    pts,f=self.query(a,b,reference,budget=min(32768,131072-(self.expanded-began)))
                    reverse_pts,r=self.query(b,a,list(reversed(reference)),budget=min(32768,131072-(self.expanded-began)))
                    self.emit(road,a,b,pts,f,r,reverse_geometry=reverse_pts,reference=reference);success+=1
                except PlanningError as exc:reasons[exc.code]+=1
            self.counts['validated_sources' if success else 'unverified_sources']+=1
        except PlanningError as exc:reasons[exc.code]+=1;self.counts['unverified_sources']+=1
        self.audit.append(dict(id=road.id,source=road.provenance,original_a=akey,original_b=bkey,
            status='validated' if success else 'unverified',validated_variants=success,issues=dict(reasons),expanded=self.expanded-began))
    def source_floor_compatible(self,u,v):
        # A measured terrain grade is not a declared difference of floors.
        # Match AzerothGPS's guard only when BOTH source heights are known.
        a=self.node_meta[u].get('source_height');b=self.node_meta[v].get('source_height')
        return a is None or b is None or abs(a-b)<=5
    def connector(self,u,v,reason,max_distance=200):
        a,b=self.nodes[u],self.nodes[v]
        if a.c!=b.c or a.distance(b)>max_distance or not self.source_floor_compatible(u,v):return False
        if a.distance(b)<=1e-5:
            if a!=b:return False
            if not self.node_polygons[u].intersection(self.node_polygons[v]):return False
            if self.node_keys[v].startswith('existing:'):u,v=v,u
            # A shared physical anchor is already a verified zero-length join;
            # remap topology instead of publishing a fake walking fragment.
            for edge in self.edges:
                if not edge['active']:continue
                for side in ('a','b'):
                    if edge[side]==v:edge[side]=u
            self.node_aliases[u].update(self.node_aliases[v]);self.node_polygons[u].update(self.node_polygons[v])
            if self.node_meta[u].get('source_height') is None:
                self.node_meta[u]['source_height']=self.node_meta[v].get('source_height')
            self.counts['verified_same_surface_junctions']+=1;return True
        key=tuple(sorted((u,v)))
        if key in getattr(self,'connector_keys',set()):return True
        try:
            pts,f=self.query(a,b,[a,b],connector=True)
            if length(pts)>max_distance*2+25:return False
            reverse_pts,r=self.query(b,a,[b,a],connector=True)
            road=SourceRoad(f'connector:{u}:{v}',[a,b],dict(source='collision_connector',reason=reason),self.node_keys[u],self.node_keys[v],
                dict(special='connector',connection_reason=reason))
            self.emit(road,a,b,pts,f,r,kind='connector',u=u,v=v,reverse_geometry=reverse_pts)
            if not hasattr(self,'connector_keys'):self.connector_keys=set()
            self.connector_keys.add(key);self.counts['validated_connectors']+=1;return True
        except PlanningError as exc:self.counts['connector_'+exc.code]+=1;return False
    def corridor(self,edge,a,b):
        reference=edge['reference']
        if a==self.nodes[edge['a']] and b==self.nodes[edge['b']]:return list(reference)
        if b==self.nodes[edge['a']] and a==self.nodes[edge['b']]:return list(reversed(reference))
        start=projection(a,reference)[1];end=projection(b,reference)[1]
        if abs(start-end)<1e-5:return list(reference)
        return slice_line(reference,start,end)
    def split(self,edge,along):
        total=length(edge['points']);points=edge['points']
        pieces=self.edge_parts.setdefault(edge['id'],[(0.,total,edge)])
        for lo,hi,part in pieces:
            if not part['active']:continue
            if abs(along-lo)<.02:return part['a']
            if abs(along-hi)<.02:return part['b']
        interval=next(((lo,hi,part) for lo,hi,part in pieces if part['active'] and lo<along<hi),None)
        if interval is None:return None
        lo,hi,old=interval;q=slice_line(points,along,along)[0]
        key=f"split:{edge['id']}:{along:.8f}"
        source_height=projection(q,edge['reference'])[3].z
        u=self.node(key,q,dict(portal=True,special=edge['metadata'].get('special','normal'),source_height=source_height,raw_key=key))
        pending=[]
        for a,b,start,end in ((old['a'],u,lo,along),(u,old['b'],along,hi)):
            seq=slice_line(points,start,end);reference=self.corridor(edge,seq[0],seq[-1])
            try:
                pts,f=self.query(seq[0],seq[-1],reference)
                rev,r=self.query(seq[-1],seq[0],list(reversed(reference)))
                pending.append((a,b,start,end,seq,pts,f,r,rev,reference))
            except PlanningError:return None
        old['active']=False;pieces.remove(interval)
        for a,b,start,end,seq,pts,f,r,rev,reference in pending:
            road=SourceRoad(edge['sources'][0],seq,edge['provenance'][0],self.node_keys[a],self.node_keys[b],edge['metadata'])
            child=self.emit(road,seq[0],seq[-1],pts,f,r,u=a,v=b,reverse_geometry=rev,reference=reference)
            child['display_id']=edge['display_id'];child['display_reference']=edge['display_reference']
            child['sources']=list(edge['sources']);child['provenance']=list(edge['provenance'])
            pieces.append((start,end,child))
        return u
    def bind_special_surfaces(self):
        """A cave/inside reference must bind to a declared entry or source floor.

        Otherwise the same 2-D cave drawing can become an isolated road on
        the terrain above it and steal a zero-distance endpoint attachment.
        Keep those successful geometry probes in the audit, never as roads.
        """
        groups=defaultdict(list);source_audits={r['id']:r for r in self.audit}
        for edge in self.edges:
            meta=edge['metadata']
            if edge['active'] and (meta.get('cave') or meta.get('indoor')):
                groups[meta.get('graph_id')].append(edge)
        excluded=set()
        for graph_id,edges in groups.items():
            adj=defaultdict(list)
            for edge in edges:
                adj[edge['a']].append(edge);adj[edge['b']].append(edge)
            visited=set()
            for origin in adj:
                if origin in visited:continue
                todo=[origin];visited.add(origin);component=[];seen_edges=set();bound=False
                while todo:
                    u=todo.pop();bound|=bool(self.node_meta[u].get('portal'))
                    for edge in adj[u]:
                        if edge['id'] not in seen_edges:
                            seen_edges.add(edge['id']);component.append(edge)
                            bound|=edge['metadata'].get('height_a') is not None or edge['metadata'].get('height_b') is not None
                        v=edge['b'] if edge['a']==u else edge['a']
                        if v not in visited:visited.add(v);todo.append(v)
                if bound:continue
                self.counts['unbound_special_surface_components']+=1
                for edge in component:
                    edge['active']=False;self.counts['unbound_surface_variants']+=1
                    for source in edge['sources']:
                        excluded.add(source)
                        source_audits[source].setdefault('excluded_surface_variants',[]).append(dict(
                            code='unbound_surface_component',edge=edge['id'],graph_id=graph_id,
                            entry=vars(self.nodes[edge['a']]),exit=vars(self.nodes[edge['b']]),
                            reason='No declared entrance or source floor binds this restricted surface component'))
        active_sources={s for e in self.edges if e['active'] for s in e['sources']}
        for source in excluded-active_sources:source_audits[source]['status']='unbound'
    def deduplicate(self):
        from adrive.road_overlap import OverlapIndex
        special_kinds={'city','capital','cave','indoor','floor_track'}
        bound_existing=defaultdict(list)
        def original_source(edge):return edge['provenance'][0].get('source')
        connector_lookup=defaultdict(list)
        def remember_connection(edge):
            if edge['kind']=='connector':
                connector_lookup[frozenset((self.nodes[edge['a']],self.nodes[edge['b']]))].append(edge)
        for edge in self.edges:
            if edge['active']:remember_connection(edge)
        def as_road(edge):
            return SourceRoad(edge['id'],edge['points'],edge['provenance'][0],str(edge['a']),str(edge['b']),edge['metadata'])
        def parts(root,lo=0.,hi=None):
            if hi is None:hi=length(root['points'])
            lo,hi=min(lo,hi),max(lo,hi)
            intervals=self.edge_parts.get(root['id'],[(0.,length(root['points']),root)])
            return [edge for a,b,edge in intervals if edge['active'] and min(hi,b)-max(lo,a)>1e-5]
        def split_at(root,along):
            # An earlier zero-distance verified join may have remapped the
            # active child endpoints. Do not return a stale inactive parent's
            # endpoint when another source matches the same original road.
            for lo,hi,edge in self.edge_parts.get(root['id'],[(0.,length(root['points']),root)]):
                if not edge['active']:continue
                if abs(along-lo)<.02:return edge['a']
                if abs(along-hi)<.02:return edge['b']
            return self.split(root,along)
        def trace(edge,sources,provenance,*,canonical=False,reason=None):
            for source in sources:
                if source not in edge['sources']:edge['sources'].append(source)
            edge['provenance'] += [p for p in provenance if p not in edge['provenance']]
            # Child metadata can initially share the source's dictionary. Make
            # this interval's replacement trace independent of its siblings.
            meta=dict(edge.get('metadata',{}))
            previous=set(meta.get('replacement_sources',()))|set(edge.get('replacement_sources',()))
            edge['replacement_sources']=sorted(previous|set(sources))
            meta['replacement_sources']=edge['replacement_sources']
            if canonical:meta['canonical_priority']='verified_special_road_graph'
            if reason:
                old=meta.get('connection_reason')
                if old and old!=reason:meta.setdefault('original_connection_reason',old)
                meta['connection_reason']=reason
            edge['metadata']=meta
        def connection(u,v,reason,sources,provenance):
            began=len(self.edges)
            if not self.connector(u,v,reason,max_distance=12):return None
            for edge in self.edges[began:]:remember_connection(edge)
            a,b=self.nodes[u],self.nodes[v]
            if a==b:return [] # verified same-polygon merge preserved aliases
            found=[]
            for edge in connector_lookup.get(frozenset((a,b)),()):
                if not edge['active']:continue
                # Reusing an older component connection is permitted only if
                # it also meets this short replacement connection's bounds.
                if edge['length']>49 or edge['forward'].get('status')!='passed' or edge['reverse'].get('status')!='passed':continue
                trace(edge,sources,provenance,reason=reason);found.append(edge)
            return found or None
        def adopt(edge,match,lookup,*,canonical=False):
            entry,exit=match['entry'],match['exit']
            a=split_at(lookup[entry['edge_id']],entry['along'])
            b=split_at(lookup[exit['edge_id']],exit['along'])
            if a is None or b is None:return False
            sources=list(edge['sources']);provenance=list(edge['provenance'])
            if connection(edge['a'],a,'overlap_entry',sources,provenance) is None:return False
            if connection(edge['b'],b,'overlap_exit',sources,provenance) is None:return False
            kept=[]
            for interval in match['path']:
                kept.extend(parts(lookup[interval['edge_id']],interval['from_along'],interval['to_along']))
            if not kept:return False
            for part in kept:trace(part,sources,provenance,canonical=canonical)
            edge['active']=False;self.counts['duplicate_whole_roads_removed']+=1
            if canonical:
                self.counts['existing_edges_canonicalized_to_special_roads']+=1
                for source in sources:
                    if source.startswith('road:'):
                        bound_existing[source].append(dict(edge=edge['id'],entry=vars(self.nodes[edge['a']]),exit=vars(self.nodes[edge['b']]),
                            replacement_edges=[part['id'] for part in kept]))
            return True

        # A short old straight segment can lie inside several connected city
        # edges. Matching only each long AGPS edge against a short old segment
        # never removes this duplicate. Match the complete old edge against
        # the verified dedicated graph first, preserving every old endpoint
        # through explicit, bidirectionally verified replacement connections.
        agps=[e for e in self.edges if e['active'] and e['kind']=='road' and original_source(e)=='AzerothGPS']
        dedicated=[e for e in agps if e['metadata'].get('special','normal') in special_kinds]
        special_ids={e['id'] for e in dedicated};lookup={e['id']:e for e in agps}
        special_index=OverlapIndex([as_road(e) for e in dedicated]) if dedicated else None
        mixed_index=None
        for edge in list(self.edges):
            if not edge['active'] or edge['kind']!='road' or original_source(edge)!='existing':continue
            road=as_road(edge);match=special_index.find(road) if special_index else None
            if match is None and special_index:
                if mixed_index is None:mixed_index=OverlapIndex([as_road(e) for e in agps])
                proposal=mixed_index.find(road)
                # Ordinary roads may complete a dedicated corridor, but this
                # pass must not reverse the normal existing-road preference
                # over unrelated wilderness merely because another line is
                # nearby. At least half the actual path must be dedicated.
                if proposal:
                    dedicated_length=sum(abs(p['to_along']-p['from_along']) for p in proposal['path'] if p['edge_id'] in special_ids)
                    if dedicated_length>=proposal['existing_length']*.5:match=proposal
            if match is not None:adopt(edge,match,lookup,canonical=True)

        # A 2-D source is one road reference, not a promise that every static
        # surface under its XY is another road. Once a complete dedicated
        # corridor binds that source to its actual road floor(s), discard the
        # remaining unsupported surface variants without welding their floors.
        # Preserve the original static-query results and the exclusion reason.
        source_audits={row['id']:row for row in self.audit}
        for source,variants in bound_existing.items():
            row=source_audits.get(source)
            if row is None:
                row=dict(id=source,status='surface_binding');self.audit.append(row);source_audits[source]=row
            row['road_binding']='verified_special_road_graph';row['bound_surface_variants']=variants
        for edge in self.edges:
            if not edge['active'] or edge['kind']!='road' or original_source(edge)!='existing':continue
            bound=[source for source in edge['sources'] if source in bound_existing]
            if not bound:continue
            edge['active']=False;self.counts['unbound_surface_variants']+=1
            item=dict(code='unbound_surface_variant',edge=edge['id'],entry=vars(self.nodes[edge['a']]),exit=vars(self.nodes[edge['b']]),
                reason='Static walkability does not bind a second surface to this source road')
            for source in bound:source_audits[source].setdefault('excluded_surface_variants',[]).append(item)

        # Remaining ordinary overlays retain the established geometry when
        # there is no >=5% improvement. Update active split children, not only
        # their superseded parent records, so source tracking cannot vanish.
        preferred=[as_road(e) for e in self.edges if e['active'] and e['kind']=='road' and original_source(e)=='existing']
        index=OverlapIndex(preferred);lookup={e['id']:e for e in self.edges}
        for edge in list(self.edges):
            if (not edge['active'] or edge['kind']!='road' or original_source(edge)!='AzerothGPS'
                    or edge['metadata'].get('canonical_priority')=='verified_special_road_graph'
                    or edge['metadata'].get('special','normal') not in ('normal','city','capital')):continue
            match=index.find(as_road(edge))
            if match and match['keep_existing']:adopt(edge,match,lookup)
    def connect_components(self):
        parents=list(range(len(self.nodes)))
        def find(u):
            while parents[u]!=u:parents[u]=parents[parents[u]];u=parents[u]
            return u
        def union(u,v):parents[find(u)]=find(v)
        degree=Counter()
        for e in self.edges:
            if e['active']:union(e['a'],e['b']);degree[e['a']]+=1;degree[e['b']]+=1
        # Different sources describe the same junction, and overlays explicitly
        # expose doors/mouths. Only these graph endpoints can acquire links.
        bins=defaultdict(list);eligible=[]
        for u,p in enumerate(self.nodes):
            meta=self.node_meta[u]
            if not degree[u] or not meta.get('portal',True):continue
            eligible.append(u);bins[(p.c,math.floor(p.x/200),math.floor(p.y/200))].append(u)
        candidates=[]
        for u in eligible:
            p=self.nodes[u];bx,by=math.floor(p.x/200),math.floor(p.y/200);near=[]
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    for v in bins.get((p.c,bx+dx,by+dy),()):
                        if u>=v or find(u)==find(v):continue
                        q=self.nodes[v];d=p.distance(q)
                        # A doorway/cave mouth only receives short access; it
                        # cannot provide a passage across another building.
                        limit=40 if self.node_meta[u].get('special')=='city' or self.node_meta[v].get('special')=='city' else 200
                        if d<=limit and self.source_floor_compatible(u,v):near.append((d,u,v,limit))
            candidates.extend(sorted(near)[:6])
        attempts=Counter()
        for k,(d,u,v,limit) in enumerate(sorted(candidates)):
            if k%1000==0:print(json.dumps(dict(connector_progress=k,total=len(candidates),counts=dict(self.counts))),flush=True)
            if find(u)==find(v):continue
            pair=tuple(sorted((find(u),find(v))))
            if attempts[pair]>=12:self.counts['connector_pair_budget_exhausted']+=1;continue
            attempts[pair]+=1
            if self.connector(u,v,'different_road_components',limit):union(u,v)
        groups=Counter(find(u) for u in degree)
        self.counts['road_components']=len(groups);self.counts['largest_component_nodes']=max(groups.values(),default=0)

    def connect_portals(self):
        from adrive.road_overlap import OverlapIndex
        preferred=[SourceRoad(e['id'],e['points'],e['provenance'][0],str(e['a']),str(e['b']),e['metadata'])
            for e in self.edges if e['active'] and e['kind']=='road' and e['metadata'].get('special','normal')=='normal']
        index=OverlapIndex(preferred);lookup={e['id']:e for e in self.edges}
        for u,p in enumerate(list(self.nodes)):
            if not self.node_meta[u].get('declared_join'):continue
            options=[]
            for i in index.grid.get((p.c,math.floor(p.x/64),math.floor(p.y/64)),()):
                edge=lookup[index.roads[i].id]
                if u in (edge['a'],edge['b']):continue
                nearest=index.lines[i].nearest(p,limit=12)
                if nearest:options.append((nearest[0],i,nearest[1]))
            for _,i,along in sorted(options)[:4]:
                v=self.split(lookup[index.roads[i].id],along)
                if v is not None and self.connector(u,v,'declared_overlay_entrance',12):
                    self.counts['declared_portals_connected']+=1;break


def build(args):
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    bundle=DataBundle.load(ROOT/'data');roads,sources,excluded=source_roads(ROOT,bundle)
    pack=NavMeshPack(args.manifest.resolve());pack.record_diagnostics=False;builder=GraphBuilder(pack)
    started=time.monotonic()
    roads.sort(key=lambda r:(r.provenance.get('source')!='existing',r.points[0].c,*sorted(pack.around(r.points[0],0))[0],r.id))
    try:
        for i,road in enumerate(roads):
            builder.validate(road)
            if i%100==0:print(json.dumps(dict(processed=i+1,total=len(roads),edges=len(builder.edges),counts=dict(builder.counts),seconds=time.monotonic()-started)),flush=True)
        (out/'source-validation-checkpoint.json').write_bytes(encode(builder.audit))
        builder.bind_special_surfaces();builder.deduplicate();builder.connect_portals();builder.connect_components()
    finally:builder.validator.close()
    # Failed candidates and superseded overlaps cannot leave orphan nodes in
    # the published graph. Preserve every original identity as an alias.
    used=sorted({e[k] for e in builder.edges if e['active'] for k in ('a','b')});remap={u:i for i,u in enumerate(used)}
    geometry=defaultdict(list);validation=defaultdict(list);records=[];display=[];display_ids=set()
    for e in builder.edges:
        if not e['active']:continue
        pts=e['points'];a,b=builder.nodes[e['a']],builder.nodes[e['b']]
        row,col=next(iter(pack.around(a,0)));key=f'{a.c}-{row}-{col}';e['shard']=key
        geometry[key].append(dict(id=e['id'],points=[[p.x,p.y,p.z] for p in pts],
            reverse_points=[[p.x,p.y,p.z] for p in e['reverse_points']],reference=[[p.x,p.y,p.z] for p in e['reference']]))
        validation[key].append(dict(id=e['id'],entry=[a.c,a.x,a.y,a.z],exit=[b.c,b.x,b.y,b.z],forward=e['forward'],reverse=e['reverse']))
        record={k:e[k] for k in ('id','a','b','length','kind','cost_factor','sources','provenance','raw_a','raw_b','metadata','shard')}
        both=[*pts,*e['reverse_points']]
        record['bounds']=[min(p.x for p in both),max(p.x for p in both),min(p.y for p in both),max(p.y for p in both)];records.append(record)
        record['reverse_length']=length(e['reverse_points'])
        record['a'],record['b']=remap[e['a']],remap[e['b']]
        identity=(e['display_id'],e['kind'])
        if identity not in display_ids:
            display_ids.add(identity);display.append(dict(id=e['id'],c=a.c,kind=e['kind'],points=[[p.x,p.y] for p in e['display_reference']]))
    def shards(rows,folder):
        entries={}
        for key,values in rows.items():
            data=encode(values);p=out/folder/f'{key}.json';p.parent.mkdir(exist_ok=True);p.write_bytes(data)
            entries[key]=dict(path=p.relative_to(out).as_posix(),bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
        return entries
    sha=pack.raw['library']['sha256'];geom=shards(geometry,'tiles');proof=shards(validation,'validation')
    displayed=encode(dict(schema=1,kind='adrive-road-display',roads=display));(out/'display.json').write_bytes(displayed)
    raw=dict(schema=1,kind='adrive-unified-roads',topology_version=2,navigation_identity=pack.identity,build=pack.build,
        native_library_sha256=sha,nodes=[[builder.nodes[u].c,builder.nodes[u].x,builder.nodes[u].y,builder.nodes[u].z] for u in used],
        node_source_ids=[builder.node_keys[u] for u in used],node_source_aliases=[sorted(builder.node_aliases[u]) for u in used],
        edges=records,shards=geom,sources=sources,counts=dict(builder.counts),created_at=datetime.now(timezone.utc).isoformat(),
        display=dict(path='display.json',bytes=len(displayed),sha256=hashlib.sha256(displayed).hexdigest()),
        validation=dict(kind='native-road-corridor-v1',navigation_identity=pack.identity,native_library_sha256=sha,
            edge_count=len(records),polygon_key_format='tile_x,tile_y,poly_index',shards=proof),
        policy=dict(road_weight=1,access_weight=2,connector_weight=2,max_entries=8,corridor_widths=[3,9,24],water=False,
            shape_points_are_nodes=False,connector_scope='different source-road components or declared entrances'))
    (out/'manifest.json').write_bytes(encode(raw));(out/'audit.json').write_bytes(encode(dict(source_count=len(roads),sources=sources,excluded=excluded,
        roads=builder.audit,counts=dict(builder.counts),expanded=builder.expanded,scope='Complete source edges, preserved original junctions, collision-verified connectors')))
    from tools.finalize_unified_roads import finalize
    print('ROAD_CANDIDATE_READY',out,finalize(out),flush=True)
