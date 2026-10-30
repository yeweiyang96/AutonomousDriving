"""Permanent source-junction and graph-quality gate; no game/device input."""
from __future__ import annotations

import argparse
from collections import Counter,defaultdict,deque
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from adrive.models import Point
from adrive.navmesh import NavMeshPack
from adrive.roads import RoadNetwork,RoadPlanner


# These are regression inputs, never production geometry/node patches. Each
# pair of complete original legs was verified in BOTH directions with the
# current native corridor filter before being made a permanent graph gate.
JUNCTION_CASES=(
    dict(name='ironforge_original_junction_9689',source_node_id='existing:9689',
         source_entry_id='existing:9925',source_exit_id='existing:9692',
         source_roads=('road:6440','road:6696'),
         start=(0,-5021.567,-929.246,501.718505859375),
         junction=(0,-4982.488,-965.43,501.718505859375),
         goal=(0,-4941.752636765585,-930.1734196801189,502.4529113769531),
         reference_length_xy=107.53643793741195),
    dict(name='ironforge_original_junction_9691',source_node_id='existing:9691',
         source_entry_id='existing:9926',source_exit_id='existing:9927',
         source_roads=('road:6442','road:6443'),
         start=(0,-4927.97,-854.465,501.718505859375),
         junction=(0,-4897.576,-896.921,501.718505859375),
         goal=(0,-4941.238,-929.728,502.43670654296875),
         reference_length_xy=107.29942469357752),
    dict(name='ironforge_original_junction_9939',source_node_id='existing:9939',
         source_entry_id='existing:9938',source_exit_id='existing:9702',
         source_roads=('road:6454','road:6930'),
         start=(0,-4999.169,-1099.048,502.00469970703125),
         junction=(0,-4992.037,-1154.152,501.718505859375),
         goal=(0,-4959.707278853019,-1192.5532347643348,501.718505859375),
         reference_length_xy=105.76188800092443),
)

# This complete cave reference failed when reverse validation was incorrectly
# guided by the forward output instead of the same immutable source corridor.
# Deleting the source must never make a remaining-edge-only gate turn green.
REQUIRED_SOURCES=('agps:Caves:0:0:1676',)


def _sources(edge):
    """Original IDs may survive on a replacement edge's source trace."""
    values=set()
    containers=[edge,edge.get('metadata',{})]
    for container in containers:
        if not isinstance(container,dict):continue
        for name in ('sources','replacement_sources','replaced_sources'):
            seq=container.get(name,())
            if isinstance(seq,str):seq=[seq]
            if isinstance(seq,(list,tuple)):
                values.update(x for x in seq if isinstance(x,str) and x)
    return values


def _replacement_sources(edge):
    values=set()
    for container in (edge,edge.get('metadata',{})):
        if not isinstance(container,dict):continue
        for key in ('replacement_sources','replaced_sources'):
            seq=container.get(key,())
            if isinstance(seq,str):seq=[seq]
            if isinstance(seq,(list,tuple)):
                values.update(s for s in seq if isinstance(s,str) and s.startswith(('road:','agps:')))
    return values


def _node_sources(raw,n,fail):
    result=[set() for _ in range(n)]
    def add(i,keys):
        if not isinstance(i,int) or isinstance(i,bool) or not 0<=i<n:
            fail('node_source_ids','Source-node mapping contains an invalid node index',node=i);return
        if isinstance(keys,str):keys=[keys]
        if not isinstance(keys,(list,tuple)) or any(not isinstance(k,str) or not k for k in keys):
            fail('node_source_ids','Source-node mapping requires nonempty original IDs',node=i);return
        result[i].update(keys)
    mapping=raw.get('node_source_ids')
    if isinstance(mapping,list) and len(mapping)==n:
        for i,keys in enumerate(mapping):add(i,keys)
    elif isinstance(mapping,dict):
        # Also permit a compact reverse map: original source ID -> node IDs.
        for key,indices in mapping.items():
            if isinstance(indices,int) and not isinstance(indices,bool):indices=[indices]
            if not isinstance(key,str) or not key or not isinstance(indices,list):
                fail('node_source_ids','Invalid reverse source-node mapping');continue
            for i in indices:add(i,[key])
    else:fail('node_source_ids','Missing original source-node mapping or wrong node count')
    aliases=raw.get('node_source_aliases')
    if isinstance(aliases,list):
        if len(aliases)!=n:fail('node_source_aliases','Source-node alias count differs from node count')
        else:
            for i,keys in enumerate(aliases):add(i,keys)
    elif isinstance(aliases,dict):
        for key,keys in aliases.items():
            try:i=int(key)
            except (ValueError,TypeError):fail('node_source_aliases','Invalid source-node alias index');continue
            add(i,keys)
    elif aliases is not None:fail('node_source_aliases','Invalid source-node aliases')
    for i,keys in enumerate(result):
        if not keys:fail('node_source_ids','Node has no traceable source identity',node=i)
    return result


def inspect_graph(raw,*,cases=JUNCTION_CASES,required_sources=REQUIRED_SOURCES,max_failures=200):
    """Pure structural checks, suitable for unit tests without a native DLL."""
    failures=[];failure_counts=Counter()
    def fail(code,message,**details):
        failure_counts[code]+=1
        if len(failures)<max_failures:failures.append(dict(code=code,error=message,**details))
    if raw.get('topology_version')!=2:
        fail('topology_version','A preserved-source topology v2 pack is required')
    policy=raw.get('policy',{})
    if not isinstance(policy,dict) or policy.get('shape_points_are_nodes') is not False:
        fail('shape_point_topology','Collision samples must not become road junction nodes')
    if not isinstance(policy,dict) or policy.get('water') is not False:
        fail('water_policy','Walking graph must retain water=false')
    nodes=raw.get('nodes',[]);edges=raw.get('edges',[])
    if not isinstance(nodes,list) or not isinstance(edges,list):
        fail('graph_format','Nodes and edges must be arrays');nodes=[];edges=[]
    if not nodes or not edges:fail('empty_graph','Road graph must contain usable nodes and edges')
    valid_nodes={}
    for i,node in enumerate(nodes):
        if (not isinstance(node,(list,tuple)) or len(node)!=4 or isinstance(node[0],bool)
                or node[0] not in (0,1)
                or any(not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v) for v in node[1:])):
            fail('node_geometry','Node must have a supported continent and finite collision XYZ',node=i);continue
        valid_nodes[i]=Point(*node)
    aliases=_node_sources(raw,len(nodes),fail)
    adjacency=defaultdict(list);source_adjacency=defaultdict(lambda:defaultdict(list));degree=Counter();edge_ids=set();kind_counts=Counter()
    endpoints=defaultdict(list);road_source_counts=Counter()
    for i,edge in enumerate(edges):
        if not isinstance(edge,dict):fail('edge_format','Edge must be an object',edge_index=i);continue
        identity=edge.get('id')
        if not isinstance(identity,str) or not identity or identity in edge_ids:
            fail('edge_identity','Each edge needs a unique stable ID',edge_index=i,edge=identity)
        else:edge_ids.add(identity)
        kind=edge.get('kind');kind_counts[str(kind)]+=1
        if kind not in ('road','connector'):fail('edge_kind','Edge must explicitly be road or connector',edge=identity)
        factor=edge.get('cost_factor');expected=2 if kind=='connector' else 1
        if isinstance(factor,bool) or not isinstance(factor,(int,float)) or factor!=expected:
            fail('edge_weight','Road cost is length; connector cost is length x2',edge=identity)
        length=edge.get('length')
        if isinstance(length,bool) or not isinstance(length,(int,float)) or not math.isfinite(length) or length<=1e-5:
            fail('edge_length','Published walking edges require positive finite length',edge=identity)
        source_ids=edge.get('sources')
        if (not isinstance(source_ids,list) or not source_ids
                or any(not isinstance(s,str) or not s for s in source_ids)):
            fail('edge_source_trace','Published edges need their original source IDs',edge=identity)
        elif kind=='road' and not any(s.startswith(('road:','agps:')) for s in _sources(edge)):
            fail('edge_source_trace','Road geometry has no original road source trace',edge=identity)
        u,v=edge.get('a'),edge.get('b');bounds=edge.get('bounds')
        # An original closed walking road has one junction and a nonzero
        # complete geometry. Its two incidences contribute degree two. A
        # synthetic zero-length join or connector self edge remains invalid.
        closed_road=(kind=='road' and isinstance(length,(int,float)) and not isinstance(length,bool)
            and math.isfinite(length) and length>1e-5 and isinstance(bounds,(list,tuple)) and len(bounds)==4
            and all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) for x in bounds)
            and max(bounds[1]-bounds[0],bounds[3]-bounds[2])>1e-5)
        if (not isinstance(u,int) or isinstance(u,bool) or not isinstance(v,int) or isinstance(v,bool)
                or u not in valid_nodes or v not in valid_nodes or (u==v and not closed_road)):
            fail('edge_endpoint','Edge endpoint index is invalid or creates a fake self edge',edge=identity);continue
        if valid_nodes[u].c!=valid_nodes[v].c:
            fail('edge_continent','Walking edges cannot cross continents',edge=identity);continue
        for side,node in (('a',u),('b',v)):
            key=edge.get('raw_'+side)
            if not isinstance(key,str) or not key:
                fail('edge_raw_endpoint','Road endpoint lacks its original identity',edge=identity,side=side)
            elif key not in aliases[node]:
                fail('edge_raw_endpoint','Original endpoint identity is not retained on its graph node',edge=identity,side=side,node=node,source_node=key)
        if (not isinstance(bounds,(list,tuple)) or len(bounds)!=4
                or any(not isinstance(x,(int,float)) or isinstance(x,bool) or not math.isfinite(x) for x in bounds)
                or bounds[0]>bounds[1] or bounds[2]>bounds[3]):
            fail('edge_bounds','Complete road geometry requires finite ordered XY bounds',edge=identity)
        elif any(not (bounds[0]-.02<=valid_nodes[k].x<=bounds[1]+.02 and bounds[2]-.02<=valid_nodes[k].y<=bounds[3]+.02) for k in (u,v)):
            fail('edge_bounds','Graph endpoint lies outside its full geometry bounds',edge=identity)
        adjacency[u].append(v);adjacency[v].append(u);degree[u]+=1;degree[v]+=1
        endpoints[(min(u,v),max(u,v),kind)].append(identity)
        traced=_replacement_sources(edge)
        reason=edge.get('metadata',{}).get('connection_reason',edge.get('connection_reason'))
        source_ids=_sources(edge) if kind=='road' else traced if kind=='connector' and reason in ('overlap_entry','overlap_exit') else ()
        if kind=='connector' and traced and reason not in ('overlap_entry','overlap_exit'):
            fail('connector_replacement_trace','Only collision-verified overlap entry/exit connectors may extend an original road replacement',edge=identity)
        if source_ids:
            for source in source_ids:
                source_adjacency[source][u].append(v);source_adjacency[source][v].append(u)
                if kind=='road':road_source_counts[source]+=1
    orphans=[i for i in range(len(nodes)) if not degree[i]]
    if orphans:fail('orphan_nodes','Published graph contains unused or isolated pseudo nodes',count=len(orphans),examples=orphans[:12])
    components={};component_sizes=Counter()
    for i in valid_nodes:
        if not degree[i] or i in components:continue
        todo=[i];components[i]=i
        while todo:
            u=todo.pop();component_sizes[i]+=1
            for v in adjacency[u]:
                if v not in components:components[v]=i;todo.append(v)
    stats=dict(nodes=len(nodes),edges=len(edges),used_nodes=len(degree),orphan_nodes=len(orphans),
        connected_components=len(component_sizes),largest_component_nodes=max(component_sizes.values(),default=0),
        edge_kinds=dict(kind_counts),degree_histogram={str(k):v for k,v in sorted(Counter(degree.values()).items())},
        parallel_endpoint_groups=sum(len(v)>1 for v in endpoints.values()),
        per_continent={str(c):dict(nodes=sum(p.c==c for p in valid_nodes.values()),
            used_nodes=sum(valid_nodes[i].c==c for i in degree),
            components=len({components[i] for i in components if valid_nodes[i].c==c})) for c in (0,1)})
    required=[]
    for source in required_sources:
        count=road_source_counts[source]
        if not count:fail('required_source_missing','A permanent complete-road regression source was removed from the published road graph',source=source)
        required.append(dict(source=source,status='passed' if count else 'failed',road_edges=count))

    def matching(point,source=None):
        return [i for i,p in valid_nodes.items() if (source is None or source in aliases[i])
                and p.distance(point)<=.02 and abs(p.z-point.z)<=.35]
    def reachable(source,starts,targets,floor):
        adj=source_adjacency[source];goals=set(targets);seen=set(starts);todo=deque(starts)
        while todo:
            u=todo.popleft()
            if u in goals:return True
            for v in adj[u]:
                if v not in seen and abs(valid_nodes[v].z-floor)<=2.5:seen.add(v);todo.append(v)
        return False
    junctions=[]
    for case in cases:
        before=sum(failure_counts.values());name=case['name']
        start,junction,goal=(Point(*case[k]) for k in ('start','junction','goal'))
        middles=matching(junction,case['source_node_id'])
        if not middles:
            fail('junction_missing_floor','Original shared junction is missing on its verified collision floor',name=name,
                source_node=case['source_node_id'],expected=vars(junction))
        starts=matching(start,case.get('source_entry_id'));goals=matching(goal,case.get('source_exit_id'))
        if not starts or not goals:
            fail('junction_endpoint_missing','Original road endpoints no longer exist on their verified floors',name=name,
                missing_start=not starts,missing_goal=not goals)
        legs=[]
        for source,ends,label in ((case['source_roads'][0],starts,'entry'),(case['source_roads'][1],goals,'exit')):
            found=bool(ends and middles and reachable(source,ends,middles,junction.z))
            if not found:fail('junction_source_disconnected','Original source road or traced replacement cannot reach its own shared junction on the same floor',
                name=name,source=source,leg=label,source_node=case['source_node_id'])
            legs.append(dict(source=source,leg=label,connected=found))
        junctions.append(dict(name=name,status='passed' if sum(failure_counts.values())==before else 'failed',
            source_node=case['source_node_id'],junction_nodes=middles,source_legs=legs))
    count=sum(failure_counts.values())
    return dict(status='passed' if not count else 'failed',failure_count=count,failures=failures,
        failures_truncated=count>len(failures),failure_counts=dict(failure_counts),stats=stats,junctions=junctions,required_sources=required)


def _verify_route(route,start,goal,case,diagnostic):
    def require(ok,message):
        if not ok:raise ValueError(message)
    require(route.start.distance(start)<=1e-7 and route.goal.distance(goal)<=1e-7
        and route.start.z is not None and route.goal.z is not None
        and abs(route.start.z-start.z)<=.35 and abs(route.goal.z-goal.z)<=.35,
        'Planner changed an original endpoint or its confirmed floor')
    require(bool(route.segments),'Nonzero source route has no complete collision path')
    require(all(s.reliability=='navmesh' for s in route.segments),'A route contains unverified/coarse geometry')
    require(route.segments[0].points[0].distance(start)<=1e-7 and route.segments[-1].points[-1].distance(goal)<=1e-7,'Native path did not reach the original XY endpoints')
    previous=start;low=min(case['start'][3],case['junction'][3],case['goal'][3])-5
    high=max(case['start'][3],case['junction'][3],case['goal'][3])+5
    for segment in route.segments:
        require(bool(segment.points),'Empty native segment')
        first=segment.points[0]
        require(first.distance(previous)<=.02 and first.z is not None and previous.z is not None and abs(first.z-previous.z)<=.35,'Collision route has a discontinuous junction')
        for p in segment.points:
            require(p.c==start.c and p.z is not None and low<=p.z<=high,'Original floor route silently switched to another floor')
        previous=segment.points[-1]
    require(previous.distance(goal)<=1e-7 and abs(previous.z-goal.z)<=.35,'Native route is incomplete')
    require(route.expanded<=500000,'Route exceeded the shared search budget')
    require(route.distance<=case['reference_length_xy']*1.25+20,'Preserved junction acquired an excessive detour')
    selection=diagnostic.get('route_selection',{})
    require(selection.get('selected')=='road_only','Route used an unexpected global fallback')
    require(not any('fallback' in str(row.get('mode','')) for row in diagnostic.get('road_trace',())),'Route performed an unexpected fallback search')


def check(manifest,roads_manifest,report,*,pack=None,network=None,run_routes=True,cases=JUNCTION_CASES,required_sources=REQUIRED_SOURCES):
    """Check candidate or active pack; callers may reuse an already loaded pack."""
    began=time.monotonic();manifest=Path(manifest).resolve();roads_manifest=Path(roads_manifest).resolve();report=Path(report).resolve()
    if roads_manifest.is_dir():roads_manifest=roads_manifest/'manifest.json'
    result=dict(status='failed',failure_count=0,failures=[],route_cases=[],full_route_validation=False,
        scope='Source graph topology and static native collision, no live game movement')
    try:
        pack=pack if pack is not None else NavMeshPack(manifest);pack.record_diagnostics=False
        network=network if network is not None else RoadNetwork(roads_manifest,pack)
        structure=inspect_graph(network.raw,cases=cases,required_sources=required_sources)
        result.update(structure)
        result.update(navigation_identity=pack.identity,native_library_sha256=pack.raw['library']['sha256'],
            road_identity=network.identity,roads_manifest_sha256=hashlib.sha256(roads_manifest.read_bytes()).hexdigest())
        if structure['status']=='passed' and run_routes:
            for case in cases:
                start,goal=Point(*case['start']),Point(*case['goal'])
                for a,b,label in ((start,goal,'forward'),(goal,start,'reverse')):
                    name=case['name']+'_'+label
                    try:
                        route=RoadPlanner(pack,network).plan(a,b)
                        _verify_route(route,a,b,case,pack.last_route_diagnostics or {})
                        row=dict(name=name,status='passed',start=vars(a),goal=vars(b),length_xy=route.distance,expanded=route.expanded,
                            selection=(pack.last_route_diagnostics or {}).get('route_selection'))
                    except Exception as exc:
                        row=dict(name=name,status='failed',code=getattr(exc,'code',type(exc).__name__),error=str(exc))
                        result['failures'].append(row);result['failure_count']+=1
                    result['route_cases'].append(row);print(json.dumps(row,ensure_ascii=False),flush=True)
            result['full_route_validation']=True
        result['status']='passed' if not result['failure_count'] else 'failed'
    except Exception as exc:
        result['failure_count']+=1;result['failures'].append(dict(name='preflight',code=getattr(exc,'code',type(exc).__name__),error=str(exc)))
        result['status']='failed'
    result['seconds']=time.monotonic()-began;report.parent.mkdir(parents=True,exist_ok=True)
    report.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print('ROAD_TOPOLOGY_'+result['status'].upper(),result['failure_count'],report,flush=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,default=ROOT/'data/navigation/manifest.json')
    group=parser.add_mutually_exclusive_group()
    group.add_argument('--roads-manifest',type=Path)
    group.add_argument('--candidate',type=Path,help='Candidate directory or its manifest')
    parser.add_argument('--report',type=Path,default=ROOT/'runtime/road-topology/active.json')
    parser.add_argument('--graph-only',action='store_true',help='Diagnostic only; skips the six native route regressions')
    args=parser.parse_args();roads=args.candidate or args.roads_manifest or ROOT/'data/roads/manifest.json'
    result=check(args.manifest,roads,args.report,run_routes=not args.graph_only)
    raise SystemExit(0 if result['status']=='passed' else 1)
