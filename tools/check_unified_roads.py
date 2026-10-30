"""Read-only gate for a unified road candidate or the active road pack."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from adrive.models import Point,PlanningError
from adrive.navmesh import NavMeshPack
from adrive.roads import RoadNetwork,RoadPlanner,length
from tools.build_unified_roads import Validator,encode
from tools.check_navigation_regressions import KNOWN_CASES,WESTFALL,ROAD_LOOP_CASES


def check(manifest,roads_manifest,report,full=True):
    pack=NavMeshPack(manifest);pack.record_diagnostics=False;net=RoadNetwork(roads_manifest,pack)
    complete_graph=net.raw.get('topology_version',1)>=2
    if complete_graph:
        from tools.build_topology_roads import GraphBuilder
        graph_validator=GraphBuilder(pack);validator=graph_validator.validator
    else:validator=Validator(pack)
    failures=[];cases=[];began=time.monotonic();counts=Counter()
    bindings={}
    try:
        if full:
            for i,e in enumerate(net.edges):
                try:
                    pts=net.points(i)
                    if net.raw.get('validation'):
                        if e['shard'] not in bindings:bindings[e['shard']]=net.validation(e['shard'])
                        row=bindings[e['shard']][e['id']]
                        if row['entry']!=[pts[0].c,pts[0].x,pts[0].y,pts[0].z] or row['exit']!=[pts[-1].c,pts[-1].x,pts[-1].y,pts[-1].z]:
                            raise ValueError('Road floor anchors differ from geometry')
                        for direction in ('forward','reverse'):
                            if row[direction]['status']!='passed' or not row[direction]['entry_polygon'] or not row[direction]['exit_polygon']:
                                raise ValueError('Missing complete native surface validation')
                    for seq in (pts,net.points(i,True)):
                        if complete_graph:
                            reference=net.reference(i,seq[0],seq[-1])
                            checked,proof=graph_validator.query(seq[0],seq[-1],reference,connector=e.get('kind')=='connector')
                            distance=length(checked)
                        else:
                            parts,_,_=validator.path(seq[0],seq[-1],seq)
                            if not parts:raise ValueError('Nonzero road edge has no complete native output')
                            distance=sum(s.distance for s in parts)
                        if distance>length(seq)*2.5+20:raise ValueError('excessive local detour')
                    counts['edges_verified']+=1
                except (PlanningError,ValueError,OSError,KeyError) as exc:
                    failures.append(dict(edge=e['id'],error=str(exc),code=getattr(exc,'code','integrity')))
                if i%10000==0:print(json.dumps(dict(edge_progress=i,total=len(net.edges),failures=len(failures))),flush=True)
    finally:validator.close()
    if not failures:
        selected=[c for c in KNOWN_CASES if c['guided']]+list(ROAD_LOOP_CASES[:2])+[WESTFALL,
            dict(name='ironforge_entry',continent=0,start=(-5040,-800,None),goal=(-4821,-1152,None)),
            dict(name='cave_entry',continent=1,start=(1452,-4788,None),goal=(1440,-4830,None))]
        for case in selected:
            try:
                a,b=Point(case['continent'],*case['start']),Point(case['continent'],*case['goal'])
                route=RoadPlanner(pack,net).plan(a,b)
                assert route.start.distance(a)<1e-7 and route.goal.distance(b)<1e-7
                assert all(s.reliability=='navmesh' for s in route.segments)
                assert route.expanded<=500000
                assert pack.last_route_diagnostics['route_selection']['selected']=='road_only'
                assert pack.last_route_diagnostics['route_selection']['route_guide_ratio']<=1.10,'Excessive detour within road corridor'
                row=dict(name=case['name'],status='passed',length_xy=route.distance,expanded=route.expanded,
                    start=vars(route.start),goal=vars(route.goal),selection=pack.last_route_diagnostics['route_selection'])
                cases.append(row);print(json.dumps(row),flush=True)
            except Exception as exc:
                row=dict(name=case['name'],status='failed',error=str(exc),code=getattr(exc,'code',type(exc).__name__))
                failures.append(row);cases.append(row);print(json.dumps(row),flush=True)
    topology=None
    if net.raw.get('topology_version',1)>=2 and not failures:
        from tools.check_road_topology import check as check_topology
        topology=check_topology(manifest,roads_manifest,report.with_suffix('.topology.json'),pack=pack,network=net)
        if topology['status']!='passed':failures.append(dict(name='source_topology',error='Original road junction or complete-route gate failed'))
    result=dict(status='passed' if not failures else 'failed',failures=failures,cases=cases,counts=dict(counts),topology=topology,
        navigation_identity=pack.identity,native_library_sha256=pack.raw['library']['sha256'],road_identity=net.identity,
        roads_manifest_sha256=__import__('hashlib').sha256(roads_manifest.read_bytes()).hexdigest(),
        full_edge_validation=full,seconds=time.monotonic()-began,scope='static native collision and road-only graph; no game movement')
    report.parent.mkdir(parents=True,exist_ok=True);report.write_bytes(encode(result))
    print('UNIFIED_ROADS_'+result['status'].upper(),len(failures),report,flush=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,default=ROOT/'data/navigation/manifest.json')
    parser.add_argument('--roads-manifest',type=Path,default=ROOT/'data/roads/manifest.json')
    parser.add_argument('--report',type=Path,default=ROOT/'runtime/unified-road-validation/active.json')
    parser.add_argument('--routes-only',action='store_true')
    a=parser.parse_args();r=check(a.manifest.resolve(),a.roads_manifest.resolve(),a.report.resolve(),not a.routes_only)
    raise SystemExit(0 if r['status']=='passed' else 1)
