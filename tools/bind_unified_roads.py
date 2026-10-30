"""Attach native surface/floor anchors to a candidate; never changes geometry."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from adrive.navmesh import NavMeshPack
from adrive.roads import RoadNetwork
from tools.build_unified_roads import Validator,encode


def bind(root,manifest):
    root=root.resolve();path=root/'manifest.json';pack=NavMeshPack(manifest)
    pack.record_diagnostics=False;network=RoadNetwork(path,pack)
    complete_graph=network.raw.get('topology_version',1)>=2
    if complete_graph:
        from tools.build_topology_roads import GraphBuilder
        graph_validator=GraphBuilder(pack);validator=graph_validator.validator
    else:validator=Validator(pack)
    rows=defaultdict(list)
    try:
        for i,e in enumerate(network.edges):
            points=network.points(i);directions=[]
            for seq in (points,network.points(i,True)):
                if complete_graph:
                    _,proof=graph_validator.query(seq[0],seq[-1],network.reference(i,seq[0],seq[-1]),connector=e.get('kind')=='connector')
                    directions.append(proof)
                else:
                    parts,expanded,width=validator.path(seq[0],seq[-1],seq)
                    if not parts:raise ValueError('Walking edge has no native output: '+e['id'])
                    directions.append(dict(status='passed',expanded=expanded,corridor_width=width,
                        entry_polygon=parts[0].edge_id.split(':')[-2],exit_polygon=parts[-1].edge_id.split(':')[-1],
                        output_length_xy=sum(s.distance for s in parts)))
            rows[e['shard']].append(dict(id=e['id'],entry=[points[0].c,points[0].x,points[0].y,points[0].z],
                exit=[points[-1].c,points[-1].x,points[-1].y,points[-1].z],
                forward=directions[0],reverse=directions[1]))
            if i%10000==0:print(json.dumps(dict(binding_progress=i,total=len(network.edges))),flush=True)
    finally:validator.close()
    entries={}
    for key,values in rows.items():
        data=encode(values);p=root/'validation'/f'{key}.json';p.parent.mkdir(exist_ok=True);p.write_bytes(data)
        entries[key]=dict(path=p.relative_to(root).as_posix(),bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
    raw=network.raw
    raw['validation']=dict(kind='native-road-corridor-v1',navigation_identity=pack.identity,
        native_library_sha256=pack.raw['library']['sha256'],edge_count=len(network.edges),
        polygon_key_format='tile_x,tile_y,poly_index',floor_evidence='collision surface at each road entry/exit',shards=entries)
    path.write_bytes(encode(raw))
    from tools.finalize_unified_roads import finalize
    identity=finalize(root)
    print('ROAD_BINDINGS_READY',root,identity,flush=True)
    return identity


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();shutil.copytree(args.candidate,args.output);bind(args.output,args.manifest.resolve())
