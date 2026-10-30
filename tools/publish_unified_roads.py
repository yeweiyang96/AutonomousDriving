"""Publish an exactly validated road candidate and its explicit native library."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tools.check_navigation import check
from adrive.navmesh import NavMeshPack
from adrive.roads import RoadNetwork


def atomic(path,data):
    temporary=path.with_suffix('.publishing.tmp');temporary.write_bytes(data);temporary.replace(path)


def publish(args):
    candidate=args.candidate.resolve();manifest=candidate/'manifest.json';raw=json.loads(manifest.read_bytes())
    nav=args.navigation_manifest.resolve();navigation=json.loads(nav.read_bytes())
    gate=json.loads(args.road_report.read_bytes());native=json.loads(args.native_report.read_bytes())
    if (gate['status']!='passed' or not gate['full_edge_validation'] or gate['counts']['edges_verified']!=len(raw['edges'])
        or gate['roads_manifest_sha256']!=hashlib.sha256(manifest.read_bytes()).hexdigest()
        or gate['native_library_sha256']!=navigation['library']['sha256']
        or raw['native_library_sha256']!=navigation['library']['sha256']):raise ValueError('Road candidate did not pass the exact full gate')
    if (native['status']!='passed' or native['full_load']['tiles_verified']!=navigation['tile_count']
        or native['native']['library_sha256']!=navigation['library']['sha256']
        or native['navigation_identity']!=navigation['identity']):raise ValueError('Native candidate did not pass complete navigation regressions')
    if raw.get('topology_version',1)>=2:
        topology=gate.get('topology') or {}
        if topology.get('status')!='passed' or not topology.get('full_route_validation'):
            raise ValueError('Source topology and original junction routes must pass before publication')
    active=ROOT/'data/navigation/manifest.json';before=active.read_bytes();old=json.loads(before)
    if old['identity']!=navigation['identity'] or old['tiles']!=navigation['tiles'] or old['maps']!=navigation['maps']:
        raise ValueError('This release must not regenerate or replace geometry')
    verified=check(nav.parent,navigation)
    if verified['tiles_verified']!=1724:raise ValueError('Unexpected complete geometry coverage')
    pack=NavMeshPack(nav);roads=RoadNetwork(manifest,pack)
    if not raw.get('validation'):raise ValueError('Road publication requires native surface bindings')
    for key in raw['shards']:roads.validation(key)
    for key,entry in raw['shards'].items():
        data=(candidate/entry['path']).read_bytes()
        if len(data)!=entry['bytes'] or hashlib.sha256(data).hexdigest()!=entry['sha256']:raise ValueError('Candidate changed after validation')
    directory=ROOT/'data/roads';directory.mkdir(exist_ok=True);destination=directory/raw['identity']
    if destination.exists():
        if (destination/'manifest.json').read_bytes()!=manifest.read_bytes():raise ValueError('Release identity already exists with different contents')
    else:shutil.copytree(candidate,destination)
    library=nav.parent/navigation['library']['path'];libdir=active.parent/'road-native'/navigation['library']['sha256']
    libdir.mkdir(parents=True,exist_ok=True);libpath=libdir/'adrive_navmesh.dll'
    if not libpath.exists():shutil.copyfile(library,libpath)
    if hashlib.sha256(libpath.read_bytes()).hexdigest()!=navigation['library']['sha256']:raise ValueError('Explicit native library copy mismatch')
    release=ROOT/'runtime/unified-road-releases'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    release.mkdir(parents=True,exist_ok=False);(release/'navigation.before.json').write_bytes(before)
    roadpath=directory/'manifest.json';road_before=roadpath.read_bytes() if roadpath.exists() else None
    if road_before:(release/'roads.before.json').write_bytes(road_before)
    new={**old,'library':{**navigation['library'],'path':libpath.relative_to(active.parent).as_posix()}}
    new['road_release']=dict(identity=raw['identity'],native_source=str(library),native_sha256=navigation['library']['sha256'])
    pointer=dict(schema=1,kind='adrive-road-pointer',path=f"{raw['identity']}/manifest.json",
        sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
        floor_index=dict(path=f"{raw['identity']}/floor-index.json",sha256=raw['floor_index']['sha256']))
    if raw.get('display'):
        display=raw['display'];display_bytes=(candidate/display['path']).read_bytes()
        if len(display_bytes)!=display['bytes'] or hashlib.sha256(display_bytes).hexdigest()!=display['sha256']:
            raise ValueError('Independent road display changed after validation')
        pointer['display']={**display,'path':f"{raw['identity']}/{display['path']}"}
    try:
        atomic(active,(json.dumps(new,ensure_ascii=False,indent=2)+'\n').encode())
        atomic(roadpath,(json.dumps(pointer,ensure_ascii=False,indent=2)+'\n').encode())
    except Exception:
        atomic(active,before)
        if road_before:atomic(roadpath,road_before)
        raise
    result=dict(road_identity=raw['identity'],navigation_identity=old['identity'],native_sha256=new['library']['sha256'],
        edges=len(raw['edges']),nodes=len(raw['nodes']),backup=str(release),road_manifest=str(roadpath),
        road_gate=str(args.road_report.resolve()),native_gate=str(args.native_report.resolve()),scope='offline release; live movement not accepted')
    (release/'release.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',type=Path,required=True);p.add_argument('--navigation-manifest',type=Path,required=True)
    p.add_argument('--road-report',type=Path,required=True);p.add_argument('--native-report',type=Path,required=True)
    publish(p.parse_args())
