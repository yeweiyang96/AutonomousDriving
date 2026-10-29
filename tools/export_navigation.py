"""Build a separate walking navmesh from the local client using Forever tools.

Server files are read-only inputs. All intermediate outputs stay in this project.
Use --prepared only to publish an already completed build with provenance.json.
"""
from __future__ import annotations
import argparse
import ctypes as C
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tools.export_maps import installed_build, atomic_json

SETTINGS = {
    'cell_size_yards': .2666666,
    'agent_radius_yards': .5333332,
    'agent_height_yards': 1.5999996,
    'walkable_climb_yards': .5333332,
    'max_slope_degrees': 50,
    'offmesh_connections': False,
    'water': False,
    'dynamic_obstacles': False,
    'partition_method': 'monotone',
}

EXPECTED_MMAPS_GENERATOR_SHA256 = (
    'c298fc3d04de4a2cd48d4a09bc53eca865f3155a66e69cbaca0b2678a6cb2de7'
)

REQUIRED_NATIVE_ABI = (
    'ad_path',
    'ad_budget_reason',
    'ad_path_bounded_v3',
)


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def record(path, root, **metadata):
    return dict(path=path.relative_to(root).as_posix(), bytes=path.stat().st_size,
                sha256=digest(path), **metadata)


def validate_native_library(path):
    if path is None:
        raise ValueError('Publishing requires --native-library')
    path = Path(path).resolve()
    if not path.is_file():
        raise ValueError('Native navigation DLL does not exist: ' + str(path))
    library = C.CDLL(str(path))
    missing = [name for name in REQUIRED_NATIVE_ABI if not hasattr(library, name)]
    if missing:
        raise ValueError(
            'Native navigation DLL is missing required ABI symbols: '
            + ', '.join(missing)
        )
    return {
        'source_path': str(path),
        'bytes': path.stat().st_size,
        'sha256': digest(path),
        'required_abi': list(REQUIRED_NATIVE_ABI),
    }


def publish(prepared, output, native_library):
    native_info = validate_native_library(native_library)
    native_library = Path(native_info['source_path'])
    provenance = json.loads((prepared/'provenance.json').read_text(encoding='utf-8'))
    generator_sha = (
        provenance.get('server', {})
        .get('tools_sha256', {})
        .get('mmaps_generator')
    )
    if generator_sha != EXPECTED_MMAPS_GENERATOR_SHA256:
        raise ValueError(
            'Prepared build used an unapproved mmaps_generator: '
            f'{generator_sha}'
        )
    if provenance['settings'] != SETTINGS:
        raise ValueError('Prepared build settings do not match the walking profile')
    if not all(provenance['stages'].get(s) == 0 for s in ('maps', 'collision', 'assembly', 'metadata', 'nav0', 'nav1')):
        raise ValueError('A required extraction/build stage did not succeed')
    expected = {(int(p.stem.split('_')[0]), int(p.stem.split('_')[1]), int(p.stem.split('_')[2]))
                for p in (prepared/'maps').glob('000[01]_*.map')}
    actual = {(int(p.stem.split('_')[0]), int(p.stem.split('_')[1]), int(p.stem.split('_')[2]))
              for p in (prepared/'mmaps').glob('000[01]_*.mmtile')}
    if not expected or actual != expected:
        raise ValueError(f'Incomplete navigation coverage: expected {len(expected)}, got {len(actual)}; missing {sorted(expected-actual)[:12]}')
    version = provenance['client']['version']
    generation = version + '-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    target = output/generation
    target.mkdir(parents=True)
    (target/'mmaps').mkdir()
    (target/'sources').mkdir()
    maps, tiles = [], []
    for c in (0, 1):
        path = prepared/'mmaps'/f'{c:04}.mmap'
        raw = path.read_bytes()
        if len(raw)<40 or struct.unpack_from('<II',raw)!=(0x4d4d4150,16):
            raise ValueError(f'Invalid mmap header: {path}')
        dest=target/'mmaps'/path.name
        shutil.copyfile(path,dest)
        maps.append(record(dest,output,c=c))
    for i,(c,row,col) in enumerate(sorted(expected),1):
        path=prepared/'mmaps'/f'{c:04}_{row:02}_{col:02}.mmtile'
        raw=path.read_bytes()
        if len(raw)<120 or struct.unpack_from('<III',raw)!=(0x4d4d4150,7,16) or struct.unpack_from('<I',raw,12)[0]!=len(raw)-20:
            raise ValueError(f'Invalid tile: {path}')
        height,radius,climb=struct.unpack_from('<3f',raw,80)
        if not all(abs(actual-SETTINGS[key]) <= .001 for actual,key in
                   ((height,'agent_height_yards'),(radius,'agent_radius_yards'),(climb,'walkable_climb_yards'))):
            raise ValueError(f'Tile uses a different agent profile: {path} {(height,radius,climb)}')
        source_map=prepared/'maps'/f'{c:04}_{row:02}_{col:02}.map'
        if struct.unpack_from('<I',source_map.read_bytes(),8)[0]!=int(version.split('.')[-1]):
            raise ValueError('Terrain came from a different client build')
        dest=target/'mmaps'/path.name
        shutil.copyfile(path,dest)
        tiles.append(record(dest,output,c=c,row=row,col=col,
                            mesh_x=struct.unpack_from('<i',raw,28)[0],mesh_y=struct.unpack_from('<i',raw,32)[0]))
        if i%200==0:
            print(f'Packaged {i}/{len(expected)} tiles',flush=True)
    dll = native_library
    shutil.copyfile(dll, target/'adrive_navmesh.dll')
    atomic_json(target/'sources/publish-native-library.json', native_info)
    for name in ('provenance.json','maps-extract.log','collision-extract.log','collision-assemble.log',
                 'metadata-extract.log','nav-0.log','nav-1.log','old-geometry-comparison.json'):
        if (prepared/name).exists():
            shutil.copyfile(prepared/name,target/'sources'/name)
    raw={'schema':1,'kind':'adrive-ground-navmesh','mmap_version':16,'client':provenance['client'],
         'settings':SETTINGS,'identity':hashlib.sha256(json.dumps(tiles,sort_keys=True).encode()).hexdigest()[:16],
         'maps':maps,'tiles':tiles,'library':record(target/'adrive_navmesh.dll',output),
         'provenance':(target/'sources/provenance.json').relative_to(output).as_posix(),
         'tile_count':len(tiles),'bytes':sum(t['bytes'] for t in tiles),
         'limitations':['Static terrain/WMO/M2 only; no doors, creatures, transports or live obstacle updates.',
                        'Navigation voxel size is not the accuracy of every collision surface or game movement.',
                        'Source code/data validation is not live game acceptance.']}
    atomic_json(target/'pack-root-manifest.json',raw)
    # Validate the complete immutable generation before changing the active pack.
    from tools.check_navigation import check
    report=check(output,raw)
    atomic_json(target/'validation.json',report)
    atomic_json(output/'manifest.json',raw)
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)


def build(args):
    build_info=installed_build(args.game_root,'wowf')
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    work=ROOT/'runtime'/f'nav-build-{build_info["version"]}-{stamp}'
    work.mkdir(parents=True)
    def wsl_path(path):
        resolved = path.resolve()
        raw = str(resolved)
        if len(raw) >= 3 and raw[1:3] in (':\\', ':/'):
            drive = raw[0].lower()
            tail = raw[3:].replace('\\', '/')
            return f'/mnt/{drive}/{tail}'
        # Fallback for non-drive paths (for example UNC/custom-mounted paths).
        # Use forward slashes before handing the argument to WSL so backslashes
        # cannot be consumed as escapes.
        return subprocess.check_output(
            [
                'wsl.exe', '-d', args.distribution, '--',
                'wslpath', '-a', '-u', raw.replace('\\', '/')
            ],
            text=True,
        ).strip()
    cwd,game=wsl_path(work),wsl_path(args.game_root)
    def call(stage,arguments,log):
        command='cd '+shlex.quote(cwd)+' && '+shlex.join(arguments)
        with (work/log).open('wb') as stream:
            result=subprocess.run(['wsl.exe','-d',args.distribution,'--','bash','-lc',command],stdout=stream,stderr=subprocess.STDOUT)
        provenance['commands'][stage]=arguments
        provenance['stages'][stage]=result.returncode
        atomic_json(work/'provenance.json',provenance)
        if result.returncode:
            raise RuntimeError(f'{stage} failed; see {work/log}')
    provenance={'client':build_info,'settings':SETTINGS,'server_source':args.server_source,
                'tools':args.tools,'commands':{},'stages':{}}
    revision=subprocess.check_output(['wsl.exe','-d',args.distribution,'--','git','-C',args.server_source,'rev-parse','HEAD'],text=True).strip()
    provenance['server_revision']=revision
    tools=args.tools.rstrip('/')
    tool_hashes=subprocess.check_output(['wsl.exe','-d',args.distribution,'--','sha256sum',
        *(tools+'/'+name for name in ('mapextractor','vmap4extractor','vmap4assembler','mmaps_generator'))],text=True)
    tool_sha256 = {
        line.split()[1].rsplit('/', 1)[-1]: line.split()[0]
        for line in tool_hashes.splitlines()
    }
    if tool_sha256.get('mmaps_generator') != EXPECTED_MMAPS_GENERATOR_SHA256:
        raise RuntimeError(
            'Unexpected mmaps_generator binary: '
            f'{tool_sha256.get("mmaps_generator")}; '
            f'expected {EXPECTED_MMAPS_GENERATOR_SHA256}'
        )
    source_diff=subprocess.check_output(['wsl.exe','-d',args.distribution,'--','git','-C',args.server_source,
        'diff','HEAD','--','src/tools','src/common/mmaps_common','dep/recastnavigation'])
    provenance['server']={'source':args.server_source,'revision':revision,
        'relevant_worktree_diff_sha256':hashlib.sha256(source_diff).hexdigest(),
        'tools_sha256':tool_sha256}
    call('maps',[tools+'/mapextractor','-i',game,'-o','.','-e','1','-p','wowf','-l','zhCN'],'maps-extract.log')
    call('metadata',[tools+'/mapextractor','-i',game,'-o','.','-e','2','-p','wowf','-l','zhCN'],'metadata-extract.log')
    call('collision',[tools+'/vmap4extractor','-d',game+'/Data','-p','wowf','-dl','zhCN','--threads',str(args.threads)],'collision-extract.log')
    (work/'vmaps').mkdir(exist_ok=True)
    call('assembly',[tools+'/vmap4assembler','Buildings','vmaps'],'collision-assemble.log')
    for c in (0,1):
        call(
            f'nav{c}',
            [
                tools+'/mmaps_generator',
                str(c),
                '--input', '.',
                '--output', '.',
                '--threads', str(args.threads),
                '--maxAngle', '50',
                '--maxAngleNotSteep', '50',
                '--walkableClimb', '2',
                '--walkableRadius', '2',
                '--partition', SETTINGS['partition_method'],
            ],
            f'nav-{c}.log'
        )
    if installed_build(args.game_root,'wowf') != build_info:
        raise RuntimeError('The game build changed during extraction; pack was not published')
    if args.prepare_only:
        print('NAVIGATION_CANDIDATE_READY', flush=True)
        print(f'PREPARED = {work}', flush=True)
        return
    publish(work,args.output,args.native_library)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--game-root',type=Path,default=Path(r'C:\Program Files (x86)\World of Warcraft'))
    parser.add_argument('--distribution',default='Ubuntu')
    parser.add_argument('--server-source',default='/home/wangzekun/forever/TrinityCore-integrated')
    parser.add_argument(
        '--tools',
        default='/home/wangzekun/forever/autonomous-driving-tools/bin',
    )
    parser.add_argument('--threads',type=int,default=4)
    parser.add_argument('--prepared',type=Path)
    parser.add_argument(
        '--native-library',
        type=Path,
        help='Explicit native navigation DLL to package; required when publishing',
    )
    parser.add_argument(
        '--prepare-only',
        action='store_true',
        help='Build the complete candidate into runtime without publishing it',
    )
    parser.add_argument('--output',type=Path,default=ROOT/'data/navigation')
    args=parser.parse_args()
    if args.prepared and args.prepare_only:
        parser.error('--prepared and --prepare-only cannot be used together')
    if not args.prepare_only and args.native_library is None:
        parser.error('--native-library is required when publishing')
    if args.prepared:
        prepared_client=json.loads((args.prepared/'provenance.json').read_text(encoding='utf-8'))['client']
        if installed_build(args.game_root,'wowf') != prepared_client:
            raise RuntimeError('Prepared data does not match the installed client build')
        publish(args.prepared.resolve(),args.output.resolve(),args.native_library.resolve())
    else:
        build(args)
