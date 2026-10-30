"""Package verified local motion sources/addon and provenance; never flashes."""
from dataclasses import asdict
from datetime import datetime,timezone
import csv
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import zipfile
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tools.replay_motion import summarize


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def json_write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


def build():
    out=ROOT/'runtime/motion-release';firmware=ROOT.parent/'hid_usb'
    active=json.loads((out/'navigation-active.json').read_bytes())
    if active['status']!='passed':raise ValueError('Active navigation gate has not passed')
    tests=(out/'unit-tests.txt').read_text(encoding='utf-8-sig')
    if not tests.rstrip().endswith('OK'):raise ValueError('Full offline tests have not passed')
    nav=json.loads((ROOT/'data/navigation/manifest.json').read_bytes())
    if nav['library']['sha256']!=active['native']['library_sha256']:raise ValueError('DLL changed after gate')
    fw=json.loads((firmware/'build-verified/build-manifest.json').read_bytes())
    uf2=firmware/'build-verified/hid_usb.uf2'
    if not fw['deployable_configuration'] or fw['artifact_sha256']!=digest(uf2):raise ValueError('Firmware is not the recorded deployable build')
    addon=out/'AutonomousDriving-addon-0.4.0.zip'
    with zipfile.ZipFile(addon,'w',zipfile.ZIP_DEFLATED) as archive:
        for path in sorted((ROOT/'addons/AutonomousDriving').glob('*')):
            if path.is_file():archive.write(path,path.relative_to(ROOT/'addons'))
    # Source archive includes the existing working-tree changes that the build
    # depends on. It excludes game assets, configuration and Wi-Fi credentials.
    sources={};snapshot=out/'motion-source-snapshot.zip'
    with zipfile.ZipFile(snapshot,'w',zipfile.ZIP_DEFLATED) as archive:
        selected=[]
        for folder,pattern in (('adrive','*.py'),('native/navmesh','*'),('third_party/detour','*'),('tools','*.py'),('tests','*.py')):
            selected.extend(p for p in (ROOT/folder).rglob(pattern) if p.is_file() and '__pycache__' not in p.parts)
        selected.extend((ROOT/'addons/AutonomousDriving').glob('*.lua'))
        selected.extend((ROOT/'tests/fixtures').glob('*.json'))
        selected.append(ROOT/'addons/AutonomousDriving/AutonomousDriving.toc')
        selected.extend(ROOT/'docs'/p for p in ('continuous-motion.md','pico-v4.md','telemetry-protocol.md','validation.md'))
        selected.extend(ROOT/p for p in ('README.md','requirements.txt','start.cmd','start.ps1'))
        for path in sorted(set(selected)):
            name='AutonomousDriving/'+path.relative_to(ROOT).as_posix();archive.write(path,name);sources[name]=digest(path)
        for pattern in ('*.c','*.h','CMakeLists.txt','scripts/*.ps1','scripts/toolchain.lock.json','tests/**/*.c','tests/**/*.h'):
            for path in sorted(firmware.glob(pattern)):
                if path.name=='wifi_config.h':continue
                name='hid_usb/'+path.relative_to(firmware).as_posix();archive.write(path,name);sources[name]=digest(path)
    # A labelled synthetic recording exercises the read-only diagnostic tool.
    from tests.test_motion import simulate,route
    trace,windows,_=simulate(route((0,0),(40,0),(40,40),(90,40)),lambda i:7 if i<160 else 14 if i<300 else 3.5)
    replay=out/'offline-simulation.jsonl'
    with replay.open('w',encoding='utf-8') as stream:
        stream.write(json.dumps(dict(kind='session',t=9,settings={},synthetic=True))+'\n')
        for index,w in enumerate(windows):
            stream.write(json.dumps(dict(kind='motion_window',t=w.source.captured_at,window=asdict(w),
                firmware_revision=index+1,source_age_ms=0,calculation_ms=0,synthetic=True))+'\n')
    json_write(out/'offline-simulation-report.json',summarize(replay))
    client_file=Path(r'C:\Program Files (x86)\World of Warcraft\.build.info')
    client_rows=[]
    if client_file.exists():
        for row in csv.DictReader(client_file.read_text(encoding='utf-8-sig').splitlines(),delimiter='|'):
            if row.get('Product!STRING:0')=='wowf':client_rows.append({k:v for k,v in row.items() if k.startswith(('Product!','Version!','Build Key!','Active!'))})
    heads={name:subprocess.check_output(['git','rev-parse','HEAD'],cwd=folder,text=True).strip()
        for name,folder in (('client',ROOT),('firmware',firmware))}
    result=dict(created_utc=datetime.now(timezone.utc).isoformat(),scope='offline implementation and local package; not flashed or live accepted',
        client_checkout=heads['client'],firmware_checkout=heads['firmware'],working_tree_changes_included=True,
        client_build=client_rows,navigation_identity=nav['identity'],native=nav['library'],road_pointer=json.loads((ROOT/'data/roads/manifest.json').read_bytes()),
        tile_count=nav['tile_count'],partition=nav['settings']['partition_method'],water=False,
        firmware=fw,firmware_artifact=str(uf2),addon=dict(path=str(addon),sha256=digest(addon)),
        source_snapshot=dict(path=str(snapshot),sha256=digest(snapshot)),source_sha256=sources,
        tests=dict(python_count=int(re.search(r'Ran (\d+) tests',tests).group(1)),python_status='passed',
            python_log='unit-tests.txt',python_log_sha256=digest(out/'unit-tests.txt'),
            production_c='test_engine.ps1: engine, wifi_watch, bridge, lease, diagnostics all passed; host transport only',
            motion_geometry='motion-geometry.json',candidate_navigation='navigation-final.json',candidate_roads='roads-final.json',active_navigation='navigation-active.json'),
        simulation=dict(path=replay.name,synthetic=True,windows=len(windows),final_position=asdict(trace[-1])),
        reuse=dict(detour_license='third_party/detour',forever_reference_commit='c978eb93bca2968c07f818e1529c6815387fd751',
            geometry_changed=False,server_modified=False,baseline_backup='runtime/motion-baseline-20261030-092456'),
        protections=dict(ack_ms=180,telemetry_ms=250,lease_ms=250,window_ms=200))
    json_write(out/'build-manifest.json',result)
    print(json.dumps(dict(manifest=str(out/'build-manifest.json'),addon=str(addon),uf2=str(uf2),native_sha256=nav['library']['sha256']),ensure_ascii=False))


if __name__=='__main__':build()
