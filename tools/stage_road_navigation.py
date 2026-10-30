"""Stage existing geometry with an explicit new DLL; never activates/rebuilds tiles."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from adrive.navmesh import Native

def stage(library,output):
    library=library.resolve(strict=True);output=output.resolve()
    native=Native(library)
    if native.corridor_path is None or native.road_project is None or native.bounded_path is None:
        raise ValueError('Selected DLL does not expose the required road/native ABI')
    active=ROOT/'data/navigation';raw=json.loads((active/'manifest.json').read_bytes())
    output.mkdir(parents=True,exist_ok=False)
    for entry in [*raw['maps'],*raw['tiles']]:
        source=(active/entry['path']).resolve();target=(output/entry['path']).resolve()
        if not source.is_relative_to(active.resolve()) or not target.is_relative_to(output):raise ValueError('Navigation path escapes pack')
        target.parent.mkdir(parents=True,exist_ok=True)
        try:os.link(source,target)
        except OSError:shutil.copyfile(source,target)
    target=output/'road-native/adrive_navmesh.dll';target.parent.mkdir();shutil.copyfile(library,target)
    raw['library']=dict(path=target.relative_to(output).as_posix(),bytes=target.stat().st_size,
        sha256=hashlib.sha256(target.read_bytes()).hexdigest())
    raw['road_native_source']=str(library)
    (output/'manifest.json').write_text(json.dumps(raw,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(output/'manifest.json')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--native-library',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();stage(args.native_library,args.output)
