"""Rebind unchanged active roads to a candidate DLL, without publishing either."""
import argparse
import json
from pathlib import Path
import shutil
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tools.bind_unified_roads import bind


def prepare(manifest,output):
    pointer=json.loads((ROOT/'data/roads/manifest.json').read_bytes())
    source=(ROOT/'data/roads'/pointer['path']).parent
    shutil.copytree(source,output)
    path=output/'manifest.json';raw=json.loads(path.read_bytes())
    navigation=json.loads(manifest.read_bytes())
    if navigation['identity']!=raw['navigation_identity']:
        raise ValueError('This release must retain the existing navigation geometry')
    raw['native_library_sha256']=navigation['library']['sha256']
    raw.pop('validation',None)
    path.write_text(json.dumps(raw,ensure_ascii=False)+'\n',encoding='utf-8')
    bind(output,manifest)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();prepare(args.manifest.resolve(),args.output.resolve())
