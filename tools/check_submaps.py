"""Check all overlay hashes, image alpha, coverage and source placement metadata."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from adrive.submaps import Submaps


def check(root,raw=None):
    if raw is None:raw=json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    catalog=Submaps.from_manifest(root,raw)
    source_dir=(root/raw.get('source_directory',str(Path(raw['areas'][0]['tiles'][0]['path']).parts[0])+'/sources')).resolve()
    if not source_dir.is_relative_to(root.resolve()):raise ValueError('Source path escapes pack')
    for source in raw['layout_source']['files']:
        path=(source_dir/source['path']).resolve()
        if not path.is_relative_to(source_dir):raise ValueError('Source path escapes directory')
        content=path.read_bytes()
        if len(content)!=source['bytes'] or hashlib.sha256(content).hexdigest()!=source['sha256']:
            raise ValueError('Retained source checksum mismatch: '+str(path))
    count=0;transparent=0
    for a in raw['areas']:
        for t in a['tiles']:
            path=root/t['path'];content=path.read_bytes()
            if len(content)!=t['bytes'] or hashlib.sha256(content).hexdigest()!=t['sha256']:
                raise ValueError('Overlay checksum mismatch: '+str(path))
            with Image.open(path) as image:
                image.load()
                if image.size!=(512,512) or image.mode not in ('RGB','RGBA'):raise ValueError('Invalid overlay image')
                lo,hi=image.convert('RGBA').getchannel('A').getextrema()
                if not hi:raise ValueError('Empty overlay image')
                transparent+=lo<255
            count+=1
    return dict(build=catalog.build,areas=len(catalog.areas),counts=raw['counts'],
                tiles_verified=count,tiles_with_transparency=transparent,bytes=raw['bytes'],
                layout_build=raw['layout_source']['wmo_layout_build'],
                scope='Offline images, hashes and world placement; not live-game verification')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',nargs='?',type=Path,default=ROOT/'data/submaps')
    args=p.parse_args();print(json.dumps(check(args.root),ensure_ascii=False,indent=2))
