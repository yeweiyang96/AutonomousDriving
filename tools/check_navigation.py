"""Verify hashes and load every collision-navmesh tile through the native reader."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0,str(ROOT))
from adrive.navmesh import NavMeshPack, Native, Mesh


def check(root, raw=None):
    if raw is None:
        raw=json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    # A staging manifest uses paths relative to root, just like the active pack.
    staging=root/'.validation-manifest.json'
    staging.write_text(json.dumps(raw),encoding='utf-8')
    try:
        pack=NavMeshPack(staging)
        counts=Counter()
        for c in (0,1):
            keys=sorted((r,col) for continent,r,col in pack.tiles if continent==c)
            for start in range(0,len(keys),32):
                with pack.mesh(c) as mesh:
                    mesh.load(keys[start:start+32])
                    counts[str(c)]+=len(mesh.loaded)
        return {'build':pack.build,'tiles_verified':sum(counts.values()),'continents':dict(counts),
                'bytes':sum(t['bytes'] for t in raw['tiles']),'settings':raw['settings'],
                'scope':'SHA256 plus native load of every tile; no live game movement'}
    finally:
        staging.unlink(missing_ok=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',nargs='?',type=Path,default=ROOT/'data/navigation')
    args=parser.parse_args()
    print(json.dumps(check(args.root),ensure_ascii=False,indent=2))
