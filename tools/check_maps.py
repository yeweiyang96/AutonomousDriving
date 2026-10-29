"""Check every installed HD map file against its manifest and local WDT references."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from adrive.maptiles import TilePyramid
from tools.export_maps import minimaps_from_wdt


def check(path):
    pyramid = TilePyramid.load(path)
    raw = json.loads(path.read_text(encoding='utf-8'))
    root = path.parent
    levels = Counter()
    for entry in raw['tiles']:
        image_path = root / entry['path']
        content = image_path.read_bytes()
        if hashlib.sha256(content).hexdigest() != entry['sha256']:
            raise ValueError(f'Hash mismatch: {entry["path"]}')
        with Image.open(image_path) as image:
            image.load()
            if image.size != (512, 512) or image.format != 'WEBP':
                raise ValueError(f'Invalid image: {entry["path"]}')
        levels[entry['n']] += 1
    sources = (root / raw['map_table']['path']).parent
    for continent in raw['continents']:
        c = continent['id']
        data = (sources / f'{c}.wdt').read_bytes()
        if hashlib.sha256(data).hexdigest() != continent['wdt_sha256']:
            raise ValueError(f'WDT hash mismatch: {c}')
        expected = minimaps_from_wdt(data)
        actual = {(t['col'], t['row']): t['file_id'] for t in raw['tiles'] if t['c'] == c and t['n'] == 1}
        if expected != actual:
            raise ValueError(f'Native tile coverage mismatch: {c}')
    return {'build': pyramid.build, 'images_verified': len(raw['tiles']),
            'native_tiles': raw['native_counts'], 'levels': dict(sorted(levels.items())),
            'bytes': raw['bytes'], 'coordinates': 'matched local WDT row/column index'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', nargs='?', type=Path, default=ROOT / 'data/maps-hd/manifest.json')
    args = parser.parse_args()
    print(json.dumps(check(args.manifest), ensure_ascii=False, indent=2))
