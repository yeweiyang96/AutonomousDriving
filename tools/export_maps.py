"""Export the installed client's native minimaps and an offline desktop tile pyramid."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import struct
import sys
import urllib.request

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tools.casc_storage import CascStorage


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def installed_build(game: Path, product: str) -> dict:
    content = (game / '.build.info').read_bytes()
    rows = csv.DictReader(io.StringIO(content.decode('utf-8-sig')), delimiter='|')
    matches = [row for row in rows if row.get('Active!DEC:1') == '1'
               and row.get('Product!STRING:0') == product]
    if len(matches) != 1:
        raise ValueError(f'Expected one active {product} client in .build.info')
    row = matches[0]
    return {'version': row['Version!STRING:0'], 'product': product,
            'build_key': row['Build Key!HEX:16'], 'build_info_sha256': digest(content)}


def minimaps_from_wdt(data: bytes) -> dict[tuple[int, int], int]:
    """MAID contains 64 rows x 64 columns; eighth uint32 is the minimap FDID."""
    offset = 0
    tiles = None
    while offset < len(data):
        if offset + 8 > len(data):
            raise ValueError('Truncated WDT header')
        signature, size = struct.unpack_from('<4sI', data, offset)
        offset += 8
        if offset + size > len(data):
            raise ValueError('Truncated WDT chunk')
        if signature == b'DIAM':
            if tiles is not None or size != 64 * 64 * 32:
                raise ValueError('Unexpected WDT MAID layout')
            tiles = {}
            for index in range(4096):
                file_id = struct.unpack_from('<I', data, offset + index * 32 + 28)[0]
                if file_id:
                    row, col = divmod(index, 64)
                    tiles[col, row] = file_id
        offset += size
    if not tiles:
        raise ValueError('No native minimap references in WDT')
    return tiles


def atomic_json(path: Path, value: dict):
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)


def write_tile(image: Image.Image, folder: Path, c: int, n: int, col: int, row: int) -> dict:
    path = folder / str(c) / str(n) / f'{col}_{row}.webp'
    path.parent.mkdir(parents=True, exist_ok=True)
    # Lossless: retain the actual 512px native detail, with no new JPEG/WebP losses.
    image.save(path, 'WEBP', lossless=True, method=3, exact=True)
    content = path.read_bytes()
    with Image.open(io.BytesIO(content)) as check:
        check.load()
        if check.size != (512, 512):
            raise ValueError(f'Invalid exported tile: {path}')
    return {'c': c, 'n': n, 'col': col, 'row': row,
            'path': path.relative_to(folder.parent).as_posix(),
            'sha256': digest(content), 'bytes': len(content)}


def export(game: Path, product: str, library: Path, output: Path, map_table: Path | None) -> dict:
    build = installed_build(game, product)
    version = build['version']
    if not version or any(char not in '0123456789.' for char in version):
        raise ValueError('Invalid client version')
    output.mkdir(parents=True, exist_ok=True)
    # Stage a complete generation before publishing its manifest. A failed run never
    # damages the pack used by an already running desktop client.
    generation = version + '-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    folder = output / generation
    folder.mkdir()
    sources = folder / 'sources'
    sources.mkdir()
    table_url = f'https://wago.tools/db2/Map/csv?build={version}'
    if map_table:
        table_data = map_table.read_bytes()
    else:
        request = urllib.request.Request(table_url, headers={'User-Agent': 'AutonomousDriving-map-export/1'})
        with urllib.request.urlopen(request, timeout=45) as response:
            table_data = response.read()
    rows = list(csv.DictReader(io.StringIO(table_data.decode('utf-8-sig'))))
    if not rows or not {'ID', 'Directory', 'WdtFileDataID'} <= rows[0].keys():
        raise ValueError('The map table is not valid Map CSV data')
    maps = {int(row['ID']): row for row in rows if row['ID'] in ('0', '1')}
    if set(maps) != {0, 1}:
        raise ValueError('Map table must contain both continents')
    (sources / 'Map.csv').write_bytes(table_data)
    exported = []
    raw_tiles = []
    continents = []
    with CascStorage(game, library, product) as storage:
        for c, table_row in sorted(maps.items()):
            wdt_id = int(table_row['WdtFileDataID'])
            wdt = storage.read(wdt_id)
            (sources / f'{c}.wdt').write_bytes(wdt)
            index = minimaps_from_wdt(wdt)
            continents.append({'id': c, 'directory': table_row['Directory'],
                               'wdt_file_id': wdt_id, 'wdt_sha256': digest(wdt),
                               'native_tiles': len(index)})
            print(f'Continent {c}: exporting {len(index)} native tiles from {version}', flush=True)
            level = {}
            for i, ((col, row), file_id) in enumerate(sorted(index.items()), 1):
                content = storage.read(file_id)
                if content[:4] != b'BLP2':
                    raise ValueError(f'FileDataID {file_id} is not BLP2')
                with Image.open(io.BytesIO(content)) as source:
                    source.load()
                    if source.size != (512, 512):
                        raise ValueError(f'Unexpected native resolution: {file_id} {source.size}')
                    image = source.convert('RGBA')
                entry = write_tile(image, folder, c, 1, col, row)
                image.close()
                level[col, row] = output / entry['path']
                entry['file_id'] = file_id
                entry['source_sha256'] = digest(content)
                exported.append(entry)
                raw_tiles.append({'c': c, 'col': col, 'row': row, 'file_id': file_id,
                                  'sha256': digest(content), 'bytes': len(content)})
                if i % 100 == 0 or i == len(index):
                    print(f'  {i}/{len(index)}', flush=True)
            for n in (2, 4, 8, 16, 32, 64):
                parents = sorted({(col // 2, row // 2) for col, row in level})
                next_level = {}
                for col, row in parents:
                    mosaic = Image.new('RGBA', (1024, 1024))
                    for dx in (0, 1):
                        for dy in (0, 1):
                            child = level.get((2 * col + dx, 2 * row + dy))
                            if child:
                                with Image.open(child) as image:
                                    mosaic.paste(image, (dx * 512, dy * 512))
                    reduced = mosaic.resize((512, 512), Image.Resampling.LANCZOS)
                    entry = write_tile(reduced, folder, c, n, col, row)
                    next_level[col, row] = output / entry['path']
                    exported.append(entry)
                    reduced.close()
                    mosaic.close()
                level = next_level
                print(f'  overview {n}: {len(parents)} tiles', flush=True)
    if installed_build(game, product) != build:
        raise RuntimeError('The installed game changed during export; pack was not published')
    library_source = library.parents[2] / 'source.json'
    counts = dict(sorted(Counter(str(tile['c']) for tile in raw_tiles).items()))
    manifest = {'schema': 1, 'kind': 'wow-native-minimap-pyramid', 'tile_pixels': 512,
                'tile_yards': 1600 / 3, 'world_top': 32 * 1600 / 3,
                'coordinate_system': 'x north, y west; tile column east, row south',
                'build': build, 'continents': continents, 'native_counts': counts,
                'tile_count': len(exported), 'bytes': sum(t['bytes'] for t in exported),
                'map_table': {'url': table_url, 'sha256': digest(table_data),
                              'path': (sources / 'Map.csv').relative_to(output).as_posix()},
                'extractor': json.loads(library_source.read_text(encoding='utf-8-sig')) if library_source.exists() else {'dll_sha256': digest(library.read_bytes())},
                'artwork': 'World of Warcraft artwork copyright Blizzard Entertainment; local client export, not covered by project or OpenMap code licenses.',
                'tiles': exported}
    archived = json.loads(json.dumps(manifest))
    for entry in archived['tiles']:
        entry['path'] = Path(entry['path']).relative_to(generation).as_posix()
    archived['map_table']['path'] = Path(archived['map_table']['path']).relative_to(generation).as_posix()
    atomic_json(folder / 'manifest.json', archived)
    atomic_json(output / 'manifest.json', manifest)
    (output / 'NOTICE.txt').write_text(manifest['artwork'] + '\nSource client: ' + version + '\n', encoding='utf-8')
    print(json.dumps({'build': version, 'native_tiles': counts, 'total_tiles': len(exported),
                      'bytes': manifest['bytes'], 'manifest': str(output / 'manifest.json')}, indent=2), flush=True)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--game-root', type=Path, required=True, help='Folder containing .build.info and Data')
    parser.add_argument('--product', default='wowf')
    parser.add_argument('--library', type=Path, default=ROOT / 'runtime/map-tools/build/Release/CascLib.dll')
    parser.add_argument('--output', type=Path, default=ROOT / 'data/maps-hd')
    parser.add_argument('--map-table', type=Path, help='Optional Map CSV for the same installed build')
    args = parser.parse_args()
    export(args.game_root.resolve(), args.product, args.library.resolve(), args.output.resolve(), args.map_table)


if __name__ == '__main__':
    main()
