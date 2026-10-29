"""Coordinate, resource validation and memory bounds for the desktop map."""
from __future__ import annotations

import json
from pathlib import Path
import struct
import tempfile
import time
import unittest

from PIL import Image

from adrive.data import DataBundle, TILE_SIZE, WORLD_TOP
from adrive.maptiles import ImageCache, Tile, TilePyramid
from adrive.mapview import Viewport
from adrive.models import Point
from tools.export_maps import minimaps_from_wdt

ROOT = Path(__file__).resolve().parents[1]


def temporary():
    parent = ROOT / 'runtime/test-maps'
    parent.mkdir(parents=True, exist_ok=True)
    return tempfile.TemporaryDirectory(dir=parent)


class NativeIndexTests(unittest.TestCase):
    def test_maid_uses_eighth_id_and_row_major_world_orientation(self):
        entries = bytearray(4096 * 32)
        struct.pack_into('<8I', entries, (49 * 64 + 31) * 32, 1, 2, 3, 4, 5, 6, 7, 123456)
        data = struct.pack('<4sII', b'REVM', 4, 18) + struct.pack('<4sI', b'DIAM', len(entries)) + entries
        self.assertEqual(minimaps_from_wdt(data), {(31, 49): 123456})
        tile = Tile(0, 1, 31, 49, Path('unused'), 512)
        self.assertAlmostEqual(tile.north, -9066.666666666668)
        self.assertAlmostEqual(tile.west, 533.3333333333321)
        view = Viewport(0, tile.north, tile.west, 1, 800, 600)
        self.assertEqual(view.screen(Point(0, tile.north, tile.west)), (400, 300))
        self.assertAlmostEqual(view.screen(Point(0, tile.north - TILE_SIZE, tile.west - TILE_SIZE))[0], 400 + TILE_SIZE)

    def test_incomplete_or_unsupported_wdt_never_becomes_empty_success(self):
        for data in (b'', b'DIAM', struct.pack('<4sI', b'DIAM', 131072),
                     struct.pack('<4sII', b'DIAM', 4, 0), struct.pack('<4sII', b'REVM', 4, 18)):
            with self.subTest(size=len(data)), self.assertRaises(ValueError):
                minimaps_from_wdt(data)


class PyramidTests(unittest.TestCase):
    def test_precision_follows_pixels_per_yard_and_visible_world_bounds(self):
        tiles = [Tile(0, n, 31 // n, 49 // n, Path(str(n)), 512) for n in (1, 2, 4, 8, 16, 32, 64)]
        tiles += [Tile(1, 1, 31, 49, Path('other'), 512)]
        pyramid = TilePyramid(tiles)
        for scale, expected in ((.012, 64), (.04, 16), (.1, 8), (.25, 2), (.7, 1), (4, 1)):
            self.assertEqual(pyramid.choose_level(0, scale), expected)
        view = Viewport(0, -9458.437, 50.658, 2, 300, 200)
        self.assertEqual(pyramid.visible(view, 1), [tiles[0]])
        view.north = 50000
        self.assertEqual(pyramid.visible(view, 1), [])

    def test_legacy_bundle_still_has_offline_map_without_hd_pack(self):
        bundle = DataBundle.load(ROOT / 'data')
        bundle.asset_root = None
        pyramid = TilePyramid.from_bundle(bundle)
        self.assertEqual(len(pyramid.tiles), 152)
        self.assertEqual(pyramid.pixels, 256)
        self.assertFalse(pyramid.build)
        self.assertEqual(pyramid.choose_level(0, 4), 4)

    def test_manifest_rejects_escaping_paths_duplicate_tiles_and_wrong_coordinates(self):
        with temporary() as folder:
            path = Path(folder) / 'manifest.json'
            raw = {'schema': 1, 'kind': 'wow-native-minimap-pyramid', 'tile_pixels': 512,
                   'tile_yards': TILE_SIZE, 'world_top': WORLD_TOP, 'tile_count': 2,
                   'native_counts': {'0': 1, '1': 1}, 'build': {'version': 'test'},
                   'tiles': [{'c': c, 'n': 1, 'col': 31, 'row': 49, 'path': f'{c}.webp'} for c in (0, 1)]}
            path.write_text(json.dumps(raw), encoding='utf-8')
            self.assertEqual(len(TilePyramid.load(path).tiles), 2)
            mutations = [lambda v: v['tiles'][0].update(path='../outside.webp'),
                         lambda v: v['tiles'][0].update(row=64),
                         lambda v: v['tiles'][0].update(n=3),
                         lambda v: v.update(tile_yards=512),
                         lambda v: v['tiles'].__setitem__(1, v['tiles'][0])]
            for mutate in mutations:
                changed = json.loads(json.dumps(raw))
                mutate(changed)
                path.write_text(json.dumps(changed), encoding='utf-8')
                with self.assertRaises(ValueError):
                    TilePyramid.load(path)


class CacheTests(unittest.TestCase):
    def test_background_loading_is_bounded_and_bad_image_is_explicit(self):
        with temporary() as folder:
            paths = [Path(folder) / f'{n}.png' for n in range(6)]
            for n, path in enumerate(paths):
                Image.new('RGB', (64, 64), (n * 30, 100, 70)).save(path)
            bad = Path(folder) / 'bad.png'
            bad.write_text('not an image', encoding='utf-8')
            cache = ImageCache(max_bytes=64 * 64 * 4 * 2, max_pending=8)
            try:
                for path in paths + [bad]:
                    self.assertIsNone(cache.get(path))
                deadline = time.monotonic() + 5
                while cache.pending and time.monotonic() < deadline:
                    cache.poll()
                    time.sleep(.005)
                self.assertFalse(cache.pending)
                self.assertIn(bad, cache.failed)
                self.assertLessEqual(cache.bytes, cache.max_bytes)
                self.assertLessEqual(len(cache.images), 2)
                self.assertIsNone(cache.get(bad))
                self.assertNotIn(bad, cache.pending)
            finally:
                cache.close()
            self.assertEqual(cache.bytes, 0)


if __name__ == '__main__':
    unittest.main()
