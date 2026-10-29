"""Offline map pyramid selection and bounded background image decoding."""
from __future__ import annotations

from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import json
import math
from pathlib import Path

from PIL import Image

from .data import TILE_SIZE, WORLD_TOP


@dataclass(frozen=True)
class Tile:
    c: int
    n: int
    col: int
    row: int
    path: Path
    pixels: int

    @property
    def north(self):
        return WORLD_TOP - self.row * self.n * TILE_SIZE

    @property
    def west(self):
        return WORLD_TOP - self.col * self.n * TILE_SIZE

    @property
    def size(self):
        return self.n * TILE_SIZE


class TilePyramid:
    def __init__(self, tiles, *, build='', error=''):
        self.tiles = {(t.c, t.n, t.col, t.row): t for t in tiles}
        self.levels = {c: sorted({t.n for t in self.tiles.values() if t.c == c})
                       for c in {t.c for t in self.tiles.values()}}
        self.build, self.error = build, error
        self.pixels = next(iter(self.tiles.values())).pixels if self.tiles else 256

    @classmethod
    def from_bundle(cls, bundle):
        manifest = bundle.asset_root / 'maps-hd/manifest.json' if bundle.asset_root else None
        error = ''
        if manifest and manifest.exists():
            try:
                return cls.load(manifest)
            except (OSError, ValueError, TypeError, KeyError) as exc:
                error = f'高清地图不可用：{exc}'
        return cls([Tile(t.c, t.n, t.col, t.row, t.path, 256) for t in bundle.tiles], error=error)

    @classmethod
    def load(cls, manifest: Path):
        raw = json.loads(manifest.read_text(encoding='utf-8'))
        if (raw['schema'] != 1 or raw['kind'] != 'wow-native-minimap-pyramid'
                or raw['tile_pixels'] != 512 or not math.isclose(raw['tile_yards'], TILE_SIZE)
                or not math.isclose(raw['world_top'], WORLD_TOP)):
            raise ValueError('地图坐标或图片格式不兼容')
        root = manifest.parent.resolve()
        tiles, keys = [], set()
        for item in raw['tiles']:
            c, n, col, row = (item[k] for k in ('c', 'n', 'col', 'row'))
            if (any(type(v) is not int for v in (c, n, col, row)) or c not in (0, 1)
                    or n not in (1, 2, 4, 8, 16, 32, 64)
                    or not 0 <= col < 64 // n or not 0 <= row < 64 // n):
                raise ValueError('地图瓦片坐标无效')
            path = (root / item['path']).resolve()
            if not path.is_relative_to(root) or path.suffix.lower() != '.webp':
                raise ValueError('地图图片路径无效')
            key = (c, n, col, row)
            if key in keys:
                raise ValueError('地图瓦片坐标重复')
            keys.add(key)
            tiles.append(Tile(c, n, col, row, path, 512))
        if not tiles or len(tiles) != raw['tile_count']:
            raise ValueError('地图瓦片清单不完整')
        for c in (0, 1):
            if sum(t.c == c and t.n == 1 for t in tiles) != raw['native_counts'][str(c)]:
                raise ValueError('大陆高清地图清单不完整')
        return cls(tiles, build=raw['build']['version'])

    def choose_level(self, c: int, scale: float) -> int:
        levels = self.levels.get(c, [1])
        target = self.pixels / (TILE_SIZE * scale)
        return max((n for n in levels if n <= target), default=levels[0])

    def visible(self, view, n: int) -> list[Tile]:
        size = n * TILE_SIZE
        half_x = (view.width / 2 + 2) / view.scale
        half_y = (view.height / 2 + 2) / view.scale
        c0 = max(0, math.floor((WORLD_TOP - view.west - half_x) / size))
        c1 = min(64 // n - 1, math.floor((WORLD_TOP - view.west + half_x) / size))
        r0 = max(0, math.floor((WORLD_TOP - view.north - half_y) / size))
        r1 = min(64 // n - 1, math.floor((WORLD_TOP - view.north + half_y) / size))
        return [tile for col in range(c0, c1 + 1) for row in range(r0, r1 + 1)
                if (tile := self.tiles.get((view.c, n, col, row))) is not None]


class ImageCache:
    """Worker threads decode PIL images only; all Tk operations stay on the UI thread."""
    def __init__(self, max_bytes=96 * 1024 * 1024, max_pending=32):
        self.max_bytes, self.max_pending = max_bytes, max_pending
        self.images = OrderedDict()
        self.pending = {}
        self.failed = set()
        self.bytes = 0
        self.closed = False
        self.executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix='map-images')

    @staticmethod
    def decode(path):
        with Image.open(path) as image:
            if image.width > 1024 or image.height > 1024:
                raise ValueError('Oversized map tile')
            image.load()
            return image.convert('RGBA')

    def get(self, path: Path, *, request=True):
        image = self.images.get(path)
        if image is not None:
            self.images.move_to_end(path)
        elif (request and not self.closed and path not in self.failed and path not in self.pending
              and len(self.pending) < self.max_pending):
            self.pending[path] = self.executor.submit(self.decode, path)
        return image

    def poll(self):
        changed = False
        for path, future in list(self.pending.items()):
            if not future.done():
                continue
            del self.pending[path]
            changed = True
            try:
                image = future.result()
            except Exception:
                self.failed.add(path)
                continue
            self.images[path] = image
            self.bytes += image.width * image.height * 4
            while self.bytes > self.max_bytes and len(self.images) > 1:
                _, old = self.images.popitem(last=False)
                self.bytes -= old.width * old.height * 4
                old.close()
        return changed

    def close(self):
        if self.closed:
            return
        self.closed = True
        for future in self.pending.values():
            if not future.cancel():
                def release(f):
                    try:
                        f.result().close()
                    except Exception:
                        pass
                future.add_done_callback(release)
        self.pending.clear()
        self.executor.shutdown(wait=False, cancel_futures=True)
        for image in self.images.values():
            image.close()
        self.images.clear()
        self.bytes = 0
