"""Portable extracted navigation data and lazy terrain decoding."""
from __future__ import annotations

import base64
from dataclasses import dataclass, field
import json
import math
from pathlib import Path

from .models import Point

TILE_SIZE = 1600 / 3
CELL_SIZE = TILE_SIZE / 32
ROAD_ACCESS_RADIUS = 5.0
WORLD_TOP = 32 * TILE_SIZE
GRID_SIZE = 2048
DIRS = ((-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1))
REVIEWED = frozenset(("user_reviewed", "user_correction"))


@dataclass(frozen=True)
class RoadEdge:
    id: str
    a: int
    b: int
    status: str
    kind: str
    source: str
    reference: str = ""


@dataclass(frozen=True)
class MapTile:
    c: int
    n: int
    col: int
    row: int
    path: Path
    north: float
    west: float
    size: float


@dataclass(frozen=True)
class MapBox:
    ui_map: int
    c: int
    west: float
    east: float
    north: float
    south: float

    def to_world(self, u: float, v: float) -> Point:
        return Point(self.c, self.north + v * (self.south - self.north), self.west + u * (self.east - self.west))

    def from_world(self, point: Point) -> tuple[float, float]:
        if point.c != self.c:
            raise ValueError("Map and point belong to different continents")
        return (point.y - self.west) / (self.east - self.west), (point.x - self.north) / (self.south - self.north)


def cell(point: Point) -> tuple[int, int]:
    return math.floor((WORLD_TOP - point.x) / CELL_SIZE), math.floor((WORLD_TOP - point.y) / CELL_SIZE)


def cell_center(c: int, row: int, col: int) -> Point:
    return Point(c, WORLD_TOP - (row + .5) * CELL_SIZE, WORLD_TOP - (col + .5) * CELL_SIZE)


def decode_rle(encoded: bytes) -> bytes:
    if len(encoded) % 2:
        raise ValueError("Odd terrain RLE length")
    output = bytearray()
    for i in range(0, len(encoded), 2):
        count, value = encoded[i:i + 2]
        if not count or len(output) + count > 1152:
            raise ValueError("Invalid terrain RLE run")
        output.extend(bytes((value,)) * count)
    if len(output) != 1152:
        raise ValueError(f"Terrain tile must decode to 1152 bytes, got {len(output)}")
    return bytes(output)


@dataclass
class DataBundle:
    nodes: list[Point]
    edges: list[RoadEdge]
    terrain: dict[int, dict[int, bytes]]
    tiles: list[MapTile] = field(default_factory=list)
    map_boxes: dict[int, MapBox] = field(default_factory=dict)
    node_metadata: list[dict] = field(default_factory=list)
    manifest: dict = field(default_factory=dict)
    asset_root: Path | None = field(default=None, repr=False)
    source_path: Path | None = field(default=None, repr=False)
    _decoded: dict[tuple[int, int], bytes] = field(default_factory=dict, repr=False)

    @classmethod
    def load(cls, path: str | Path) -> DataBundle:
        path = Path(path)
        if path.is_dir():
            path = path / "bundle.json"
        raw = json.loads(path.read_text(encoding="utf-8"))
        if raw.get("schema") != 1:
            raise ValueError("Unsupported navigation data schema")
        nodes = [Point(int(p[0]), float(p[1]), float(p[2])) for p in raw["nodes"]]
        if any(not math.isfinite(p.x) or not math.isfinite(p.y) for p in nodes):
            raise ValueError("Non-finite road coordinate")
        edges = [RoadEdge(**e) for e in raw["edges"]]
        for edge in edges:
            if not 0 <= edge.a < len(nodes) or not 0 <= edge.b < len(nodes):
                raise ValueError(f"Road endpoint missing: {edge.id}")
            if nodes[edge.a].c != nodes[edge.b].c:
                raise ValueError(f"Cross-continent road edge: {edge.id}")
        terrain = {int(c): {int(k): base64.b64decode(v, validate=True) for k, v in grid.items()} for c, grid in raw["terrain"].items()}
        for c, grid in terrain.items():
            for key, encoded in grid.items():
                if not 0 <= key < 4096:
                    raise ValueError(f"Terrain tile outside grid: {c}/{key}")
                decode_rle(encoded)
        tiles = [MapTile(**{**tile, "path": (path.parent / tile["path"]).resolve()}) for tile in raw["tiles"]]
        boxes = {int(k): MapBox(**v) for k, v in raw["map_boxes"].items()}
        return cls(nodes, edges, terrain, tiles, boxes, raw.get("node_metadata", []), raw.get("manifest", {}),
                   asset_root=path.parent.resolve(), source_path=path.resolve())

    def info(self, c: int, row: int, col: int) -> tuple[int, bool]:
        if not 0 <= row < GRID_SIZE or not 0 <= col < GRID_SIZE:
            return 0, False
        key = (row // 32) * 64 + col // 32
        cache_key = (c, key)
        tile = self._decoded.get(cache_key)
        if tile is None:
            encoded = self.terrain.get(c, {}).get(key)
            if encoded is None:
                return 0, False
            tile = decode_rle(encoded)
            self._decoded[cache_key] = tile
        offset = (row % 32) * 32 + col % 32
        return tile[offset], bool(tile[1024 + offset // 8] & (1 << (offset % 8)))

    def dry(self, c: int, row: int, col: int) -> bool:
        conn, water = self.info(c, row, col)
        return bool(conn) and not water

    def can_step(self, c: int, row: int, col: int, nr: int, nc: int,
                 *, allow_water: bool = False) -> bool:
        delta = (nr - row, nc - col)
        if delta not in DIRS:
            return False
        conn, water = self.info(c, row, col)
        target_conn, target_water = self.info(c, nr, nc)
        return (bool(conn & (1 << DIRS.index(delta))) and bool(target_conn) and
                (allow_water or not (water or target_water)))

    def line_clear(self, a: Point, b: Point, *, road_access: bool = False) -> bool:
        """Conservative supercover grid traversal; never hops over a blocked cell.

        Within a cell the source has no finer geometry. Such links are always
        labelled terrain estimates, including exact endpoint-to-grid links.

        A selected reviewed ground road may support an endpoint access estimate
        within ROAD_ACCESS_RADIUS: water can describe the surface below a bridge.
        Only that bounded access may ignore water, never missing connectivity,
        direction bits or blocked corners. Ordinary terrain stays dry-only.
        """
        if a.c != b.c or (road_access and a.distance(b) > ROAD_ACCESS_RADIUS):
            return False
        r, col = cell(a)
        goal_r, goal_col = cell(b)
        conn, water = self.info(a.c, r, col)
        goal_conn, goal_water = self.info(a.c, goal_r, goal_col)
        if not conn or not goal_conn or (not road_access and (water or goal_water)):
            return False

        def can_step(row, col, nr, nc):
            return self.can_step(a.c, row, col, nr, nc, allow_water=road_access)

        dr = (a.x - b.x) / CELL_SIZE
        dc = (a.y - b.y) / CELL_SIZE
        sr, sc = (1 if dr > 0 else -1), (1 if dc > 0 else -1)
        rr, cc = (WORLD_TOP - a.x) / CELL_SIZE, (WORLD_TOP - a.y) / CELL_SIZE
        tr = ((r + 1 - rr) / dr if dr > 0 else (rr - r) / -dr) if dr else math.inf
        tc = ((col + 1 - cc) / dc if dc > 0 else (cc - col) / -dc) if dc else math.inf
        step_r, step_c = (1 / abs(dr) if dr else math.inf), (1 / abs(dc) if dc else math.inf)
        for _ in range(abs(goal_r - r) + abs(goal_col - col) + 2):
            if (r, col) == (goal_r, goal_col):
                return True
            if abs(tr - tc) < 1e-10:
                nr, nc = r + sr, col + sc
                # At a corner, require both side cells and their traversals too.
                if not (can_step(r, col, nr, nc) and
                        can_step(r, col, nr, col) and
                        can_step(r, col, r, nc) and
                        can_step(nr, col, nr, nc) and
                        can_step(r, nc, nr, nc)):
                    return False
                tr += step_r
                tc += step_c
            elif tr < tc:
                nr, nc = r + sr, col
                if not can_step(r, col, nr, nc):
                    return False
                tr += step_r
            else:
                nr, nc = r, col + sc
                if not can_step(r, col, nr, nc):
                    return False
                tc += step_c
            r, col = nr, nc
        return False
