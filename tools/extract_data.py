"""Rebuild the portable dataset from the supplied, unmodified reference addons."""
from __future__ import annotations

import argparse
import base64
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from adrive.data import TILE_SIZE, WORLD_TOP, decode_rle
from adrive.lua_data import LiteralReader, array


def extract(root: Path, output: Path) -> dict:
    root, output = root.resolve(), output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    sources: list[dict] = []

    def read(relative: str) -> LiteralReader:
        path = root / relative
        content = path.read_bytes()
        sources.append({"path": relative, "sha256": hashlib.sha256(content).hexdigest(), "bytes": len(content)})
        return LiteralReader(content.decode("utf-8-sig"))

    roads = read("addons/MapWise/RouteRoadData.lua").table_after("return")
    raw_nodes = [array(p) for p in array(roads["nodes"])]
    raw_edges = [array(e) for e in array(roads["edges"])]
    nodes = [row[:3] for row in raw_nodes]
    edges = []
    for i, row in enumerate(raw_edges):
        a, b, status, kind, source = row[:5]
        if not (isinstance(a, int) and isinstance(b, int) and 1 <= a <= len(nodes) and 1 <= b <= len(nodes)):
            raise ValueError(f"Invalid source road edge {i + 1}")
        if nodes[a - 1][0] != nodes[b - 1][0]:
            raise ValueError(f"Road edge crosses continents: {i + 1}")
        edges.append({"id": f"road:{i + 1}", "a": a - 1, "b": b - 1, "status": status, "kind": kind, "source": source, "reference": row[5] if len(row) > 5 else ""})
    terrain = {}
    for c in (0, 1):
        grid = read(f"addons/MapWise/RouteTerrain{c}.lua").table_after("return")
        terrain[c] = {}
        for key, value in grid.items():
            encoded = value.encode("latin-1")
            decode_rle(encoded)
            terrain[c][key] = base64.b64encode(encoded).decode("ascii")

    native = read("addons/MapWise/NativeIndex.lua")
    boxes = {name: native.table_after("Twm_mapareas", "[", name, "]", "=") for name in ("Azeroth", "Kalimdor", "2991")}
    mapping = native.table_after("Twm_UiMapID2Zone", "=")
    mapping.update({1414: {1: "Kalimdor", 2: 0}, 1415: {1: "Azeroth", 2: 0}})
    map_boxes = {}
    for ui_map, relation in mapping.items():
        name, area = array(relation)
        west, east, north, south = array(boxes[name][area])
        map_boxes[ui_map] = {"ui_map": ui_map, "c": {"Azeroth": 0, "Kalimdor": 1, "2991": 2991}[name], "west": west, "east": east, "north": north, "south": south}

    lod = read("addons/OpenMap/Data/MinimapLOD.lua").table_after("ns", ".", "MapMinimapLOD", "=")
    tile_output = output / "maps"
    tile_output.mkdir(exist_ok=True)
    tiles = []
    for c, groups in lod.items():
        for group in array(groups):
            n, keys = array(group)
            for key in array(keys):
                col, row = divmod(key, 64)
                name = f"{c}_{n}_{col}_{row}.tga"
                relative = f"addons/OpenMap/Media/Textures/MapMiniLOD/{name}"
                source = root / relative
                content = source.read_bytes()
                width, height = struct.unpack_from("<HH", content, 12)
                if content[2] != 2 or width != 256 or height != 256 or content[16] != 32:
                    raise ValueError(f"Unexpected local map tile format: {relative}")
                shutil.copyfile(source, tile_output / name)
                sources.append({"path": relative, "sha256": hashlib.sha256(content).hexdigest(), "bytes": len(content)})
                tiles.append({"c": c, "n": n, "col": col, "row": row, "path": f"maps/{name}", "north": WORLD_TOP - row * n * TILE_SIZE, "west": WORLD_TOP - col * n * TILE_SIZE, "size": n * TILE_SIZE})

    licenses = ["addons/MapWise/LICENSE-Lodestar.txt", "addons/MapWise/LICENSE-TerrainWorldMap.txt", "addons/MapWise/SOURCES.txt", "addons/OpenMap/LICENSE.txt", "addons/OpenMap/Media/Textures/NOTICE.md"]
    notices = output / "notices"
    notices.mkdir(exist_ok=True)
    for relative in licenses:
        source = root / relative
        content = source.read_bytes()
        sources.append({"path": relative, "sha256": hashlib.sha256(content).hexdigest(), "bytes": len(content)})
        shutil.copyfile(source, notices / (("OpenMap-" if "/OpenMap/" in relative else "MapWise-") + source.name))
    counts = {"nodes": len(nodes), "edges": len(edges), "terrain_tiles": {str(c): len(grid) for c, grid in terrain.items()}, "map_tiles": len(tiles), "map_boxes": len(map_boxes), "road_status": dict(sorted(Counter(e["status"] for e in edges).items()))}
    manifest = {"schema": 1, "coordinate_system": "world yards; x north, y west; grid row south, column east", "cell_size_yards": TILE_SIZE / 32, "counts": counts, "sources": sources, "limitations": ["Source review labels are not project playtest evidence.", "Terrain is a coarse 2D grid, not collision or enemy-risk data.", "Game artwork remains Blizzard artwork, not MIT licensed."]}
    bundle = {"schema": 1, "nodes": nodes, "node_metadata": [{"name": row[3] if len(row) > 3 else "", "zone": row[4] if len(row) > 4 else None, "access_radius": row[5] if len(row) > 5 else None} for row in raw_nodes], "edges": edges, "terrain": terrain, "tiles": tiles, "map_boxes": map_boxes, "native_boxes": boxes, "manifest": manifest}
    (output / "bundle.json").write_text(json.dumps(bundle, ensure_ascii=False, separators=(",", ":"), sort_keys=True) + "\n", encoding="utf-8")
    (output / "sources.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "README.md").write_text("# 导航数据\n\n由 `python tools/extract_data.py` 从仓库参考插件提取。`bundle.json` 包含所有原始道路（含首版禁止使用的交通类型）、两大陆地形 RLE、水域位、地图坐标与底图索引。客户端只启用地面道路。`sources.json` 记录输入文件摘要、原始数据计数和来源限制。\n\n`maps/` 为 OpenMap 已有小地图瓦片的原样副本，仍属于 Blizzard 的游戏美术，不属于 OpenMap 的 MIT 许可。来源及随附许可在 `notices/`；不包含插件中的额外装饰美术。\n\n`maps-hd/` 是独立的高清地图包，由 `tools/export_maps.py` 从本机游戏资源导出，原始贴图 512×512。客户端优先读取其 `manifest.json`，没有高清包时使用旧概览图。高清包不依赖参考插件的图片，`tools/extract_data.py` 不会覆盖它。来源版本、原始与导出文件摘要、WDT 索引覆盖和版权说明均随包保留；运行 `tools/check_maps.py` 可完整校验。\n\n每个地形块解码为 1024 个八方向连通字节和 128 个水域位字节。约 16.7 码的网格不能表示建筑内部、桥下/桥上楼层、精确碰撞或怪物危险。来源中的审核状态不等于本项目已在游戏中验收。\n\n`navigation/` 是独立的精细导航包，由 `tools/export_navigation.py` 复用本机 Forever 工具，从当前游戏地形和 WMO/M2 碰撞重新生成。客户端检测到清单后优先使用带高度的静态碰撞路线，`tools/extract_data.py` 不覆盖此目录。生成参数、当前版本、覆盖范围、文件摘要及提取日志随包保留；运行 `tools/check_navigation.py` 完整校验。旧道路只作为偏好，不能绕过碰撞判定。动态门、移动运输和怪物不包含在静态数据中；详见 `../docs/navigation.md`。\n\n`submaps/` 是洞穴和地下主城的透明叠加包，由 `tools/export_submaps.py` 读取当前游戏图片，复用 OpenMap 的世界定位、旋转、裁剪和主层规则。当前图片版本与参考定位版本分别记录；`tools/check_submaps.py` 校验图片及来源摘要。此目录不受旧数据提取影响，不改变寻路数据。详情见 `../docs/submaps.md`。\n", encoding="utf-8")
    return counts


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, default=ROOT / "data")
    args = parser.parse_args()
    print(json.dumps(extract(args.root, args.output), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
