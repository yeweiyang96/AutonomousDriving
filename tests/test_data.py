from __future__ import annotations

import hashlib
import json
from pathlib import Path
import unittest

from adrive.data import DataBundle, MapBox, decode_rle
from adrive.lua_data import LiteralReader, LuaDataError, array
from adrive.models import Point

ROOT = Path(__file__).resolve().parents[1]


class LuaLiteralTests(unittest.TestCase):
    def test_literal_tables_comments_strings_and_signed_exponents(self):
        source = r'''-- return {fake=true}
        function f() return {name="中文", [2]={-1.25e2,true,false,nil}, bytes="\001\255\\\"", ['comment']='--not a comment'} end'''
        data = LiteralReader(source).table_after("return")
        self.assertEqual(data["name"], "中文")
        self.assertEqual(array(data[2]), [-125., True, False, None])
        self.assertEqual(data["bytes"], chr(1) + chr(255) + '\\"')
        self.assertEqual(data["comment"], "--not a comment")

    def test_calls_expressions_and_duplicate_keys_rejected(self):
        for value in ('{a=os.execute("echo nope")}', '{a=1+2}', '{a=1,a=2}', '{a=0/0}'):
            with self.subTest(value=value), self.assertRaises(LuaDataError):
                LiteralReader("return " + value).table_after("return")

    def test_malformed_strings_and_arrays_rejected(self):
        for text in ('return {x="\\999"}', 'return {x="unterminated}', '--[[not closed'):
            with self.subTest(text=text), self.assertRaises(LuaDataError):
                LiteralReader(text).table_after("return")
        with self.assertRaises(LuaDataError):
            array({1: "a", 3: "b"})

    def test_rle_rejects_missing_or_excess_bytes(self):
        for encoded in (b'\1', b'\0\1', bytes((255, 1)) * 5, b'\1\1'):
            with self.subTest(encoded=encoded), self.assertRaises(ValueError):
                decode_rle(encoded)

    def test_coordinate_roundtrip_and_orientation(self):
        box = MapBox(1415, 0, 100., -500., 1000., -2000.)
        point = box.to_world(.25, .75)
        self.assertEqual(point, Point(0, -1250., -50.))
        self.assertEqual(box.from_world(point), (.25, .75))
        with self.assertRaises(ValueError):
            box.from_world(Point(1, 0., 0.))


@unittest.skipUnless((ROOT / "data/bundle.json").exists(), "Run tools/extract_data.py first")
class ExtractedDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle = DataBundle.load(ROOT / "data")

    def test_all_source_rows_preserved(self):
        source = LiteralReader((ROOT / "addons/MapWise/RouteRoadData.lua").read_text(encoding="utf-8-sig")).table_after("return")
        raw_nodes = [array(p) for p in array(source["nodes"])]
        raw_edges = [array(e) for e in array(source["edges"])]
        self.assertEqual(len(raw_nodes), len(self.bundle.nodes))
        self.assertEqual(len(raw_edges), len(self.bundle.edges))
        for p, row in zip(self.bundle.nodes, raw_nodes):
            self.assertEqual((p.c, p.x, p.y), tuple(row[:3]))
        for e, row in zip(self.bundle.edges, raw_edges):
            self.assertEqual((e.a + 1, e.b + 1, e.status, e.kind, e.source), tuple(row[:5]))

    def test_source_hashes_current_and_tiles_identical(self):
        for source in self.bundle.manifest["sources"]:
            path = ROOT / source["path"]
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), source["sha256"], source["path"])
        self.assertEqual(len(self.bundle.tiles), 152)
        for tile in self.bundle.tiles:
            self.assertEqual(tile.path.read_bytes(), (ROOT / "addons/OpenMap/Media/Textures/MapMiniLOD" / tile.path.name).read_bytes())

    def test_terrain_bytes_valid_and_source_tiles_preserved(self):
        for c in (0, 1):
            source = LiteralReader((ROOT / f"addons/MapWise/RouteTerrain{c}.lua").read_text(encoding="utf-8-sig")).table_after("return")
            self.assertEqual(set(source), set(self.bundle.terrain[c]))
            for key, text in source.items():
                self.assertEqual(text.encode("latin-1"), self.bundle.terrain[c][key])
                self.assertEqual(len(decode_rle(self.bundle.terrain[c][key])), 1152)

    def test_native_map_boxes_and_tile_orientation(self):
        self.assertEqual(self.bundle.map_boxes[1414].c, 1)
        self.assertEqual(self.bundle.map_boxes[1415].c, 0)
        for tile in self.bundle.tiles:
            self.assertGreater(tile.size, 0)
            self.assertEqual(tile.path.name, f"{tile.c}_{tile.n}_{tile.col}_{tile.row}.tga")
        for box in self.bundle.map_boxes.values():
            point = box.to_world(.37, .61)
            u, v = box.from_world(point)
            self.assertAlmostEqual(u, .37)
            self.assertAlmostEqual(v, .61)


if __name__ == "__main__":
    unittest.main()
