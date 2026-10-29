#!/usr/bin/env python3
"""Collect the fixed Westfall route case using the Python standard library.

Run with the project's Python. No project module is imported or executed. This
script only reads explicitly named inputs and writes a new runtime case ZIP.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import stat
import sys
import uuid
import zipfile


DEFAULT_ROOT = r"D:\wangzekun\Documents\AutonomousDriving"
COLLECTOR_VERSION = "westfall-case-v2"
MAX_FILE_BYTES = 16 * 1024 * 1024
MAX_INPUT_BYTES = 64 * 1024 * 1024
MAX_ARCHIVE_BYTES = 64 * 1024 * 1024
SOURCE_FILES = (
    "adrive/navmesh.py", "adrive/road_corridor.py", "adrive/data.py",
    "adrive/models.py", "adrive/planner.py",
)

# Pinned to the saved screenshot case, not runtime/route-quality/last-plan.json.
BASELINE = json.loads(r'''
{
  "case": "westfall-map0-20261029T032323",
  "source_manifest_sha256": "47a21c1b6845fa8cf74546d532959366eaad0da7b0de04c95cc91add1ec77c69",
  "source_plan_sha256": "18bc407b2c9d699b808b07fb6cdd48fd45c78028a7ec53ab21962cd5b317dbb6",
  "manifest_metadata": {
    "schema": 1,
    "kind": "adrive-ground-navmesh",
    "mmap_version": 16,
    "client": {
      "version": "1.60.1.70658",
      "product": "wowf",
      "build_key": "165410aca770316ea0f88100bb4830c2",
      "build_info_sha256": "a33feb82421b4e8a67cedbec8f3bf19b5b2cdaa2c14de5d357dd71d7e013d3b3"
    },
    "settings": {
      "cell_size_yards": 0.2666666,
      "agent_radius_yards": 0.5333332,
      "agent_height_yards": 1.5999996,
      "walkable_climb_yards": 0.5333332,
      "max_slope_degrees": 50,
      "offmesh_connections": false,
      "water": false,
      "dynamic_obstacles": false
    },
    "identity": "9e09d71b0068bb29",
    "library": {
      "path": "adrive_navmesh.dll",
      "bytes": 169472,
      "sha256": "fc5191c9d63c4e4b3c917ef4e522ffd6f85c01520df881583f881acbe30d56ed"
    }
  },
  "route": {
    "start": {
      "c": 0,
      "x": -9485.03509172938,
      "y": 73.4356325318914,
      "z": null
    },
    "goal": {
      "c": 0,
      "x": -11031.469938793904,
      "y": 1498.066095269559,
      "z": null
    },
    "started_at_utc": "2026-10-29T03:23:23.919235+00:00",
    "planner_version": "road-corridor-v1",
    "max_expansions": 500000
  },
  "map_header": {
    "path": "1.60.1.70658-20261028T203512Z/mmaps/0000.mmap",
    "bytes": 40,
    "sha256": "11f29fcf6940331baab4221832dd3c60cafd7b8946d3f7503bbfaffc66695226",
    "c": 0
  },
  "capture_keys": [
    [
      0,
      48,
      30
    ],
    [
      0,
      49,
      30
    ],
    [
      0,
      50,
      30
    ],
    [
      0,
      51,
      28
    ],
    [
      0,
      51,
      29
    ],
    [
      0,
      51,
      30
    ],
    [
      0,
      52,
      28
    ],
    [
      0,
      52,
      29
    ],
    [
      0,
      52,
      30
    ],
    [
      0,
      53,
      28
    ],
    [
      0,
      53,
      29
    ],
    [
      0,
      53,
      30
    ]
  ],
  "tiles": [
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_48_30.mmtile",
      "bytes": 2210536,
      "sha256": "9012c69529815a6a8dec918487db3b71aec0ae3781914dd057551a085bdd1f61",
      "c": 0,
      "row": 48,
      "col": 30,
      "mesh_x": 15,
      "mesh_y": 13
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_48_31.mmtile",
      "bytes": 1417260,
      "sha256": "88001fb075f3e43e2b55444693ea9c905a688e0293373c65809a166738a63ce6",
      "c": 0,
      "row": 48,
      "col": 31,
      "mesh_x": 14,
      "mesh_y": 13
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_48_32.mmtile",
      "bytes": 1256884,
      "sha256": "a46c2aa4fc0aa8f3eda05ed5687838c6da58b598f89ca43b93dfb0110a40c739",
      "c": 0,
      "row": 48,
      "col": 32,
      "mesh_x": 13,
      "mesh_y": 13
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_48_33.mmtile",
      "bytes": 945052,
      "sha256": "cf0df1beea064fdcacfdb8289571243b9848da7a956d37a5c96af1857dec753a",
      "c": 0,
      "row": 48,
      "col": 33,
      "mesh_x": 12,
      "mesh_y": 13
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_48_34.mmtile",
      "bytes": 1151300,
      "sha256": "f9ee42f36096fd15f94d13f1fff9df2fdb44b04afb1da94cf0b6a7d1d8cc5e07",
      "c": 0,
      "row": 48,
      "col": 34,
      "mesh_x": 11,
      "mesh_y": 13
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_48_35.mmtile",
      "bytes": 1179648,
      "sha256": "d4a1950711047f7fb379397b0cd8be943a2b2352b182414217f25c193add4e9d",
      "c": 0,
      "row": 48,
      "col": 35,
      "mesh_x": 10,
      "mesh_y": 13
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_48_36.mmtile",
      "bytes": 1255288,
      "sha256": "072a63c970e5cf64f77670d09e1a5ed092614086b90d72197889b7408cceeb60",
      "c": 0,
      "row": 48,
      "col": 36,
      "mesh_x": 9,
      "mesh_y": 13
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_49_30.mmtile",
      "bytes": 1117520,
      "sha256": "115e52fa4aade5fbf757a8276141217f1aa8c0ddb1f67cb557138190e9665df4",
      "c": 0,
      "row": 49,
      "col": 30,
      "mesh_x": 15,
      "mesh_y": 12
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_49_31.mmtile",
      "bytes": 1140828,
      "sha256": "608bea8bb6a268c922b90923109b10f8efa4aee00a2cdb5131c6ad7dc474e429",
      "c": 0,
      "row": 49,
      "col": 31,
      "mesh_x": 14,
      "mesh_y": 12
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_49_32.mmtile",
      "bytes": 1081168,
      "sha256": "87c3039aacca9e5aa0e2c10619945fb60e00beda3e2e7dc77ee90b36403e8b66",
      "c": 0,
      "row": 49,
      "col": 32,
      "mesh_x": 13,
      "mesh_y": 12
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_49_33.mmtile",
      "bytes": 923280,
      "sha256": "fcd53183d656eb895ca360734e3798bd29ad5b255e4b86d308b644f454e1c595",
      "c": 0,
      "row": 49,
      "col": 33,
      "mesh_x": 12,
      "mesh_y": 12
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_49_34.mmtile",
      "bytes": 1090716,
      "sha256": "6cdcb1b6bb4384778b2a2b2bc115a07aab840e79f2780e1cf38f0111f5f574d8",
      "c": 0,
      "row": 49,
      "col": 34,
      "mesh_x": 11,
      "mesh_y": 12
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_49_35.mmtile",
      "bytes": 1132908,
      "sha256": "81fb70658118ff692ea3600577674542ee8e953b9129c37c85556e5631aae553",
      "c": 0,
      "row": 49,
      "col": 35,
      "mesh_x": 10,
      "mesh_y": 12
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_49_36.mmtile",
      "bytes": 935060,
      "sha256": "c71fdee9c7ef59dfe212085e1b7a7ac2382ae37738181d813367f5dd87ca4b99",
      "c": 0,
      "row": 49,
      "col": 36,
      "mesh_x": 9,
      "mesh_y": 12
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_50_30.mmtile",
      "bytes": 872108,
      "sha256": "bc9c9bdb64b9e75aaa1aea37df0858f5f40d1716acbc1409ed93b38c427893eb",
      "c": 0,
      "row": 50,
      "col": 30,
      "mesh_x": 15,
      "mesh_y": 11
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_50_31.mmtile",
      "bytes": 951284,
      "sha256": "7df825c6ea1f50e9cbdb97f5381b04b7a31fb44e497eb8bc7ba2195e6cedb5cb",
      "c": 0,
      "row": 50,
      "col": 31,
      "mesh_x": 14,
      "mesh_y": 11
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_50_32.mmtile",
      "bytes": 803972,
      "sha256": "0ae228bc4ee8748fb93718206258655e9fdc68d7b145ebe3c2cbc7aec5c2f73d",
      "c": 0,
      "row": 50,
      "col": 32,
      "mesh_x": 13,
      "mesh_y": 11
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_50_33.mmtile",
      "bytes": 675296,
      "sha256": "68c8d764acd38e3afa5435dfd26cc2c81ddcfaa226a6581353bd72eb50e5f9b8",
      "c": 0,
      "row": 50,
      "col": 33,
      "mesh_x": 12,
      "mesh_y": 11
    },
    {
      "path": "repairs/bridge-0000-50-34-20261029T012545Z-53263d6f/0000_50_34.mmtile",
      "bytes": 888768,
      "sha256": "74dd28d4d559518bca84b678ac3fb3ccd5ba294c8ab65bebcede927acc46df1c",
      "c": 0,
      "row": 50,
      "col": 34,
      "mesh_x": 11,
      "mesh_y": 11
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_50_35.mmtile",
      "bytes": 1007408,
      "sha256": "f4fea35c469950cc98bc6d11ca342fc21f9bcc80fa22a92720635264d1bbccce",
      "c": 0,
      "row": 50,
      "col": 35,
      "mesh_x": 10,
      "mesh_y": 11
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_50_36.mmtile",
      "bytes": 1085976,
      "sha256": "5fe25ba8c9373af569c4692cf4a564cdcba823cc70ff079295d3d8cdc88e8138",
      "c": 0,
      "row": 50,
      "col": 36,
      "mesh_x": 9,
      "mesh_y": 11
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_51_28.mmtile",
      "bytes": 621384,
      "sha256": "5ebef83974bf04d100fbcdd1fa8251af9c4ad182aca1ecd4f2f1601adf76a0d1",
      "c": 0,
      "row": 51,
      "col": 28,
      "mesh_x": 17,
      "mesh_y": 10
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_51_29.mmtile",
      "bytes": 487860,
      "sha256": "b3b8d82b1bd04896d490471f09df5e78103b8945bde9c67402b369eb7bbbe973",
      "c": 0,
      "row": 51,
      "col": 29,
      "mesh_x": 16,
      "mesh_y": 10
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_51_30.mmtile",
      "bytes": 550172,
      "sha256": "ced0fe2973ccc39b4f19fe2bc394d0ab02eba52a27befc0af2fa899357d13e70",
      "c": 0,
      "row": 51,
      "col": 30,
      "mesh_x": 15,
      "mesh_y": 10
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_51_31.mmtile",
      "bytes": 852636,
      "sha256": "bbb102f1e5560b770c73700645ce824d1e37dff2a1c9b4f8c998f036bb8c1a87",
      "c": 0,
      "row": 51,
      "col": 31,
      "mesh_x": 14,
      "mesh_y": 10
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_51_32.mmtile",
      "bytes": 1187172,
      "sha256": "9bf107eb61c02a79aafb135e22a6a23ebc30c7779b81d2c1cb0c096981fa1ff4",
      "c": 0,
      "row": 51,
      "col": 32,
      "mesh_x": 13,
      "mesh_y": 10
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_51_33.mmtile",
      "bytes": 971148,
      "sha256": "3581469932e50f42e1d9bf91bc8b139c33740bddab1a659c512940365df1fc54",
      "c": 0,
      "row": 51,
      "col": 33,
      "mesh_x": 12,
      "mesh_y": 10
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_51_34.mmtile",
      "bytes": 1630484,
      "sha256": "9aef32c41b7042e4150f9ca3ffce222930ff0d9ccd28ec18421141157d732469",
      "c": 0,
      "row": 51,
      "col": 34,
      "mesh_x": 11,
      "mesh_y": 10
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_51_35.mmtile",
      "bytes": 1359336,
      "sha256": "15a29886169197bf4ac54ce472887e8bcff4d45475c29be260540e7bfeb429fb",
      "c": 0,
      "row": 51,
      "col": 35,
      "mesh_x": 10,
      "mesh_y": 10
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_51_36.mmtile",
      "bytes": 1406520,
      "sha256": "3637af5430e8925732f5006e8dd17f06703d3ad118d7f6a4e836ecc32aed7dad",
      "c": 0,
      "row": 51,
      "col": 36,
      "mesh_x": 9,
      "mesh_y": 10
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_52_28.mmtile",
      "bytes": 557620,
      "sha256": "ae755c1c979dbff926f4ecdd6da8f66ef60d4ca3d79c376fd0cfb2eee6780656",
      "c": 0,
      "row": 52,
      "col": 28,
      "mesh_x": 17,
      "mesh_y": 9
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_52_29.mmtile",
      "bytes": 890824,
      "sha256": "b9ce956a2ed50c2e404563ae49951ba610355fa4b3122c4552dd007828a93903",
      "c": 0,
      "row": 52,
      "col": 29,
      "mesh_x": 16,
      "mesh_y": 9
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_52_30.mmtile",
      "bytes": 572580,
      "sha256": "13aac3fbb3bdd7782c1bd76f31f52d63dbb6d5713825ae3cc87d56292508531d",
      "c": 0,
      "row": 52,
      "col": 30,
      "mesh_x": 15,
      "mesh_y": 9
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_52_31.mmtile",
      "bytes": 1050624,
      "sha256": "2ab855d47cdff9b6b08b1459a7019591724ba538b30a2028dbe6228b1abea004",
      "c": 0,
      "row": 52,
      "col": 31,
      "mesh_x": 14,
      "mesh_y": 9
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_52_32.mmtile",
      "bytes": 1357888,
      "sha256": "6677f424ab6fa3ff8a0878113dda47916bbea92305f2d2d1278db61a2349e0fb",
      "c": 0,
      "row": 52,
      "col": 32,
      "mesh_x": 13,
      "mesh_y": 9
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_52_33.mmtile",
      "bytes": 1244468,
      "sha256": "4c638d1fe4387dd5a0723faefb86754443bef267016acc1122ee5bb0061a331f",
      "c": 0,
      "row": 52,
      "col": 33,
      "mesh_x": 12,
      "mesh_y": 9
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_52_34.mmtile",
      "bytes": 1268960,
      "sha256": "257a8dd0fb0cdbe9f55a25618525cb750d93f8489539fce45063a4c0fdc7c0c4",
      "c": 0,
      "row": 52,
      "col": 34,
      "mesh_x": 11,
      "mesh_y": 9
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_52_35.mmtile",
      "bytes": 1939612,
      "sha256": "d861c7f2af162434e90406eff553a438ebd6059857cf660e22ef6b2e12fd67b9",
      "c": 0,
      "row": 52,
      "col": 35,
      "mesh_x": 10,
      "mesh_y": 9
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_53_28.mmtile",
      "bytes": 448948,
      "sha256": "a9b68b91694203aa224e89d414c5afe24caca553d5720e9e7918ce5dde8de09a",
      "c": 0,
      "row": 53,
      "col": 28,
      "mesh_x": 17,
      "mesh_y": 8
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_53_29.mmtile",
      "bytes": 1137632,
      "sha256": "e8cbe27643327ddb9c6bfb7cc999a73f89287b39dd9af056cb7652fd3d1e80a6",
      "c": 0,
      "row": 53,
      "col": 29,
      "mesh_x": 16,
      "mesh_y": 8
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_53_30.mmtile",
      "bytes": 1297344,
      "sha256": "cd9d2b4bb5970fb0789c5e24e21d40fe136800da9cc25081504a967076ecf73e",
      "c": 0,
      "row": 53,
      "col": 30,
      "mesh_x": 15,
      "mesh_y": 8
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_53_31.mmtile",
      "bytes": 1166520,
      "sha256": "96a20844c93ed3d81eba6050a9db9ca6df6c32c3e47526ad9838aadf9accf7de",
      "c": 0,
      "row": 53,
      "col": 31,
      "mesh_x": 14,
      "mesh_y": 8
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_53_32.mmtile",
      "bytes": 1342908,
      "sha256": "60a61fe00694ff82f6eeddb0848c60b4bd4230e8f5f081f2e2bff24aa209c17d",
      "c": 0,
      "row": 53,
      "col": 32,
      "mesh_x": 13,
      "mesh_y": 8
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_53_33.mmtile",
      "bytes": 1536936,
      "sha256": "4892fda8be9fc52d898cec804666ca95687037e1f148443d3a0299d52597f034",
      "c": 0,
      "row": 53,
      "col": 33,
      "mesh_x": 12,
      "mesh_y": 8
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_53_34.mmtile",
      "bytes": 1351960,
      "sha256": "e9e9f8bdcadc6b061d05c67b9a167276229a7cf3aa339a907325ebbcc8b70cde",
      "c": 0,
      "row": 53,
      "col": 34,
      "mesh_x": 11,
      "mesh_y": 8
    },
    {
      "path": "1.60.1.70658-20261028T203512Z/mmaps/0000_53_35.mmtile",
      "bytes": 1254948,
      "sha256": "2435f1f42144637cba83f97e71fad95c438ad7537c17de25c3752e115619a8ed",
      "c": 0,
      "row": 53,
      "col": 35,
      "mesh_x": 10,
      "mesh_y": 8
    }
  ]
}
''')


def say(message):
    print(message, flush=True)


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, indent=2) + "\n").encode("utf-8")


def digest(data):
    return hashlib.sha256(data).hexdigest()


def checked_relative(name):
    if not isinstance(name, str) or not name or "\x00" in name:
        raise ValueError("Missing or invalid relative path")
    name = name.replace("\\", "/")
    posix, windows = PurePosixPath(name), PureWindowsPath(name)
    if (posix.is_absolute() or windows.is_absolute() or windows.drive
            or any(p in ("", ".", "..") or ":" in p for p in name.split("/"))):
        raise ValueError("Absolute, parent or special path refused: " + name)
    return posix


def contained(base, name, *, exists=True):
    base = base.resolve(strict=True)
    relative = checked_relative(name)
    path = base.joinpath(*relative.parts).resolve(strict=exists)
    try:
        path.relative_to(base)
    except ValueError:
        raise ValueError("Path or link leaves its allowed root: " + str(name)) from None
    return path


def signature(info):
    return (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns)


def read_snapshot(base, relative, limit, *, keep):
    path = contained(base, relative)
    path_before = path.stat()
    with path.open("rb") as stream:
        handle_before = os.fstat(stream.fileno())
        if not stat.S_ISREG(handle_before.st_mode):
            raise ValueError("Input is not a regular file")
        if handle_before.st_size > limit:
            raise ValueError("Input exceeds the remaining byte limit")
        hasher, parts, count = hashlib.sha256(), [], 0
        while True:
            block = stream.read(min(1024 * 1024, limit + 1 - count))
            if not block:
                break
            count += len(block)
            if count > limit:
                raise ValueError("Input grew past the byte limit")
            hasher.update(block)
            if keep:
                parts.append(block)
        handle_after = os.fstat(stream.fileno())
    checked = contained(base, relative)
    path_after = checked.stat()
    # Windows path and handle APIs may expose different dev/ino/time values for
    # the same unchanged file. Compare each API only with its own earlier read.
    if (checked != path or signature(path_before) != signature(path_after)
            or signature(handle_before) != signature(handle_after)
            or count != handle_before.st_size):
        raise ValueError("Input changed during its read")
    snapshot = {
        "resolved_path": str(path), "bytes": count, "sha256": hasher.hexdigest(),
        "path_stat_signature": list(signature(path_after)),
        "handle_stat_signature": list(signature(handle_after)),
    }
    return snapshot, b"".join(parts) if keep else None


def matches_metadata(observed, expected):
    return (observed.get("bytes") == expected.get("bytes")
            and observed.get("sha256") == expected.get("sha256"))


def find_unique(rows, key, fields):
    if not isinstance(rows, list):
        raise ValueError("Manifest entry list is missing or invalid")
    found = [row for row in rows if isinstance(row, dict)
             and tuple(row.get(field) for field in fields) == tuple(key)]
    if len(found) != 1:
        raise ValueError("Manifest must contain exactly one entry for " + str(key))
    return found[0]


def collect(root, *, reference=None, progress=say):
    reference = BASELINE if reference is None else reference
    root = Path(root).resolve(strict=True)
    if not root.is_dir():
        raise ValueError("Project root is not a directory")
    runtime = contained(root, "runtime", exists=False)
    runtime.mkdir(exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = runtime / ("westfall-case-" + stamp + "-" + uuid.uuid4().hex[:8])
    out.mkdir(exist_ok=False)
    progress("OUTPUT = " + str(out))
    inventory, members, warnings = [], {}, []
    bytes_read = 0
    bytes_captured = 0
    watches = []

    def record(label, base, relative, member=None, expected=None, required=True):
        nonlocal bytes_read, bytes_captured
        progress("READ " + label + " : " + str(relative))
        item = {
            "label": label, "relative_path": relative, "required": required,
            "archive_member": member, "included": False, "stable": False,
        }
        inventory.append(item)
        try:
            allowance = min(MAX_FILE_BYTES, MAX_INPUT_BYTES - bytes_read)
            if member:
                allowance = min(allowance, MAX_ARCHIVE_BYTES - bytes_captured - 1024 * 1024)
            if allowance < 0:
                raise ValueError("Collection byte budget exhausted")
            snap, data = read_snapshot(base, relative, allowance, keep=bool(member))
            bytes_read += snap["bytes"]
            item.update(snap)
            item["read_ok"] = True
            if expected is not None:
                item["expected"] = expected
                item["matches_expected"] = matches_metadata(item, expected)
                if not item["matches_expected"]:
                    warnings.append(label + ": bytes/SHA differ from the screenshot baseline")
            watches.append((item, base, relative))
            if member:
                members[member] = data
                bytes_captured += len(data)
                item["included"] = True
            return item, data
        except (OSError, ValueError, RuntimeError) as error:
            item["read_ok"] = False
            item["error"] = str(error)
            warnings.append(label + ": " + str(error))
            progress("  UNAVAILABLE: " + str(error))
            return item, None

    progress("STAGE 1/4: Capture the manifest and check the fixed reference metadata.")
    manifest_record, manifest_data = record(
        "manifest", root, "data/navigation/manifest.json", "manifest.json")
    manifest, metadata_errors = {}, []
    if manifest_data is not None:
        try:
            manifest = json.loads(manifest_data.decode("utf-8-sig"))
            if not isinstance(manifest, dict):
                raise ValueError("Manifest is not a JSON object")
        except (UnicodeError, ValueError) as error:
            metadata_errors.append("Manifest cannot be parsed: " + str(error))
            manifest = {}
    else:
        metadata_errors.append("Manifest is unavailable")
    metadata_checks = []
    for key, value in reference["manifest_metadata"].items():
        matched = manifest.get(key) == value
        metadata_checks.append({"field": key, "matches": matched, "required_for_map0": key != "identity",
                                "expected": value, "observed": manifest.get(key)})
        if not matched and key != "identity":
            metadata_errors.append("Manifest field differs: " + key)
    indexed_tiles = manifest.get("tiles")
    computed_identity = (digest(json.dumps(indexed_tiles, sort_keys=True).encode())[:16]
                         if isinstance(indexed_tiles, list) else None)
    identity_self_consistent = (isinstance(manifest.get("identity"), str)
                                and manifest["identity"] == computed_identity)
    if not identity_self_consistent:
        metadata_errors.append("Current navigation identity does not match its full tile index")
    exact_manifest_match = (manifest_data is not None
                            and digest(manifest_data) == reference["source_manifest_sha256"])
    if manifest_data is not None and not exact_manifest_match:
        note = ("Current manifest differs from the saved screenshot; all 46 fixed map0 inputs "
                "will be checked independently. This historical difference alone does not mean missing evidence.")
        warnings.append(note)
        progress("NOTE: " + note)
    try:
        navroot = contained(root, "data/navigation")
    except (OSError, ValueError, RuntimeError) as error:
        navroot = None
        metadata_errors.append("Navigation root unavailable: " + str(error))

    progress("STAGE 2/4: Verify 46 fixed tiles; archive only the 12 missing here.")
    capture_keys = {tuple(key) for key in reference["capture_keys"]}
    tile_records, tile_metadata_checks = [], []
    for expected in reference["tiles"]:
        key = (expected["c"], expected["row"], expected["col"])
        label = "tile:%d/%d/%d" % key
        member = ("mmaps/%04d_%02d_%02d.mmtile" % key) if key in capture_keys else None
        try:
            current = find_unique(manifest.get("tiles"), key, ("c", "row", "col"))
            matched = (matches_metadata(current, expected)
                       and current.get("mesh_x") == expected.get("mesh_x")
                       and current.get("mesh_y") == expected.get("mesh_y"))
            tile_metadata_checks.append({
                "key": list(key), "matches": matched,
                "path_matches": current.get("path") == expected["path"],
                "current": current, "expected": expected,
            })
            if not matched:
                metadata_errors.append(label + ": manifest bytes/hash/grid differ")
            if navroot is None:
                raise ValueError("Navigation root unavailable")
            item, _ = record(label, navroot, current.get("path"), member, expected)
        except (OSError, ValueError, RuntimeError) as error:
            message = label + ": " + str(error)
            metadata_errors.append(message)
            progress("UNAVAILABLE " + message)
            item = {"label": label, "required": True, "read_ok": False,
                    "archive_member": member, "included": False,
                    "stable": False, "error": str(error)}
            inventory.append(item)
        tile_records.append(item)

    header_record = None
    try:
        current_header = find_unique(manifest.get("maps"), (0,), ("c",))
        if not matches_metadata(current_header, reference["map_header"]):
            metadata_errors.append("Map0 header manifest bytes/hash differ")
        if navroot is None:
            raise ValueError("Navigation root unavailable")
        header_record, _ = record("map0_header", navroot, current_header.get("path"),
                                  "mmaps/0000.mmap", reference["map_header"])
    except (OSError, ValueError, RuntimeError) as error:
        metadata_errors.append("Map0 header: " + str(error))

    progress("STAGE 3/4: Copy five planner sources as data and optional current last-plan.")
    source_records = []
    for name in SOURCE_FILES:
        item, _ = record("source:" + name, root, name, name)
        source_records.append(item)
    latest_record, latest_data = record(
        "current_last_plan", root, "runtime/route-quality/last-plan.json",
        "current-last-plan.json", required=False)
    latest = {
        "optional": True, "used_to_select_targets": False,
        "note": "This is the current log; it may describe a different route than the saved screenshot.",
        "available": latest_data is not None,
    }
    if latest_data is not None:
        try:
            current_plan = json.loads(latest_data.decode("utf-8-sig"))
            latest.update({key: current_plan.get(key) for key in
                           ("started_at_utc", "planner_version", "navigation_identity", "start", "goal")})
            latest["matches_reference_endpoints"] = all(
                current_plan.get(key) == reference["route"][key] for key in ("start", "goal"))
            latest["is_exact_saved_log"] = digest(latest_data) == reference["source_plan_sha256"]
        except (UnicodeError, ValueError, AttributeError) as error:
            latest["parse_error"] = str(error)

    progress("STAGE 4/4: Recheck input hashes and write the diagnostic ZIP.")
    for item, base, relative in watches:
        progress("RECHECK " + item["label"])
        try:
            after, _ = read_snapshot(base, relative, MAX_FILE_BYTES, keep=False)
            item["after"] = after
            item["stable"] = all(item[key] == after[key] for key in
                                 ("resolved_path", "bytes", "sha256",
                                  "path_stat_signature", "handle_stat_signature"))
            if not item["stable"]:
                warnings.append(item["label"] + ": input changed after the first snapshot")
        except (OSError, ValueError, RuntimeError) as error:
            item["stable"] = False
            item["recheck_error"] = str(error)
            warnings.append(item["label"] + ": recheck failed: " + str(error))

    def valid(item, expected=False):
        return bool(item and item.get("read_ok") and item.get("stable")
                    and (not expected or item.get("matches_expected")))

    required_members = {"manifest.json", "mmaps/0000.mmap", *SOURCE_FILES}
    required_members.update("mmaps/%04d_%02d_%02d.mmtile" % key for key in capture_keys)
    map0_inputs_match = (not metadata_errors and valid(manifest_record)
                        and valid(header_record, True)
                        and all(valid(item, True) for item in tile_records))
    baseline_matches = exact_manifest_match and map0_inputs_match
    required_inputs_complete = (valid(manifest_record) and valid(header_record, True)
                               and all(valid(item, True) for item in tile_records)
                               and all(valid(item) for item in source_records))
    snapshots_stable = all(item.get("stable") for item, _, _ in watches)
    required_members_present = required_members <= members.keys()
    complete = (map0_inputs_match and required_inputs_complete and snapshots_stable
                and required_members_present)
    summary = {
        "schema": 2, "collector": COLLECTOR_VERSION, "created_at_utc": stamp,
        "project_root": str(root), "output_directory": str(out),
        "path": str(out / "westfall-case.zip"),
        "collection_complete": complete,
        "exact_screenshot_manifest_match": exact_manifest_match,
        "required_map0_nav_inputs_match": map0_inputs_match,
        "navigation_identity_self_consistent": identity_self_consistent,
        "computed_navigation_identity": computed_identity,
        "baseline_matches_screenshot": baseline_matches,
        "baseline_scope": "baseline_matches_screenshot additionally requires the exact saved manifest bytes. "
                          "required_map0_nav_inputs_match requires a self-consistent current identity, "
                          "the captured client/settings/library metadata, map0 header and all 46 tile bytes/hashes. "
                          "A change elsewhere in the full navigation index is recorded, not assumed to change map0. "
                          "Current source files are captured, but their equivalence to the screenshot's "
                          "executed code is not established.",
        "fixed_route": reference["route"],
        "expected_navigation_identity": reference["manifest_metadata"]["identity"],
        "observed_navigation_identity": manifest.get("identity"),
        "verified_tile_count": sum(valid(item, True) for item in tile_records),
        "required_tile_count": len(reference["tiles"]),
        "requested_capture_keys": reference["capture_keys"],
        "captured_tile_count": sum(item.get("included", False) for item in tile_records),
        "required_capture_count": len(capture_keys),
        "required_inputs_complete": required_inputs_complete,
        "input_snapshots_stable": snapshots_stable,
        "required_members_present": required_members_present,
        "missing_archive_members": sorted(required_members - members.keys()),
        "manifest_metadata_checks": metadata_checks,
        "tile_metadata_checks": tile_metadata_checks,
        "metadata_errors": metadata_errors,
        "latest_plan": latest,
        "warnings": warnings,
        "first_pass_bytes_read": bytes_read,
        "captured_input_bytes": bytes_captured,
        "limits": {"per_file": MAX_FILE_BYTES, "first_pass_inputs": MAX_INPUT_BYTES,
                   "archive_uncompressed": MAX_ARCHIVE_BYTES},
    }
    members["reference-case.json"] = encoded(reference)
    members["inventory.json"] = encoded(inventory)
    members["summary.json"] = encoded(summary)
    members["README.txt"] = (
        "Fixed Westfall map0 route evidence collection.\n"
        "Only the 12 requested tile files are archived; all 46 reference tile files were checked.\n"
        "See summary.json for baseline match, completeness, missing files and metadata differences.\n"
        "Historical manifest equality is separate from the independently verified required map0 inputs.\n"
        "The current navigation identity must match the full current tile index.\n"
        "inventory.json records original paths, bytes/SHA256 and before/after stability.\n"
        "Path stat and handle fstat signatures are each checked against the same API, never against each other.\n"
        "Current planner .py files are evidence only; this collector never executes them.\n"
        "current-last-plan.json is optional and may describe a later, different route.\n"
        "The fixed reference-case.json determines this collection's route and targets.\n"
        "READY means this bounded evidence collection is complete; it is not a navigation fix.\n"
        "No WSL, DLL loading, generation, active file update or activation is performed.\n"
    ).encode("utf-8")
    if sum(map(len, members.values())) > MAX_ARCHIVE_BYTES:
        raise ValueError("Diagnostic report exceeds archive budget; input files were not modified")
    temporary = out / "westfall-case.zip.partial"
    with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for name, data in members.items():
            progress("PACK " + name)
            archive.writestr(name, data)
    # Check every archived byte against the in-memory snapshot, including reports.
    with zipfile.ZipFile(temporary, "r") as archive:
        if set(archive.namelist()) != set(members):
            raise ValueError("Archive member verification failed")
        for name, data in members.items():
            packed = archive.read(name)
            if len(packed) != len(data) or digest(packed) != digest(data):
                raise ValueError("Archive checksum verification failed: " + name)
    os.replace(temporary, out / "westfall-case.zip")
    (out / "summary.json").write_bytes(members["summary.json"])
    (out / "inventory.json").write_bytes(members["inventory.json"])
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", "--project-root", default=DEFAULT_ROOT,
                        help="AutonomousDriving project directory")
    args = parser.parse_args(argv)
    say("WESTFALL_CASE " + COLLECTOR_VERSION + ": starting fixed map0 route evidence collection.")
    try:
        result = collect(args.root)
    except (OSError, ValueError, RuntimeError, zipfile.BadZipFile) as error:
        say("WESTFALL_CASE_FAILED: " + str(error))
        return 2
    say("WESTFALL_CASE_READY" if result["collection_complete"] else "WESTFALL_CASE_INCOMPLETE")
    say("MAP0_INPUTS_MATCH = " + str(result["required_map0_nav_inputs_match"]))
    say("TILES = %d/%d captured; %d/%d verified" % (
        result["captured_tile_count"], result["required_capture_count"],
        result["verified_tile_count"], result["required_tile_count"]))
    say("ZIP = " + result["path"])
    say("Upload this ZIP. Missing or changed evidence is recorded inside it.")
    return 0 if result["collection_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
