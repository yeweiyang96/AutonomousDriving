"""Collect one navigation failure for offline reproduction; never run its DLL."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import sys
import tempfile
import uuid
import xml.etree.ElementTree as ET
import zipfile

MAX_FILE_BYTES = 32 * 1024 * 1024
MAX_TOTAL_BYTES = 128 * 1024 * 1024
START = (-10065.478503779887, -1507.212281364077, 28.680936813354492)
GOAL = (-10079.181030588508, -1468.9011349807895, 28.680936813354492)
EXPECTED_DLL_SHA = 'fc5191c9d63c4e4b3c917ef4e522ffd6f85c01520df881583f881acbe30d56ed'
OBSERVED = {
    'source': 'native.ad_path', 'rc': 6, 'native_reason': 'node_pool_exhausted',
    'native_reason_code': 2,
    'native_status': '0x40000020', 'stage': 'fallback_path', 'radius': 2,
    'plan_budget': 500000, 'expanded_before': 16668, 'passed_budget': 483332,
    'call_expanded': 65535, 'output_count': 0, 'output_capacity': 65535,
    'continent': 0, 'loaded_tiles': 49, 'loaded_bytes': 56694268,
    'max_bytes': 402653184, 'start': list(START), 'goal': list(GOAL),
}
SOURCE_FILES = (
    'native/navmesh/bridge.cpp', 'native/navmesh/CMakeLists.txt',
    'native/navmesh/fixture.cpp', 'adrive/navmesh.py', 'adrive/models.py',
    'adrive/data.py', 'adrive/__init__.py',
)
SOURCE_SUFFIXES = {'.h', '.hpp', '.hxx', '.hh', '.inl', '.inc', '.c', '.cc', '.cpp', '.cxx'}
README = """# Navigation failure reproduction subset

This archive contains a partial project, the original navigation manifest,
both map headers, the selected continent-0 tiles, and the active query DLL.
It does not contain all assets named by the original manifest. Do not publish
it as a complete navigation pack or run a full-pack validator on it.

case.json separates the user's observed results from data collected now.
The collector did not load the DLL, run pathfinding, build code, or access the
game installation, WSL, account data, or runtime configuration.

Reproduce with guide=None and blocked_edges empty, using the exact coordinates.
Load base tiles in the recorded order, run radius-1 search, then load only the
recorded additional tiles in order and run radius-2 search with its remaining
budget. Do not load all tiles at once. Do not run radius 4: its additional
tiles were not collected. A radius-1 no_route does not prove a bridge is broken.

Keep the coordinate conversion and float32 conversion used by navmesh.py and
bridge.cpp. The DLL is included for inspection or a separate Windows replay;
the collector never executes it. Sources can be built separately for analysis.
compile-summary.json contains only selected project XML properties; imported
MSBuild properties and inherited options have NOT been expanded.

inventory.json lists the SHA256 and length of every member except itself.
"""


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _json(value):
    return (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode('utf-8')


def _relative(name):
    name = str(name).replace('\\', '/')
    if not name or any(p in ('', '.', '..') or ':' in p for p in name.split('/')):
        raise ValueError(f'不安全的相对路径：{name}')
    path = PurePosixPath(name)
    if path.is_absolute():
        raise ValueError(f'不允许绝对路径：{name}')
    return path.as_posix()


def _inside_file(root, relative):
    relative = _relative(relative)
    path = root.joinpath(*PurePosixPath(relative).parts).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError(f'文件缺失或超出允许目录：{relative}')
    return path


def _read(path):
    if path.stat().st_size > MAX_FILE_BYTES:
        raise ValueError(f'单文件超过 32 MiB：{path.name}')
    with path.open('rb') as stream:
        data = stream.read(MAX_FILE_BYTES + 1)
    if len(data) > MAX_FILE_BYTES:
        raise ValueError(f'读取中的文件超过 32 MiB：{path.name}')
    return data


def _load_project_api(root):
    """Import definitions only; do not construct NavMeshPack/Native or load a DLL."""
    names = {'adrive': 'adrive/__init__.py', 'adrive.navmesh': 'adrive/navmesh.py',
             'adrive.models': 'adrive/models.py', 'adrive.data': 'adrive/data.py'}
    for name, relative in names.items():
        existing = sys.modules.get(name)
        if existing is not None and Path(existing.__file__).resolve() != _inside_file(root, relative):
            raise RuntimeError('解释器已导入另一份 adrive；请用新 Python 进程运行采集器。')
    old_path, old_bytecode = list(sys.path), sys.dont_write_bytecode
    try:
        sys.path.insert(0, str(root))
        sys.dont_write_bytecode = True
        nav = importlib.import_module('adrive.navmesh')
        models = importlib.import_module('adrive.models')
        for name, relative in names.items():
            if Path(sys.modules[name].__file__).resolve() != _inside_file(root, relative):
                raise RuntimeError(f'导入的模块不属于指定项目：{name}')
        return nav.NavMeshPack, models.Point, nav.TILE_SIZE, nav.WORLD_TOP
    finally:
        sys.path[:] = old_path
        sys.dont_write_bytecode = old_bytecode


def _compile_summary(root):
    relative = 'runtime/navmesh-build/adrive_navmesh.vcxproj'
    if not (root / relative).exists():
        return {'source': relative, 'available': False, 'inherited_options_expanded': False}
    data = _read(_inside_file(root, relative))
    document = ET.fromstring(data)
    properties = []

    def walk(node, conditions=()):
        condition = node.get('Condition')
        conditions = conditions + ((condition,) if condition else ())
        unresolved = False
        for text in conditions:
            substituted = text.replace('$(Configuration)', 'Release').replace('$(Platform)', 'x64')
            match = re.fullmatch(r'''\s*['"](.*?)['"]\s*==\s*['"](.*?)['"]\s*''', substituted)
            if match and '$(' not in substituted:
                if match[1].casefold() != match[2].casefold():
                    return
            else:
                unresolved = True
        tag = node.tag.rsplit('}', 1)[-1]
        if tag == 'ClCompile':
            for child in node:
                if child.text and child.text.strip():
                    properties.append({'name': child.tag.rsplit('}', 1)[-1],
                                       'scope': 'per_file' if node.get('Include') else 'configuration_default',
                                       'source_file': node.get('Include'),
                                       'value': child.text.strip(), 'conditions': list(conditions),
                                       'property_condition': child.get('Condition'),
                                       'condition_unresolved': unresolved or bool(child.get('Condition'))})
        if tag in ('PlatformToolset', 'WindowsTargetPlatformVersion', 'VCToolsVersion') and node.text:
            properties.append({'name': tag, 'value': node.text.strip(),
                               'conditions': list(conditions), 'condition_unresolved': unresolved})
        for child in node:
            walk(child, conditions)

    walk(document)
    return {'source': relative, 'source_sha256': _sha(data), 'available': True,
            'configuration': 'Release|x64', 'inherited_options_expanded': False,
            'note': 'Raw XML properties only; imports, %(...) and $(...) inheritance are not expanded.',
            'properties': properties}


def collect_case(root: Path, output: Path | None = None) -> dict:
    root = Path(root).resolve()
    if not root.is_dir():
        raise ValueError('项目根目录不存在。')
    manifest = _inside_file(root, 'data/navigation/manifest.json')
    nav_root = manifest.parent
    before = _read(manifest)
    raw = json.loads(before.decode('utf-8-sig'))
    if (raw.get('schema') != 1 or raw.get('kind') != 'adrive-ground-navmesh'
            or raw.get('mmap_version') != 16):
        raise ValueError('导航清单格式不匹配。')
    maps = {entry['c']: entry for entry in raw['maps']}
    tiles = {(t['c'], t['row'], t['col']): t for t in raw['tiles']}
    if set(maps) != {0, 1} or len(maps) != len(raw['maps']) or len(tiles) != len(raw['tiles']):
        raise ValueError('地图索引缺失或重复。')
    if any(c not in (0, 1) or not 0 <= r < 64 or not 0 <= col < 64 for c, r, col in tiles):
        raise ValueError('瓦片索引超出范围。')
    for relative in SOURCE_FILES:
        _inside_file(root, relative)
    pack_class, Point, tile_size, world_top = _load_project_api(root)
    if not math.isfinite(tile_size) or tile_size <= 0 or not math.isfinite(world_top):
        raise ValueError('项目坐标常量无效。')
    start, goal = Point(0, *START), Point(0, *GOAL)
    distance = start.distance(goal)
    steps = max(1, math.ceil(distance / (tile_size / 2)))
    if steps > 4096:
        raise ValueError('本次短路线的采样步数异常；请核对项目坐标常量。')
    base = set()
    for i in range(steps + 1):
        t = i / steps
        base.update(pack_class.around(Point(0, start.x + (goal.x-start.x)*t,
                                           start.y + (goal.y-start.y)*t)))
    if any(len(key) != 2 or not all(isinstance(v, int) and 0 <= v < 64 for v in key) for key in base):
        raise ValueError('NavMeshPack.around 返回的瓦片坐标无效。')
    extra = {(r+dr, c+dc) for r, c in base for dr in range(-2, 3) for dc in range(-2, 3)
             if 0 <= r+dr < 64 and 0 <= c+dc < 64}
    loaded, order, stages = set(), [], []
    for radius, requested, expanded_before, budget, result, expanded in (
            (1, base, 0, 500000, 'no_route', 16668),
            (2, extra, 16668, 483332, 'node_pool_exhausted', 65535)):
        newly, missing = [], []
        for key in sorted(requested):
            if (0, *key) not in tiles:
                missing.append(list(key))
            elif key not in loaded:
                loaded.add(key)
                order.append(key)
                newly.append(list(key))
        stages.append({'radius': radius, 'requested_keys': [list(k) for k in sorted(requested)],
                       'newly_loaded_keys': newly, 'loaded_keys_in_order': [list(k) for k in order],
                       'missing_keys': missing,
                       'observed_query': {'expanded_before': expanded_before, 'passed_budget': budget,
                                          'result': result, 'expanded': expanded,
                                          'evidence': ('Inferred from the radius-2 NAV_DEBUG and guide=None control flow.'
                                                       if radius == 1 else 'Direct user-provided radius-2 NAV_DEBUG.')}})
    tile_bytes = sum(tiles[(0, *key)]['bytes'] for key in order)
    warnings = []
    if len(order) != 49:
        warnings.append(f'当前选择了 {len(order)} 块瓦片，日志记录为 49；不是已确认的同一现场。')
    if tile_bytes != 56694268:
        warnings.append(f'当前瓦片共 {tile_bytes} 字节，日志记录为 56694268；请核对清单版本。')
    if raw['library'].get('sha256') != EXPECTED_DLL_SHA:
        warnings.append('当前活动 DLL 的 SHA256 与报告该错误时的 DLL 不同。')
    compile_summary = _compile_summary(root)
    if not compile_summary['available']:
        warnings.append('未找到指定 vcxproj；没有采集到 Release|x64 编译属性。')

    runtime = (root / 'runtime').resolve()
    if not runtime.is_relative_to(root):
        raise ValueError('runtime 指向项目根目录之外。')
    if output is None:
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        final = runtime / ('navigation-case-' + stamp + '-' + uuid.uuid4().hex[:8] + '.zip')
    else:
        output = Path(output)
        final = (output if output.is_absolute() else root / output).resolve()
    if not final.is_relative_to(runtime) or final.suffix.lower() != '.zip':
        raise ValueError('--output 必须是项目 runtime 内的 .zip 文件。')
    if final.exists():
        raise FileExistsError(f'输出已存在，未覆盖：{final}')
    final.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(prefix='.navigation-case-', suffix='.tmp', dir=str(final.parent))
    temporary = Path(temporary_name)
    inventory, members, total = [], set(), 0
    try:
        with os.fdopen(fd, 'w+b') as stream:
            with zipfile.ZipFile(stream, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
                def add(member, data, role, include_inventory=True):
                    nonlocal total
                    member = _relative(member)
                    if member in members:
                        raise ValueError(f'重复 ZIP 成员：{member}')
                    if len(data) > MAX_FILE_BYTES or total + len(data) > MAX_TOTAL_BYTES:
                        raise ValueError('复现包超过单文件 32 MiB 或总未压缩 128 MiB 限制。')
                    archive.writestr(member, data)
                    members.add(member)
                    total += len(data)
                    if include_inventory:
                        inventory.append({'path': member, 'bytes': len(data), 'sha256': _sha(data), 'role': role})

                def asset(entry, role):
                    relative = _relative(entry['path'])
                    data = _read(_inside_file(nav_root, relative))
                    if len(data) != entry['bytes'] or _sha(data) != entry['sha256']:
                        raise ValueError(f'导航资产大小或 SHA256 不匹配：{relative}')
                    add('data/navigation/' + relative, data, role)

                add('data/navigation/manifest.json', before, 'original_manifest')
                for c in (0, 1):
                    asset(maps[c], 'map_header')
                for key in order:
                    asset(tiles[(0, *key)], 'navigation_tile')
                asset(raw['library'], 'active_query_dll_not_executed')
                for relative in SOURCE_FILES:
                    add(relative, _read(_inside_file(root, relative)), 'project_source')
                vendor = _inside_file(root, 'third_party/detour/Include/DetourNavMesh.h').parent.parent
                for folder in ('Include', 'Source'):
                    directory = vendor / folder
                    if not directory.is_dir():
                        raise ValueError(f'Detour 缺少 {folder} 目录。')
                    selected = sorted(p for p in directory.rglob('*') if p.is_file() and p.suffix.lower() in SOURCE_SUFFIXES)
                    if not selected:
                        raise ValueError(f'Detour 的 {folder} 没有源文件。')
                    for path in selected:
                        relative = 'third_party/detour/' + path.relative_to(vendor).as_posix()
                        add(relative, _read(_inside_file(root, relative)), 'detour_source')
                for path in sorted(vendor.iterdir()):
                    if (path.is_file() and path.name.upper().startswith(('SOURCE', 'README', 'LICENSE', 'COPYING'))
                            and path.suffix.lower() in ('', '.json', '.md', '.txt', '.rst')):
                        relative = 'third_party/detour/' + path.name
                        add(relative, _read(_inside_file(root, relative)), 'detour_provenance_or_license')
                case = {
                    'schema': 1, 'purpose': 'partial offline reproduction; not a production navigation pack',
                    'created_at_utc': datetime.now(timezone.utc).isoformat(),
                    'continent': 0, 'start': list(START), 'goal': list(GOAL),
                    'guide': None, 'blocked_edges': [], 'max_expansions': 500000,
                    'constants': {'TILE_SIZE': tile_size, 'WORLD_TOP': world_top},
                    'distance_from_project_Point': distance, 'corridor_sample_steps': steps,
                    'stages': stages, 'observed_nav_debug': OBSERVED,
                    'stage_results_are_user_observations_not_collector_measurements': True,
                    'collected_tile_count': len(order), 'collected_tile_bytes': tile_bytes,
                    'active_library': raw['library'], 'observed_library_sha256': EXPECTED_DLL_SHA,
                    'matches_reported_tile_count_bytes_and_dll_hash': (
                        len(order) == 49 and tile_bytes == 56694268 and raw['library'].get('sha256') == EXPECTED_DLL_SHA),
                    'original_manifest_sha256': _sha(before), 'warnings': warnings,
                    'dll_executed': False, 'pathfinding_executed': False, 'build_executed': False,
                }
                add('case.json', _json(case), 'reproduction_case')
                add('compile-summary.json', _json(compile_summary), 'selected_compile_properties')
                add('README.md', README.encode('utf-8'), 'instructions')
                add('inventory.json', _json({'algorithm': 'sha256', 'excludes_itself': True, 'files': inventory}),
                    'inventory', include_inventory=False)
            stream.flush()
            os.fsync(stream.fileno())
        if _read(manifest) != before:
            raise RuntimeError('采集期间原始 manifest 已改变；未生成最终 ZIP，请重新采集。')
        if os.name == 'nt':
            os.rename(temporary, final)  # Windows refuses to overwrite an existing destination.
        else:
            os.link(temporary, final)  # Atomic no-overwrite publication on the test host.
            temporary.unlink()
    finally:
        temporary.unlink(missing_ok=True)
    return {'zip': str(final), 'files': len(members), 'uncompressed_bytes': total,
            'tile_count': len(order), 'tile_bytes': tile_bytes,
            'matches_reported_checks': case['matches_reported_tile_count_bytes_and_dll_hash'],
            'warnings': warnings, 'dll_executed': False, 'pathfinding_executed': False}


def main():
    parser = argparse.ArgumentParser(description='只读采集导航失败复现包；不执行 DLL、寻路或构建。')
    parser.add_argument('--project-root', type=Path, default=Path.cwd(), help='包含 adrive、native、data 的项目根目录')
    parser.add_argument('--output', type=Path, help='可选：项目 runtime 内一个尚不存在的 .zip 路径')
    args = parser.parse_args()
    report = collect_case(args.project_root, args.output)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print('采集完成。请上传上面的 ZIP 文件。')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('采集未完成：' + str(exc), file=sys.stderr)
        raise SystemExit(1)
