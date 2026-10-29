"""Collect and verify one isolated source baseline for the Kalimdor bridge.

Uses the standard library and the already installed, hash-pinned WSL generator.
The active navigation pack is read only. New outputs stay in one new runtime folder.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import struct
import subprocess
import tempfile
import threading
import time
import uuid
import zipfile

MAX_FILE = 32 * 1024 * 1024
DEBUG_MAX_FILE = 64 * 1024 * 1024
DEBUG_MAX_TOTAL = 192 * 1024 * 1024
DEBUG_TIMEOUT = 600
DEBUG_TOOL_SHA = 'cdd490271d715b945ea214da5b787ef366ac3b5c49b9c3e89c769f52f5a33451'
GENERATOR_PATH = '/home/wangzekun/forever/server-integrated/bin/mmaps_generator'
TRANSPORT_REVISION = '3-target-config-wsl-heartbeat'


@dataclass(frozen=True)
class TileTarget:
    continent: int
    row: int
    col: int
    tile_sha256: str
    mmap_sha256: str

    def __post_init__(self):
        if self.continent not in (0, 1) or not 0 <= self.row < 64 or not 0 <= self.col < 64:
            raise ValueError('Unsupported diagnostic tile')
        for digest in (self.tile_sha256, self.mmap_sha256):
            if len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
                raise ValueError('Expected a pinned lowercase SHA256')

    @property
    def coordinates(self):
        return (self.continent, self.row, self.col)

    @property
    def map_name(self):
        return f'{self.continent:04d}'

    @property
    def stem(self):
        return f'{self.map_name}_{self.row:02d}_{self.col:02d}'

    @property
    def tile_name(self):
        return self.stem + '.mmtile'

    @property
    def mmap_name(self):
        return self.map_name + '.mmap'

    @property
    def products(self):
        return ('mmaps/' + self.mmap_name, 'mmaps/' + self.tile_name,
                'meshes/map' + self.stem + '.obj', 'meshes/map' + self.stem + '.mesh',
                'meshes/' + self.stem + '.pmesh', 'meshes/' + self.stem + '.dmesh')

    def payload(self):
        return {'continent': self.continent, 'row': self.row, 'col': self.col,
                'tile_name': self.tile_name, 'mmap_name': self.mmap_name,
                'tile_sha256': self.tile_sha256, 'mmap_sha256': self.mmap_sha256}


DEFAULT_TARGET = TileTarget(1, 37, 38,
    'bf81052e0ebbc9e5bbd58e2ae269181b14c0e59e9a2f9874d5ba6707e292a825',
    '21bb845d158b253bae85d30004cd56abbd79542b3afad5bc3513efe736eb48c9')


def say(message):
    print(message, flush=True)


def source_evidence_complete(report):
    records = report.get('files', [])
    return (bool(records) and not report.get('errors')
            and report.get('collision_dependencies_complete') is True
            and all(r.get('included') for r in records if r.get('required') or r.get('present')))


def tile_debug_readme(target):
    return f"""# Isolated bridge source baseline: {target.stem}

The installed hash-pinned generator runs once with original parameters into a new
isolated output directory. No active manifest, tile, DLL, source or tool is replaced.
The original work directory is matched by published provenance and target hash.

The exact .mesh keeps float32 geometry; OBJ/.mesh combine liquid and solid data
without storing their split or area flags. Raw terrain, VMAP spawn/tree/index files,
referenced VMO files, optional road sidecars and metadata are captured to recover
those semantics. Liquid counts from a different tile must not be reused here.
The debug pmesh has an extra borderSize=5 horizontal offset.

case.json records baseline status and source_evidence_complete separately.
A successful generator exit is insufficient: the generated target and map header
must match the active pinned bytes, required outputs must exist, and input snapshots
must be stable. Missing evidence and byte limits are explicit, never fabricated.

current-last-plan.json, when present, is a best-effort latest application snapshot;
it may describe another route. See case.json rather than assuming it is this bridge.
This package does not include an executable replacement navigation pack.
Limits: each file 64 MiB; total uncompressed archive 192 MiB.
inventory.json records every other member's SHA256 and length.
"""



def sha(data):
    return hashlib.sha256(data).hexdigest()


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode('utf-8')


def relative(name):
    name = str(name).replace('\\', '/')
    if (not name or '\x00' in name or PurePosixPath(name).is_absolute()
            or any(p in ('', '.', '..') or ':' in p for p in name.split('/'))):
        raise ValueError('Unsafe relative path: ' + repr(name))
    return name


def inside(base, name, must_exist=True):
    base = Path(base).resolve()
    candidate = base.joinpath(*PurePosixPath(relative(name)).parts)
    # Do not follow links/junctions out of an explicitly permitted directory.
    resolved = candidate.resolve()
    if not resolved.is_relative_to(base):
        raise ValueError('Path escapes permitted directory: ' + str(candidate))
    if must_exist and not resolved.is_file():
        raise FileNotFoundError(str(candidate))
    return resolved


def read_limited(path, limit=MAX_FILE):
    if path.stat().st_size > limit:
        raise ValueError('File exceeds byte limit: ' + str(path))
    with path.open('rb') as handle:
        data = handle.read(limit + 1)
    if len(data) > limit:
        raise ValueError('File grew beyond byte limit: ' + str(path))
    return data


def shallow_dirs(base, maximum=512, warnings=None):
    """At most two directory levels; never walk junctions outside base."""
    base = Path(base).resolve()
    output, pending = [], [(base, 0)]
    if not base.is_dir():
        return [], False
    capped = False
    while pending:
        parent, depth = pending.pop(0)
        output.append(parent)
        if depth >= 2:
            continue
        try:
            for child in sorted(parent.iterdir(), key=str):
                if (child.is_dir() and not child.is_symlink()
                        and child.resolve().is_relative_to(base)):
                    if len(output) + len(pending) >= maximum:
                        capped = True
                        break
                    pending.append((child, depth + 1))
        except OSError as exc:
            if warnings is not None:
                warnings.append('Skipped unreadable directory: ' + str(parent) + ': ' + str(exc))
    return output, capped


def tile_metadata(data):
    result = {'bytes': len(data), 'sha256': sha(data), 'first_120_bytes_hex': data[:120].hex()}
    if len(data) >= 120:
        result.update(mmap_magic_version=list(struct.unpack_from('<III', data)),
                      mesh_x=struct.unpack_from('<i', data, 28)[0],
                      mesh_y=struct.unpack_from('<i', data, 32)[0],
                      agent_height_radius_climb=list(struct.unpack_from('<3f', data, 80)))
    return result


def find_original_workdirs(root, provenance_bytes, target_sha, warnings, target=DEFAULT_TARGET):
    wanted = json.loads(provenance_bytes.decode('utf-8-sig'))
    found, matches = [], []
    candidates, capped = shallow_dirs(root / 'runtime', warnings=warnings)
    if capped:
        warnings.append('Windows runtime directory discovery stopped at 512 directories.')
    for candidate in candidates:
        entry = {'directory': str(candidate), 'match': False}
        try:
            p = candidate / 'provenance.json'
            if not p.is_file():
                continue
            raw = read_limited(inside(candidate, 'provenance.json'))
            entry['provenance_sha256'] = sha(raw)
            entry['provenance_match'] = ('exact_bytes' if raw == provenance_bytes else
                                         'json_equivalent' if json.loads(raw.decode('utf-8-sig')) == wanted
                                         else 'different')
            if entry['provenance_match'] != 'different':
                tile = read_limited(inside(candidate, 'mmaps/' + target.tile_name))
                entry['target_sha256'] = sha(tile)
                entry['match'] = entry['target_sha256'] == target_sha
                if entry['match']:
                    matches.append(candidate)
        except (OSError, ValueError, TypeError) as exc:
            entry['error'] = str(exc)
        found.append(entry)
    return found, matches


TILE_DEBUG_WORKER = r"""
import hashlib, json, os, pathlib, signal, struct, subprocess, sys, time
P = PAYLOAD
T = P['target']
TILE_PRODUCT = 'mmaps/' + T['tile_name']
MAP_PRODUCT = 'mmaps/' + T['mmap_name']
out = pathlib.Path(P['output'])
original = pathlib.Path(P['input'])
tool = pathlib.Path(P['tool'])
R = {'schema': 1, 'mode': 'isolated-single-tile-baseline', 'target': T, 'actual_input': str(original),
     'output': str(out), 'tool_path': str(tool), 'generator_started': False,
     'timed_out': False, 'returncode': None, 'warnings': [], 'optional_inputs': {},
     'products': [], 'timeout_seconds': P['timeout'], 'input_unchanged': None}
process = None
can_write = False
def digest_file(path):
    if path.is_symlink(): raise ValueError('Refusing symlink: ' + str(path))
    size = path.stat().st_size
    if size > P['file_limit']: raise ValueError('File exceeds 64 MiB limit: ' + str(path))
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''): h.update(block)
    return {'bytes': size, 'sha256': h.hexdigest()}
def under(path, parent):
    try: path.resolve().relative_to(parent.resolve()); return True
    except (ValueError, OSError): return False
def save_report(name):
    data = (json.dumps(R, ensure_ascii=False, indent=2) + '\n').encode('utf-8')
    temporary = out / (name + '.tmp')
    with temporary.open('wb') as f: f.write(data); f.flush(); os.fsync(f.fileno())
    os.replace(temporary, out / name)
def stop_group():
    if process is None or process.poll() is not None: return
    R['termination_requested'] = True
    try: os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError: return
    try: process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try: os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError: pass
        process.wait(timeout=5)
        R['group_sigkill_sent'] = True
def terminated(signum, frame):
    raise RuntimeError('WSL runner received signal ' + str(signum))
signal.signal(signal.SIGTERM, terminated)
try:
    if (not out.is_absolute() or not original.is_absolute() or not tool.is_absolute()
            or not out.is_dir() or not original.is_dir() or out.is_symlink()
            or under(out, original) or under(original, out)):
        raise ValueError('Input/output paths are invalid or overlap.')
    if any(p.name != 'input-evidence' for p in out.iterdir()):
        raise ValueError('Isolated output already contains generation files; refusing to overwrite.')
    can_write = True
    # Only a SHA-verified ELF is ever executed. The expected value is fixed in
    # the Windows caller and is also checked against published provenance.
    if tool.is_symlink(): raise ValueError('Installed generator path is a symlink; execution refused.')
    with tool.open('rb') as f:
        if f.read(4) != b'\x7fELF': raise ValueError('Installed generator is not ELF.')
    R['tool'] = digest_file(tool)
    R['tool']['matches_expected'] = R['tool']['sha256'] == P['tool_sha256']
    if not R['tool']['matches_expected']: raise ValueError('Installed generator SHA256 changed; execution refused.')
    for name, expected in (('provenance.json', P['input_provenance_sha256']),
                           (TILE_PRODUCT, P['target_sha256'])):
        path = original / name
        if not under(path, original): raise ValueError('Original input path escapes its work directory.')
        actual = digest_file(path)
        if actual['sha256'] != expected: raise ValueError('Original input changed before execution: ' + name)
    for name in ('world_metadata.csv', 'offmesh.txt'):
        path = original / name
        if path.exists():
            if not under(path, original): raise ValueError('Optional input escapes work directory: ' + name)
            R['optional_inputs'][name] = {'exists': True, **digest_file(path)}
        else: R['optional_inputs'][name] = {'exists': False}
    command = [str(tool), str(T['continent']), '--tile', str(T['row']) + ',' + str(T['col']), '--input', str(original), '--output', str(out),
               '--threads', '1', '--maxAngle', '50', '--maxAngleNotSteep', '50',
               '--walkableClimb', '2', '--walkableRadius', '2', '--debugOutput', 'true', '--silent']
    R['command'] = command
    R['cwd'] = str(out)
    R['started_at_unix'] = time.time()
    with (out / 'generator.log').open('xb') as log:
        process = subprocess.Popen(command, cwd=str(out), stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        R['generator_started'] = True
        R['generator_pid'] = process.pid
        save_report('runner-status.json')
        try: R['returncode'] = process.wait(timeout=P['timeout'])
        except subprocess.TimeoutExpired:
            R['timed_out'] = True
            R['warnings'].append('Generator exceeded timeout; its process group was terminated.')
            stop_group()
            R['returncode'] = process.returncode
        log.flush()
    R['elapsed_seconds'] = time.time() - R['started_at_unix']
except BaseException as exc:
    R['error'] = type(exc).__name__ + ': ' + str(exc)
finally:
    try: stop_group()
    except BaseException as exc: R['termination_error'] = str(exc)
    if process is not None: R['returncode'] = process.returncode
    if can_write:
        for name in P['products']:
            row = {'path': name, 'exists': False, 'nonempty': False}
            try:
                path = out / name
                if not under(path, out): raise ValueError('Output path escapes isolation directory.')
                if path.is_file():
                    row.update(exists=True, nonempty=path.stat().st_size > 0,
                               bytes=path.stat().st_size)
                    row.update(digest_file(path))
                    if name == TILE_PRODUCT:
                        with path.open('rb') as f: header = f.read(120)
                        valid = (len(header) >= 120
                                 and struct.unpack_from('<III', header) == (0x4d4d4150, 7, 16)
                                 and struct.unpack_from('<I', header, 12)[0] == row['bytes'] - 20
                                 and struct.unpack_from('<II', header, 20) == (0x444e4156, 7))
                        row['header_valid'] = valid
                        row['matches_published_sha256'] = row['sha256'] == P['target_sha256']
                        row['first_120_bytes_hex'] = header.hex()
                    elif name == MAP_PRODUCT:
                        with path.open('rb') as f: header = f.read(40)
                        row['header_valid'] = len(header) == 40 and struct.unpack_from('<II', header) == (0x4d4d4150, 16)
                        row['matches_published_sha256'] = row['sha256'] == P['mmap_sha256']
                        row['matches_published_header'] = header.hex() == P['mmap_header_hex']
            except (OSError, ValueError, struct.error) as exc: row['error'] = str(exc)
            if not row['nonempty']: R['warnings'].append('Missing/empty required output: ' + name)
            if row.get('header_valid') is False: R['warnings'].append('Invalid generated header: ' + name)
            if row.get('matches_published_sha256') is False: R['warnings'].append('Baseline differs from published data: ' + name)
            R['products'].append(row)
        try:
            R['input_unchanged'] = (
                digest_file(original / 'provenance.json')['sha256'] == P['input_provenance_sha256']
                and digest_file(original / TILE_PRODUCT)['sha256'] == P['target_sha256'])
            if not R['input_unchanged']: R['warnings'].append('Original provenance/output tile changed during the run.')
        except (OSError, ValueError) as exc: R['input_recheck_error'] = str(exc)
        try:
            R['tool_after'] = digest_file(tool)
            R['tool_unchanged'] = R['tool_after']['sha256'] == P['tool_sha256']
            if not R['tool_unchanged']: R['warnings'].append('Generator binary changed during the run.')
        except (OSError, ValueError) as exc:
            R['tool_unchanged'] = False
            R['tool_recheck_error'] = str(exc)
        R['required_outputs_nonempty'] = len(R['products']) == len(P['products']) and all(x['nonempty'] for x in R['products'])
        R['required_outputs_verified'] = (R['required_outputs_nonempty']
            and all(x.get('sha256') and not x.get('error') for x in R['products']))
        R['baseline_matches_published'] = all(
            next((x.get('matches_published_sha256') is True and x.get('header_valid') is True
                  for x in R['products'] if x['path'] == name), False)
            for name in (MAP_PRODUCT, TILE_PRODUCT))
        R['verified_baseline'] = (R['generator_started'] and R['returncode'] == 0 and not R['timed_out']
                                  and not R.get('error') and R['required_outputs_verified']
                                  and R['baseline_matches_published'] and R['input_unchanged'] is True
                                  and R['tool_unchanged'] is True)
        try: save_report('runner-result.json')
        except OSError as exc: R['report_write_error'] = str(exc)
    print(json.dumps(R, ensure_ascii=False))
"""


WSL_PATH_WORKER = r"""
import json, subprocess, sys
payload = json.load(sys.stdin)
result = subprocess.run(['wslpath', '-a', payload['windows_path']],
                        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE, timeout=20)
sys.stdout.buffer.write(result.stdout)
sys.stderr.buffer.write(result.stderr)
raise SystemExit(result.returncode)
"""


def wsl_path(path, distribution):
    # The Windows path travels only in stdin JSON, never through a Linux shell
    # or WSL's command-line conversion. wslpath still uses the real mount setup.
    payload = json.dumps({'windows_path': str(path)}, ensure_ascii=True).encode('utf-8')
    result = subprocess.run(['wsl.exe', '-d', distribution, '--exec',
                             'python3', '-B', '-c', WSL_PATH_WORKER],
                            input=payload, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=30)
    if result.returncode:
        raise RuntimeError('wslpath failed: ' + result.stderr[:1000].decode('utf-8', 'replace'))
    value = result.stdout.decode('utf-8', 'strict').removesuffix('\n').removesuffix('\r')
    if not value.startswith('/') or '\x00' in value or '\n' in value:
        raise ValueError('wslpath returned an invalid absolute path.')
    return value


def run_tile_debug_worker(payload, distribution, heartbeat_seconds=10):
    # Heartbeats are printed by the Windows caller, never into the worker JSON.
    program = 'PAYLOAD = ' + repr(payload) + '\n' + TILE_DEBUG_WORKER
    stopped = threading.Event()
    started = time.monotonic()

    def heartbeat():
        while not stopped.wait(heartbeat_seconds):
            say('  单瓦片生成仍在运行，已等待 %d 秒 ...' % (time.monotonic()-started))

    worker = threading.Thread(target=heartbeat, daemon=True)
    worker.start()
    try:
        result = subprocess.run(['wsl.exe', '-d', distribution, '--exec', 'python3', '-B', '-'],
                                input=program.encode('utf-8'), stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, timeout=DEBUG_TIMEOUT + 120)
    finally:
        stopped.set()
        worker.join(timeout=1)
    if result.returncode:
        raise RuntimeError('WSL tile runner failed: ' + result.stderr[:1600].decode('utf-8', 'replace'))
    return json.loads(result.stdout.decode('utf-8', 'strict'))



def parse_target_vmtile(data, indices=None):
    """Read the exact VMAP_4.E ModelSpawn serialization from the supplied source."""
    if len(data) < 12 or data[:8] != b'VMAP_4.E':
        raise ValueError('Target vmtile does not use VMAP_4.E.')
    count = struct.unpack_from('<I', data, 8)[0]
    if count > 100000:
        raise ValueError('Target vmtile spawn count exceeds the diagnostic limit.')
    offset, spawns = 12, []

    def take(fmt):
        nonlocal offset
        size = struct.calcsize(fmt)
        if offset + size > len(data):
            raise ValueError('Truncated ModelSpawn at byte %d.' % offset)
        value = struct.unpack_from(fmt, data, offset)
        offset += size
        return value

    for _ in range(count):
        values = take('<BBI7f')
        flags, adt, ident = values[:3]
        record = {'flags': flags, 'adt_id': adt, 'id': ident,
                  'position_internal': list(values[3:6]),
                  'rotation_degrees': list(values[6:9]), 'scale': values[9]}
        if flags & 1:  # MOD_HAS_BOUND, exact value from ModelInstance.h.
            bounds = take('<6f')
            record['bounds_internal'] = [list(bounds[:3]), list(bounds[3:])]
        length = take('<I')[0]
        if not 1 <= length <= 500 or offset + length > len(data):
            raise ValueError('Invalid ModelSpawn name length at byte %d.' % offset)
        name = data[offset:offset+length].decode('utf-8', 'strict')
        offset += length
        record['name'] = relative(name)
        if record['name'] != name:
            raise ValueError('ModelSpawn name needs path rewriting; refusing to guess.')
        spawns.append(record)
    if offset != len(data):
        raise ValueError('Unexpected trailing bytes in target vmtile.')
    if indices is not None:
        if len(indices) < 12 or indices[:8] != b'VMAP_4.E':
            raise ValueError('Target vmtileidx does not use VMAP_4.E.')
        index_count = struct.unpack_from('<I', indices, 8)[0]
        if index_count != count or len(indices) != 12 + 4 * count:
            raise ValueError('vmtile and vmtileidx counts/lengths disagree.')
        for i, spawn in enumerate(spawns):
            spawn['tree_index'] = struct.unpack_from('<I', indices, 12+4*i)[0]
    return spawns


def target_vmtree_primitive_count(data):
    if len(data) < 44 or data[:12] != b'VMAP_4.ENODE':
        raise ValueError('Unexpected target vmtree header.')
    tree_words = struct.unpack_from('<I', data, 36)[0]
    offset = 40 + 4 * tree_words
    if offset + 4 > len(data):
        raise ValueError('Truncated target vmtree BIH nodes.')
    count = struct.unpack_from('<I', data, offset)[0]
    if offset + 4 + 4 * count != len(data):
        raise ValueError('Unexpected target vmtree BIH object length.')
    return count


def collect_tile_input_evidence(work, add, warnings, target=DEFAULT_TARGET):
    """Copy the declared target's collision dependencies plus bounded metadata.

    add(name, bytes, role) returns whether an archive member was included.
    This is target-specific input evidence, not a replacement full input tree.
    Snapshot checks describe current bytes, not their original extraction date.
    """
    work = Path(work).resolve()
    report = {'schema': 1, 'source_directory': str(work), 'target': list(target.coordinates),
              'source_format': 'VMAP_4.E; ModelSpawn uint8 flags,uint8 adtId',
              'files': [], 'errors': [], 'collision_dependencies_complete': False,
              'complete_generator_input_directory': False,
              'historical_raw_input_hashes_available': False}
    captured = set()

    def record(name, role, required=False):
        name = relative(name)
        if name in captured:
            return None
        row = {'path': name, 'required': required, 'present': False, 'included': False}
        report['files'].append(row)
        captured.add(name)
        try:
            candidate = inside(work, name, must_exist=False)
            if not candidate.exists():
                if required:
                    raise FileNotFoundError('Required target input is absent: ' + name)
                return None
            if not candidate.is_file():
                raise ValueError('Target input is not a regular file: ' + name)
            row['present'] = True
            data = read_limited(candidate, 64 * 1024 * 1024)
            row.update(bytes=len(data), sha256=sha(data))
            row['included'] = bool(add('inputs/' + name, data, role))
            if not row['included']:
                report['errors'].append('Input omitted by archive budget: ' + name)
            return data
        except (OSError, ValueError) as exc:
            row['error'] = str(exc)
            report['errors'].append(str(exc))
            warnings.append('Target input evidence unavailable: ' + name)
            return None

    # Ground, neighbour edge strips and optional road mask, as loaded by TerrainBuilder.
    for row in range(max(0, target.row-1), min(64, target.row+2)):
        for col in range(max(0, target.col-1), min(64, target.col+2)):
            name = 'maps/%s_%02d_%02d.map' % (target.map_name, row, col)
            data = record(name, 'raw-terrain', required=(row == target.row or col == target.col))
            if data is not None:
                entry = report['files'][-1]
                entry['header_client_build'] = struct.unpack_from('<I', data, 8)[0] if len(data) >= 12 else None
                entry['matches_case_client_build'] = entry['header_client_build'] == 70658
                if not entry['matches_case_client_build']:
                    warnings.append('Target raw map client build differs: ' + name)
    record('maps/' + target.stem + '.road', 'optional-road-mask')
    for name in ('world_metadata.csv', 'offmesh.txt'):
        record(name, 'optional-generator-input')

    # Map discovery depends on tile lists and VMAP filenames. Preserve both inventories.
    discovery = {'tilelists': [], 'target_vmap_files': [], 'dbc_locale_directories': []}
    try:
        maps = inside(work, 'maps', must_exist=False)
        candidates = sorted(p for p in maps.iterdir() if p.suffix == '.tilelist' and p.is_file())
        if len(candidates) > 1024:
            raise ValueError('Too many tilelist files for this diagnostic scope.')
        for path in candidates:
            name = path.relative_to(work).as_posix()
            discovery['tilelists'].append(path.name)
            record(name, 'map-tile-discovery', required=path.name.startswith(target.map_name))
    except (OSError, ValueError) as exc:
        report['errors'].append(str(exc))
        warnings.append('Could not collect complete tilelist discovery evidence.')
    try:
        directory = inside(work, 'vmaps/' + target.map_name, must_exist=False)
        items = sorted(directory.iterdir(), key=lambda p: p.name)
        if len(items) > 20000:
            raise ValueError('Too many target-map VMAP files for diagnostic inventory.')
        for path in items:
            if path.is_file():
                resolved = inside(work, 'vmaps/' + target.map_name + '/' + path.name)
                discovery['target_vmap_files'].append({'name': path.name, 'bytes': resolved.stat().st_size})
    except (OSError, ValueError) as exc:
        report['errors'].append(str(exc))
        warnings.append('Could not collect complete target-map VMAP discovery evidence.')
    try:
        dbc = inside(work, 'dbc', must_exist=False)
        locales = sorted(p for p in dbc.iterdir() if p.is_dir())
        if len(locales) > 32:
            raise ValueError('Too many dbc locale directories for diagnostic scope.')
        for locale in locales:
            # Generator enumerates child directories; record all names, collect only its two DB2 inputs.
            inside(work, 'dbc/' + locale.name, must_exist=False)
            discovery['dbc_locale_directories'].append(locale.name)
            for name in ('Map.db2', 'LiquidType.db2'):
                record('dbc/' + locale.name + '/' + name, 'generator-db2-metadata')
    except (OSError, ValueError) as exc:
        report['errors'].append(str(exc))
        warnings.append('Could not collect DB2 metadata for target generation.')
    report['discovery'] = discovery
    add('inputs/discovery.json', json_bytes(discovery), 'input-discovery-inventory')

    # The fixed format uses a separate spawn-index file; models themselves are flat
    # dependencies under vmaps/ and have an optional .vmo.road sidecar.
    tree = record('vmaps/' + target.map_name + '/' + target.map_name + '.vmtree', 'vmap-tree', required=True)
    tile = record('vmaps/' + target.map_name + '/' + target.stem + '.vmtile', 'vmap-target-spawns', required=True)
    idx = record('vmaps/' + target.map_name + '/' + target.stem + '.vmtileidx', 'vmap-target-indices', required=True)
    collision_start = len(report['errors'])
    if tile is not None and idx is not None and tree is not None:
        try:
            spawns = parse_target_vmtile(tile, idx)
            prim_count = target_vmtree_primitive_count(tree)
            if any(s['tree_index'] >= prim_count for s in spawns):
                raise ValueError('Target VMAP spawn references an out-of-range tree element.')
            names = sorted({s['name'] for s in spawns})
            report.update(spawn_count=len(spawns), model_count=len(names), tree_primitive_count=prim_count)
            add('inputs/target-model-spawns.json', json_bytes(spawns), 'decoded-vmap-spawns')
            for name in names:
                data = record('vmaps/' + name + '.vmo', 'vmap-referenced-model', required=True)
                if data is not None and not data.startswith(b'VMAP_4.EWMOD'):
                    report['errors'].append('Referenced model has unexpected VMAP header: ' + name)
                    warnings.append('Unexpected referenced model format: ' + name)
                record('vmaps/' + name + '.vmo.road', 'optional-model-road-sidecar')
            required_records = [r for r in report['files'] if r['required'] and r['path'].startswith('vmaps/')]
            report['collision_dependencies_complete'] = (
                len(report['errors']) == collision_start and
                all(r.get('included') for r in required_records))
        except (ValueError, UnicodeError, struct.error) as exc:
            report['errors'].append(str(exc))
            warnings.append('Could not resolve exact target VMAP model dependencies: ' + str(exc))
    report['source_evidence_complete'] = source_evidence_complete(report)
    add('inputs/collection.json', json_bytes(report), 'target-input-collection-report')
    return report


def verify_tile_input_evidence(work, report, warnings, target=DEFAULT_TARGET):
    """Check captured inputs again after generation; no historical-hash claim."""
    work = Path(work).resolve()
    result = {'checked_files': 0, 'changes_or_errors': [], 'unchanged': False}
    for entry in report.get('files', []):
        name = entry['path']
        try:
            path = inside(work, name, must_exist=False)
            if not entry['present']:
                if path.exists():
                    result['changes_or_errors'].append('Previously absent input now exists: ' + name)
            elif not entry.get('sha256'):
                result['changes_or_errors'].append('No complete pre-run hash: ' + name)
            else:
                data = read_limited(path, 64 * 1024 * 1024)
                if sha(data) != entry['sha256'] or len(data) != entry['bytes']:
                    result['changes_or_errors'].append('Input changed during diagnostic generation: ' + name)
                result['checked_files'] += 1
        except (OSError, ValueError) as exc:
            result['changes_or_errors'].append(name + ': ' + str(exc))
    discovery = report.get('discovery', {})
    try:
        maps = inside(work, 'maps', must_exist=False)
        now = sorted(p.name for p in maps.iterdir() if p.suffix == '.tilelist' and p.is_file())
        if now != discovery.get('tilelists', []):
            result['changes_or_errors'].append('Map tilelist filenames changed during generation.')
        directory = inside(work, 'vmaps/' + target.map_name, must_exist=False)
        now = []
        for p in sorted(directory.iterdir(), key=lambda p: p.name):
            if p.is_file():
                resolved = inside(work, 'vmaps/' + target.map_name + '/' + p.name)
                now.append({'name': p.name, 'bytes': resolved.stat().st_size})
        if now != discovery.get('target_vmap_files', []):
            result['changes_or_errors'].append('Target-map VMAP discovery inventory changed during generation.')
        dbc = inside(work, 'dbc', must_exist=False)
        now = sorted(p.name for p in dbc.iterdir() if p.is_dir())
        if now != discovery.get('dbc_locale_directories', []):
            result['changes_or_errors'].append('DB2 locale directory names changed during generation.')
    except (OSError, ValueError) as exc:
        result['changes_or_errors'].append('Input discovery could not be rechecked: ' + str(exc))
    result['unchanged'] = not result['changes_or_errors']
    if not result['unchanged']:
        warnings.append('Target input bytes could not be confirmed stable during generation.')
    return result


def collect_tile_debug(root, distribution='Ubuntu', target=DEFAULT_TARGET):
    root = Path(root).resolve()
    if not root.is_dir():
        raise ValueError('Project root does not exist.')
    runtime = (root / 'runtime').resolve()
    if not runtime.is_relative_to(root):
        raise ValueError('runtime points outside project root.')
    runtime.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = runtime / ('bridge-source-' + target.stem + '-' + stamp + '-' + uuid.uuid4().hex[:8])
    output.mkdir(exist_ok=False)
    warnings, references = [], {}
    staged, staged_names, staged_total = [], set(), 0
    stage_root = output / 'input-evidence'
    target_path, published_mmap_path = None, None
    work, manifest_path, provenance_path = None, None, None
    report = {'schema': 1, 'mode': 'isolated-single-tile-baseline', 'root': str(root),
              'target': target.payload(), 'source_evidence_complete': False,
              'transport_revision': TRANSPORT_REVISION,
              'output_directory': str(output), 'distribution': distribution,
              'warnings': warnings, 'generator_started': False, 'verified_baseline': False,
              'production_activated': False, 'parameter_sweep': False,
              'input_evidence': None, 'file_limit': DEBUG_MAX_FILE, 'total_limit': DEBUG_MAX_TOTAL}
    say('阶段 1/3：核对目标瓦片、版本和原始工作目录 ...')
    try:
        manifest_path = inside(root, 'data/navigation/manifest.json')
        manifest_bytes = read_limited(manifest_path)
        references['published/manifest.json'] = manifest_bytes
        manifest = json.loads(manifest_bytes.decode('utf-8-sig'))
        if manifest.get('kind') != 'adrive-ground-navmesh' or manifest.get('mmap_version') != 16:
            raise ValueError('Unexpected navigation manifest format.')
        targets = [t for t in manifest['tiles'] if (t.get('c'), t.get('row'), t.get('col')) == target.coordinates]
        if len(targets) != 1 or targets[0].get('sha256') != target.tile_sha256:
            raise ValueError('Active target does not match the pinned ' + target.stem + ' tile.')
        target_path = inside(manifest_path.parent, targets[0]['path'])
        target_data = read_limited(target_path)
        if sha(target_data) != target.tile_sha256 or len(target_data) != targets[0].get('bytes'):
            raise ValueError('Active target tile bytes/hash do not match its manifest.')
        report['target_record'] = targets[0]
        report['target_metadata'] = tile_metadata(target_data)
        provenance_path = inside(manifest_path.parent, manifest['provenance'])
        provenance_bytes = read_limited(provenance_path)
        references['published/sources/provenance.json'] = provenance_bytes
        provenance = json.loads(provenance_bytes.decode('utf-8-sig'))
        server = provenance.get('server', {})
        if server.get('tools_sha256', {}).get('mmaps_generator') != DEBUG_TOOL_SHA:
            raise ValueError('Published provenance does not identify the verified generator binary.')
        settings = provenance.get('settings', {})
        for key, expected in (('cell_size_yards', .2666666), ('agent_height_yards', 1.5999996),
                              ('agent_radius_yards', .5333332), ('walkable_climb_yards', .5333332),
                              ('max_slope_degrees', 50)):
            if not isinstance(settings.get(key), (int, float)) or not abs(settings[key] - expected) <= .00001:
                raise ValueError('Published profile differs from the fixed baseline: ' + key)
        candidates, matches = find_original_workdirs(root, provenance_bytes, target.tile_sha256, warnings, target)
        matches = list(dict.fromkeys(p.resolve() for p in matches))
        report['runtime_candidates'] = candidates
        known = (runtime / 'nav-build-70658').resolve()
        if len(matches) == 1:
            work = matches[0]
        elif known in matches:
            work = known
            warnings.append('Multiple matching work directories; selected the independently matched nav-build-70658 directory.')
        else:
            raise ValueError('Original work directory is missing or ambiguous; generator was not run.')
        if output.is_relative_to(work) or work.is_relative_to(output):
            raise ValueError('Original input and isolated output must not overlap.')
        original_provenance = read_limited(inside(work, 'provenance.json'))
        if json.loads(original_provenance.decode('utf-8-sig')) != provenance:
            raise ValueError('Matched work provenance changed before the run.')
        if sha(read_limited(inside(work, 'mmaps/' + target.tile_name))) != target.tile_sha256:
            raise ValueError('Matched work target changed before the run.')
        references['original-work/provenance.json'] = original_provenance
        maps = [m for m in manifest['maps'] if m.get('c') == target.continent]
        if len(maps) != 1:
            raise ValueError('Published target-map header is missing or ambiguous.')
        published_mmap_path = inside(manifest_path.parent, maps[0]['path'])
        mmap = read_limited(published_mmap_path)
        if len(mmap) < 40 or len(mmap) != maps[0]['bytes'] or sha(mmap) != maps[0]['sha256'] or sha(mmap) != target.mmap_sha256:
            raise ValueError('Published target-map header bytes/hash do not match its manifest.')
        references['published/mmaps/' + target.mmap_name] = mmap
        report['actual_input'] = str(work)
        report['input_provenance_sha256'] = sha(original_provenance)
        report['expected_target_sha256'] = target.tile_sha256
        report['expected_mmap_sha256'] = sha(mmap)
        latest = inside(root, 'runtime/route-quality/last-plan.json', must_exist=False)
        if latest.is_file():
            try:
                latest_data = read_limited(latest, 8*1024*1024)
                references['current-last-plan.json'] = latest_data
                latest_json = json.loads(latest_data.decode('utf-8-sig'))
                report['latest_plan_snapshot'] = {
                    'sha256': sha(latest_data),
                    'started_at_utc': latest_json.get('started_at_utc'),
                    'start': latest_json.get('start'), 'goal': latest_json.get('goal'),
                    'same_continent_as_target': latest_json.get('start', {}).get('c') == target.continent,
                    'note': 'Latest snapshot may describe a different route; it is not an endpoint source for this target.'}
            except (OSError, ValueError, TypeError) as exc:
                warnings.append('Optional latest planning snapshot unavailable: ' + str(exc))
        stage_root.mkdir(exist_ok=False)

        def stage_add(name, data, role='input-evidence'):
            nonlocal staged_total
            name = relative(name)
            if name in staged_names:
                raise ValueError('Duplicate staged input member: ' + name)
            # Reserve space for generation products, logs and final reports.
            if len(data) > DEBUG_MAX_FILE or staged_total + len(data) > DEBUG_MAX_TOTAL - DEBUG_MAX_FILE:
                warnings.append('Input evidence byte limit: skipped ' + name)
                return False
            destination = inside(stage_root, name, must_exist=False)
            destination.parent.mkdir(parents=True, exist_ok=True)
            with destination.open('xb') as handle:
                handle.write(data)
            staged_names.add(name)
            staged_total += len(data)
            staged.append({'name': name, 'role': role, 'path': destination,
                           'bytes': len(data), 'sha256': sha(data)})
            return True

        say('阶段 2/3：保存原始地形、模型依赖和输入快照 ...')
        report['input_collection_phase'] = 'before_generator'
        report['input_collection_started_utc'] = datetime.now(timezone.utc).isoformat()
        report['input_evidence'] = collect_tile_input_evidence(work, stage_add, warnings, target)
        report['input_collection_finished_utc'] = datetime.now(timezone.utc).isoformat()
        report['source_evidence_complete'] = source_evidence_complete(report['input_evidence'])
        if not report['source_evidence_complete']:
            raise ValueError('Required source evidence is incomplete; isolated generation was not started.')
        payload = {'target': target.payload(), 'input': wsl_path(work, distribution), 'output': wsl_path(output, distribution),
                   'tool': GENERATOR_PATH,
                   'tool_sha256': DEBUG_TOOL_SHA, 'target_sha256': target.tile_sha256,
                   'input_provenance_sha256': sha(original_provenance),
                   'mmap_sha256': sha(mmap), 'mmap_header_hex': mmap[:40].hex(),
                   'products': list(target.products), 'timeout': DEBUG_TIMEOUT, 'file_limit': DEBUG_MAX_FILE}
        report['worker_payload'] = payload
        say('阶段 3/3：生成单个基线瓦片 ' + target.stem + '，最长等待 10 分钟 ...')
        worker_report = run_tile_debug_worker(payload, distribution)
        report['runner'] = worker_report
        report['generator_started'] = bool(worker_report.get('generator_started'))
        report['verified_baseline'] = bool(worker_report.get('verified_baseline'))
        warnings.extend(worker_report.get('warnings', []))
        if worker_report.get('error'):
            warnings.append('Generator runner error: ' + str(worker_report['error']))
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, subprocess.TimeoutExpired) as exc:
        report['error'] = type(exc).__name__ + ': ' + str(exc)
        warnings.append('Baseline was refused, failed or incomplete; existing diagnostics are still packaged.')
        result_path = output / 'runner-result.json'
        if result_path.is_file():
            try:
                report['runner'] = json.loads(read_limited(result_path).decode('utf-8'))
                report['generator_started'] = bool(report['runner'].get('generator_started'))
            except (OSError, ValueError):
                pass

    if work is not None and report.get('input_evidence') is not None:
        report['input_verification_phase'] = 'after_generator_attempt'
        report['input_verification_started_utc'] = datetime.now(timezone.utc).isoformat()
        try:
            report['input_stability'] = verify_tile_input_evidence(work, report['input_evidence'], warnings, target)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            report['input_stability'] = {'unchanged': False, 'error': str(exc)}
            warnings.append('Original input stability verification failed.')
        report['verified_baseline'] = bool(report['verified_baseline'] and report['input_stability'].get('unchanged'))
        report['input_verification_finished_utc'] = datetime.now(timezone.utc).isoformat()

    say('正在核对输入快照并打包调试产物 ...')
    final = output / ('bridge-source-' + target.stem + '.zip')
    fd, temporary_name = tempfile.mkstemp(prefix='.tile-debug-', suffix='.tmp', dir=output)
    temporary = Path(temporary_name)
    inventory, members, total = [], set(), 0
    try:
        with os.fdopen(fd, 'w+b') as stream:
            with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
                def add(name, data, role='input-evidence', essential=False):
                    nonlocal total
                    name = relative(name)
                    if name in members:
                        raise ValueError('Duplicate archive member: ' + name)
                    if len(data) > DEBUG_MAX_FILE or total + len(data) > DEBUG_MAX_TOTAL - (0 if essential else 2 * 1024 * 1024):
                        warnings.append('Archive byte limit: skipped ' + name)
                        return False
                    archive.writestr(name, data)
                    members.add(name)
                    total += len(data)
                    inventory.append({'path': name, 'bytes': len(data), 'sha256': sha(data), 'role': role})
                    return True

                add('README.md', tile_debug_readme(target).encode('utf-8'), 'scope')
                report['references_packaged'] = {}
                for name, data in references.items():
                    report['references_packaged'][name] = add(name, data, 'verified-reference')
                report['packaged_products'] = []
                for name in ('runner-status.json', 'runner-result.json', 'generator.log',
                             *target.products, 'meshes/map' + target.map_name + '.map'):
                    row = {'path': name, 'included': False}
                    try:
                        path = inside(output, name)
                        data = read_limited(path, DEBUG_MAX_FILE)
                        if name in target.products:
                            expected = next((p for p in report.get('runner', {}).get('products', [])
                                             if p['path'] == name), None)
                            if (not expected or len(data) != expected.get('bytes')
                                    or sha(data) != expected.get('sha256')):
                                raise ValueError('Required generation output changed or lacks its verified snapshot: ' + name)
                        row.update(bytes=len(data), sha256=sha(data),
                                   included=add(name, data, 'isolated-generation-output'))
                    except (OSError, ValueError) as exc:
                        row['error'] = str(exc)
                        if name in target.products or name == 'generator.log':
                            warnings.append('Diagnostic artifact unavailable or too large: ' + name)
                    report['packaged_products'].append(row)
                report['staged_input_packaging'] = []
                for item in staged:
                    entry = {'path': item['name'], 'included': False}
                    try:
                        data = read_limited(item['path'], DEBUG_MAX_FILE)
                        if len(data) != item['bytes'] or sha(data) != item['sha256']:
                            raise ValueError('Staged pre-run input evidence changed: ' + item['name'])
                        entry['included'] = add(item['name'], data, item['role'])
                    except (OSError, ValueError) as exc:
                        entry['error'] = str(exc)
                        warnings.append('Pre-run input evidence unavailable at packaging: ' + item['name'])
                    report['staged_input_packaging'].append(entry)

                unchanged = True
                report['final_snapshot_checks'] = {}
                for label, path, member in (('manifest', manifest_path, 'published/manifest.json'),
                                             ('provenance', provenance_path, 'published/sources/provenance.json')):
                    if path is None or member not in references:
                        unchanged = False
                        report['final_snapshot_checks'][label] = {'checked': False}
                        continue
                    try:
                        current = read_limited(path)
                        same = current == references[member]
                        report['final_snapshot_checks'][label] = {'checked': True, 'unchanged': same,
                                                                   'after_sha256': sha(current)}
                        if not same:
                            unchanged = False
                            warnings.append('Published ' + label + ' changed during the run; baseline is not verified.')
                            add('published/' + label + '.after.json', current, 'changed-reference')
                    except (OSError, ValueError) as exc:
                        unchanged = False
                        report['final_snapshot_checks'][label] = {'checked': False, 'error': str(exc)}
                if target_path is not None and 'target_record' in report:
                    try:
                        after = read_limited(target_path)
                        same = sha(after) == target.tile_sha256 and len(after) == report['target_record']['bytes']
                        report['final_snapshot_checks']['published_target'] = {
                            'checked': True, 'unchanged': same, 'after_sha256': sha(after)}
                        if not same:
                            unchanged = False
                            warnings.append('Published target tile changed during the run; baseline is not verified.')
                    except (OSError, ValueError) as exc:
                        unchanged = False
                        report['final_snapshot_checks']['published_target'] = {'checked': False, 'error': str(exc)}
                if published_mmap_path is not None and 'published/mmaps/' + target.mmap_name in references:
                    try:
                        after = read_limited(published_mmap_path)
                        same = after == references['published/mmaps/' + target.mmap_name]
                        report['final_snapshot_checks']['published_mmap'] = {'checked': True, 'unchanged': same}
                        if not same:
                            unchanged = False
                            warnings.append('Published target-map header changed during the run; baseline is not verified.')
                    except (OSError, ValueError) as exc:
                        unchanged = False
                        report['final_snapshot_checks']['published_mmap'] = {'checked': False, 'error': str(exc)}
                report['published_snapshots_unchanged'] = unchanged
                report['source_evidence_complete'] = bool(report['source_evidence_complete']
                    and all(row['included'] for row in report['staged_input_packaging']))
                report['verified_baseline'] = bool(report['verified_baseline'] and unchanged)
                required_names = {*target.products, 'runner-result.json', 'generator.log'}
                report['packaged_required_outputs_complete'] = all(
                    next((p['included'] for p in report['packaged_products'] if p['path'] == name), False)
                    for name in required_names)
                report['collection_complete'] = bool(report['verified_baseline']
                    and report['source_evidence_complete'] and report['packaged_required_outputs_complete']
                    and all(report['references_packaged'].values()))
                case_bytes = json_bytes(report)
                if not add('case.json', case_bytes, 'diagnostic-report', essential=True):
                    raise ValueError('Could not include diagnostic report within budget.')
                listing = json_bytes({'schema': 1, 'members': inventory,
                                      'uncompressed_bytes_excluding_inventory': total})
                if len(listing) > DEBUG_MAX_FILE or total + len(listing) > DEBUG_MAX_TOTAL:
                    raise ValueError('Diagnostic inventory exceeds budget.')
                archive.writestr('inventory.json', listing)
            stream.flush()
            os.fsync(stream.fileno())
        if os.name == 'nt':
            os.rename(temporary, final)
        else:
            os.link(temporary, final)
            temporary.unlink()
        return {'path': str(final), 'output_directory': str(output),
                'transport_revision': TRANSPORT_REVISION,
                'verified_baseline': report['verified_baseline'],
                'source_evidence_complete': report['source_evidence_complete'],
                'generator_started': report['generator_started'],
                'collection_complete': report['collection_complete'],
                'packaged_required_outputs_complete': report['packaged_required_outputs_complete'],
                'error': report.get('error') or report.get('runner', {}).get('error'),
                'warnings': warnings}
    finally:
        temporary.unlink(missing_ok=True)


def main():
    say('BRIDGE SOURCE ' + DEFAULT_TARGET.stem + '：启动，准备单瓦片基线采集。')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', '--project-root', dest='root', type=Path, default=Path(r'D:\wangzekun\Documents\AutonomousDriving'))
    parser.add_argument('--distribution', default='Ubuntu')
    args = parser.parse_args()
    if os.name != 'nt':
        parser.exit(1, 'This entrypoint targets the existing Windows + WSL installation.\n')
    try:
        result = collect_tile_debug(args.root, args.distribution)
    except (OSError, ValueError, KeyError, RuntimeError, TypeError) as exc:
        parser.exit(1, 'Collection failed: ' + str(exc) + '\n')
    say(json.dumps(result, ensure_ascii=False, indent=2))
    ready = result['collection_complete']
    say('BRIDGE_SOURCE_READY' if ready else 'BRIDGE_SOURCE_INCOMPLETE')
    if result.get('error'):
        say('ERROR = ' + result['error'])
    say('ZIP = ' + result['path'])
    say('请上传这个 ZIP。' if ready else '本次证据不完整；请上传这个 ZIP，其中保留了原因和已有证据。')
    return 0 if ready else 2


if __name__ == '__main__':
    raise SystemExit(main())

