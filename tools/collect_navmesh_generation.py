"""Collect generation evidence for 0000_50_34.

The default mode is read-only collection. Explicit --tile-debug runs one
baseline tile with the original parameters into a new isolated runtime folder.
This file uses the Python standard library and does not import project modules.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import struct
import subprocess
import tempfile
import uuid
import zipfile

MAX_FILE = 32 * 1024 * 1024
MAX_TOTAL = 128 * 1024 * 1024
LOG_LIMIT = 1024 * 1024
TARGET = (0, 50, 34)
TARGET_NAME = '0000_50_34.mmtile'
TARGET_SHA = '616d2cf2905a2868ee83f3a12aa60540d04b73b2186d895033084dfc3e95fd79'
DEBUG_TOOL_SHA = 'cdd490271d715b945ea214da5b787ef366ac3b5c49b9c3e89c769f52f5a33451'
DEBUG_MAX_FILE = 64 * 1024 * 1024
DEBUG_MAX_TOTAL = 192 * 1024 * 1024
DEBUG_TIMEOUT = 600
TRANSPORT_REVISION = '2-wsl-exec-stdin-json'
DEBUG_PRODUCTS = ('mmaps/0000.mmap', 'mmaps/0000_50_34.mmtile',
                  'meshes/map0000_50_34.obj', 'meshes/map0000_50_34.mesh',
                  'meshes/0000_50_34.pmesh', 'meshes/0000_50_34.dmesh')
SOURCE_NAMES = ('maps-extract.log', 'collision-extract.log', 'collision-assemble.log',
                'metadata-extract.log', 'nav-0.log', 'nav-1.log',
                'old-geometry-comparison.json')
README = """# Navigation generation evidence, target 0000_50_34

The collector reads existing files. It does not execute mmaps_generator, load
the query DLL, build code, change settings, extract game assets, or regenerate
maps. Only this archive is written into the project's runtime directory.

The 49 query tiles and Windows DLL from the earlier reproduction are NOT copied.
case.json verifies the active target tile and describes any matching original
work directory. Only the selected 3x3 original terrain .map files are copied.
The vmaps listing is an inventory, not a complete collision dependency set.
This archive is not a complete, runnable generation input or navigation pack.
Matching a directory's provenance and output tile establishes association, not
historical integrity of its raw inputs. Raw map hashes are measured now; their
build headers are compared with the published client build when available.

Published source logs are allowlisted. Large logs contain the beginning and
end only; case.json records the original length and the omitted byte interval.
Missing inputs, skipped files, limits, and source/tool mismatches are explicit.

WSL source snapshots come from the recorded Git revision if it exists locally.
Current HEAD, a restricted raw change list (not a complete patch), and current
allowlisted working files are
recorded separately. A locally available banner revision may also be included.
The generator binary is read as bytes only after checking its ELF magic.
The recorded source revision, banner revision, and binary hash are independent
evidence: none is asserted to be the source used to compile that binary.

Build metadata contains allowlisted CMake variables and matching compilation
records, never whole application configuration or database configuration.
No network fetch, Git checkout/reset, build, or generator invocation is used.
WSL discovery is shallow and limited to the known source/tool parent paths.

inventory.json records SHA256 and bytes for every member except itself.
Read case.json and wsl/report.json before assuming any requested evidence exists.
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


def limited_log(path):
    size = path.stat().st_size
    with path.open('rb') as handle:
        if size <= LOG_LIMIT:
            data = handle.read(LOG_LIMIT + 1)
            if len(data) > LOG_LIMIT:
                raise ValueError('Log changed while reading: ' + str(path))
            return data, {'original_bytes': size, 'truncated': False, 'sha256': sha(data)}
        half = LOG_LIMIT // 2
        first = handle.read(half)
        handle.seek(max(half, size - half))
        last = handle.read(half)
    marker = ('\n\n[COLLECTOR: omitted original byte interval %d..%d; '
              'this is a head/tail excerpt]\n\n' % (half, size - half)).encode('ascii')
    return first + marker + last, {'original_bytes': size, 'truncated': True,
                                 'omitted_byte_interval': [half, size - half],
                                 'original_sha256': None}


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


def target_log_excerpt(path):
    """Bounded text search, retaining both possible pair orders as evidence."""
    scan_limit, output_limit, line_limit = 32 * 1024 * 1024, 256 * 1024, 64 * 1024
    pattern = re.compile(rb'0000_50_34|(?<![0-9])(?:50\s*[,/_]\s*34|34\s*[,/_]\s*50)(?![0-9])')
    info = {'scope': 'nav-0.log text matches, not verified generator coordinate semantics',
            'pattern': pattern.pattern.decode('ascii'), 'scan_limit_bytes': scan_limit,
            'output_limit_bytes': output_limit, 'line_read_limit_bytes': line_limit,
            'matching_chunks': 0, 'scanned_bytes': 0, 'original_bytes': path.stat().st_size}
    chunks, previous, after, output_size = [], [], 0, 0
    emitted = set()
    with path.open('rb') as handle:
        while info['scanned_bytes'] < scan_limit:
            offset = info['scanned_bytes']
            line = handle.readline(min(line_limit, scan_limit - offset))
            if not line:
                break
            info['scanned_bytes'] += len(line)
            hit = pattern.search(line) is not None
            if hit:
                info['matching_chunks'] += 1
                after = 3
            chosen = previous + [(offset, line)] if hit else [(offset, line)] if after else []
            for pos, text in chosen:
                if pos in emitted:
                    continue
                encoded = ('[original byte %d] ' % pos).encode('ascii') + text
                if output_size + len(encoded) > output_limit:
                    info['output_truncated'] = True
                    break
                chunks.append(encoded)
                emitted.add(pos)
                output_size += len(encoded)
            if info.get('output_truncated'):
                break
            if after and not hit:
                after -= 1
            previous = (previous + [(offset, line)])[-3:]
    info['scan_complete'] = info['scanned_bytes'] >= info['original_bytes']
    info['long_lines_may_be_split'] = True
    return b''.join(chunks), info


def tile_metadata(data):
    result = {'bytes': len(data), 'sha256': sha(data), 'first_120_bytes_hex': data[:120].hex()}
    if len(data) >= 120:
        result.update(mmap_magic_version=list(struct.unpack_from('<III', data)),
                      mesh_x=struct.unpack_from('<i', data, 28)[0],
                      mesh_y=struct.unpack_from('<i', data, 32)[0],
                      agent_height_radius_climb=list(struct.unpack_from('<3f', data, 80)))
    return result


def find_original_workdirs(root, provenance_bytes, target_sha, warnings):
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
                tile = read_limited(inside(candidate, 'mmaps/' + TARGET_NAME))
                entry['target_sha256'] = sha(tile)
                entry['match'] = entry['target_sha256'] == target_sha
                if entry['match']:
                    matches.append(candidate)
        except (OSError, ValueError, TypeError) as exc:
            entry['error'] = str(exc)
        found.append(entry)
    return found, matches


def vmap_listing(work, warnings):
    directory = work / 'vmaps'
    report = {'directory': str(directory), 'complete_model_dependencies': False, 'files': []}
    if not directory.is_dir():
        report['missing'] = True
        warnings.append('Matched original work directory has no vmaps directory.')
        return report
    allowed = directory.resolve()
    if not allowed.is_relative_to(work.resolve()):
        report['skipped'] = 'vmaps points outside matched work directory'
        warnings.append(report['skipped'])
        return report
    report['scope'] = 'One directory level only; subdirectories are listed, not traversed.'
    try:
        entries = sorted(directory.iterdir(), key=lambda p: p.name)
    except OSError as exc:
        report['error'] = str(exc)
        warnings.append('Unable to list matched vmaps directory: ' + str(exc))
        return report
    for index, item in enumerate(entries):
        if index >= 20000:
            report['truncated'] = True
            warnings.append('vmaps listing stopped at 20000 entries.')
            break
        if item.is_symlink() or not item.resolve().is_relative_to(allowed):
            report['files'].append({'name': item.name, 'skipped': 'symlink or external path'})
        else:
            report['files'].append({'name': item.name, 'is_directory': item.is_dir(),
                                    'bytes': item.stat().st_size if item.is_file() else None})
    return report


# Executed by WSL python3 on stdin. It only reads permitted files and emits an
# in-memory ZIP on stdout; it never creates files in the Linux filesystem.
WSL_WORKER = r'''
import hashlib, io, json, os, pathlib, re, subprocess, sys, zipfile
P = PAYLOAD
LIMIT = 32 * 1024 * 1024
BUDGET = min(P['budget'], 88 * 1024 * 1024)
DIRS = ('src/tools/mmaps_generator', 'src/common/mmaps_common',
        'dep/recastnavigation/Recast', 'dep/recastnavigation/Detour',
        'dep/recastnavigation/DebugUtils', 'src/common/Collision')
EXTRA = ('CMakeLists.txt', 'src/CMakeLists.txt', 'src/tools/CMakeLists.txt',
         'src/common/CMakeLists.txt', 'dep/CMakeLists.txt',
         'dep/recastnavigation/CMakeLists.txt', 'COPYING', 'LICENSE',
         'License.txt', 'dep/recastnavigation/License.txt')
SUFFIX = {'.h', '.hh', '.hpp', '.hxx', '.c', '.cc', '.cpp', '.cxx', '.inl', '.inc', '.cmake'}
report = {'warnings': [], 'requested_source': P.get('source'),
          'requested_revision': P.get('revision'), 'source_is_binary_build_proof': False,
          'snapshots': [], 'working_files': [], 'cmake': [], 'other_provenance_paths': []}
store = io.BytesIO()
archive = zipfile.ZipFile(store, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=5)
total = 0
names = set()
def digest(data): return hashlib.sha256(data).hexdigest()
def safe(name):
    if not name or '\\' in name or '\x00' in name or any(x in ('', '.', '..') or ':' in x for x in name.split('/')):
        raise ValueError('unsafe archive path')
    return name
def add(name, data):
    global total
    safe(name)
    if name in names: raise ValueError('duplicate archive path: ' + name)
    if len(data) > LIMIT or total + len(data) > BUDGET - 1024 * 1024:
        report['warnings'].append('Byte limit: skipped ' + name)
        return False
    archive.writestr(name, data)
    names.add(name)
    total += len(data)
    return True
def read(path, limit=LIMIT):
    if path.stat().st_size > limit: raise ValueError('file too large: ' + str(path))
    with path.open('rb') as handle: data = handle.read(limit + 1)
    if len(data) > limit: raise ValueError('file grew beyond limit')
    return data
def permitted(name):
    if name in EXTRA: return True
    if not any(name.startswith(d + '/') for d in DIRS): return False
    path = pathlib.PurePosixPath(name)
    return path.suffix.lower() in SUFFIX or path.name == 'CMakeLists.txt' or path.name.lower().startswith(('license', 'copying'))
def within(path, base):
    try: path.resolve().relative_to(base.resolve()); return True
    except (ValueError, OSError): return False
env = os.environ.copy()
env['GIT_OPTIONAL_LOCKS'] = '0'
env['GIT_TERMINAL_PROMPT'] = '0'
env['GIT_PAGER'] = 'cat'
env.pop('GIT_EXTERNAL_DIFF', None)
source = pathlib.Path(P['source']) if P.get('source') else None
def git(*args):
    cmd = ['git', '-c', 'core.fsmonitor=false', '--no-pager', '-C', str(source)] + list(args)
    completed = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30, env=env)
    if completed.returncode:
        raise RuntimeError('git ' + args[0] + ': ' + completed.stderr[:800].decode('utf-8', 'replace').strip())
    return completed.stdout
def snapshot(revision, label):
    details = {'label': label, 'requested_revision': revision, 'files': 0, 'missing_directories': []}
    report['snapshots'].append(details)
    if not re.fullmatch(r'[0-9a-fA-F]{7,40}', str(revision or '')):
        details['error'] = 'revision is missing or is not a hexadecimal Git revision'
        report['warnings'].append(label + ': ' + details['error'])
        return
    revision = git('rev-parse', '--verify', str(revision) + '^{commit}').decode().strip()
    if not re.fullmatch(r'[0-9a-f]{40}', revision): raise ValueError('unexpected resolved revision')
    details['resolved_revision'] = revision
    tree = git('ls-tree', '-r', '-l', '-z', revision, '--', *DIRS, *EXTRA)
    present = set()
    for entry in tree.split(b'\0'):
        if not entry: continue
        meta, path_bytes = entry.split(b'\t', 1)
        mode, kind, oid, size = meta.decode('ascii').split()
        name = path_bytes.decode('utf-8', 'strict')
        for d in DIRS:
            if name.startswith(d + '/'): present.add(d)
        if not permitted(name): continue
        if mode == '120000' or kind != 'blob':
            report['warnings'].append('Skipped Git symlink/non-blob: ' + name)
            continue
        if int(size) > LIMIT or total + int(size) > BUDGET - 1024 * 1024:
            report['warnings'].append('Snapshot byte limit: skipped ' + name)
            continue
        data = git('cat-file', 'blob', oid)
        if len(data) != int(size): raise ValueError('Git blob length mismatch')
        if add(label + '/' + safe(name), data): details['files'] += 1
    details['missing_directories'] = [d for d in DIRS if d not in present]
    for d in details['missing_directories']:
        report['warnings'].append(label + ': missing source directory ' + d)

# Capture the installed binary separately, without executing it or following
# an unexpected symlink outside the declared tools directory.
tools_dir = pathlib.Path(P.get('tools') or '/home/wangzekun/forever/server-integrated/bin')
binary = tools_dir / 'mmaps_generator'
report['generator'] = {'path': str(binary), 'executed': False, 'expected_sha256': P.get('tool_sha256')}
try:
    if binary.is_symlink() or not within(binary, tools_dir): raise ValueError('generator is a symlink or outside tools directory')
    with binary.open('rb') as handle: magic = handle.read(4)
    if magic != b'\x7fELF': raise ValueError('generator does not have ELF magic; not collected')
    binary_data = read(binary)
    if binary_data[:4] != b'\x7fELF': raise ValueError('generator changed while reading; ELF magic no longer matches')
    actual_hash = digest(binary_data)
    report['generator'].update(bytes=len(binary_data), sha256=actual_hash,
                               matches_provenance=(actual_hash == P.get('tool_sha256')) if P.get('tool_sha256') else None)
    if P.get('tool_sha256') and actual_hash != P['tool_sha256']:
        report['warnings'].append('Generator SHA256 differs from published provenance; this is a different current binary.')
    elif not P.get('tool_sha256'):
        report['warnings'].append('Published provenance has no generator SHA256 to compare.')
    report['generator']['included'] = add('tools/mmaps_generator', binary_data)
except (OSError, ValueError) as exc:
    report['generator']['error'] = str(exc)
    report['warnings'].append('Generator unavailable: ' + str(exc))

if source is None or not source.is_absolute() or not source.is_dir():
    report['warnings'].append('Recorded source directory is unavailable; no source snapshot collected.')
else:
    source = source.resolve()
    report['source_directory'] = str(source)
    try:
        report['current_head'] = git('rev-parse', '--verify', 'HEAD').decode().strip()
        snapshot(P.get('revision'), 'source-recorded-revision')
    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
        report['warnings'].append('Recorded source snapshot failed: ' + str(exc))
    banner = re.search(r'\b([0-9a-fA-F]{12,40})\b', str(P.get('banner') or ''))
    if banner:
        report['tool_banner_revision'] = banner.group(1)
        try:
            resolved = git('rev-parse', '--verify', banner.group(1) + '^{commit}').decode().strip()
            report['tool_banner_revision_resolved'] = resolved
            recorded = report['snapshots'][0].get('resolved_revision') if report['snapshots'] else None
            if resolved != recorded: snapshot(resolved, 'source-banner-revision')
        except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
            report['warnings'].append('Banner revision is not available as a local source snapshot: ' + str(exc))
    else:
        report['warnings'].append('Tool banner revision is absent; no matching banner source snapshot was attempted.')
    try:
        tracked = [x.decode('utf-8', 'strict') for x in git('ls-files', '-z', '--', *DIRS, *EXTRA).split(b'\0') if x]
        tracked = [name for name in tracked if permitted(name)]
        diff = git('diff', '--raw', '--no-ext-diff', '--no-textconv', 'HEAD', '--', *tracked) if tracked else b''
        report['restricted_change_list_format'] = 'git diff --raw HEAD; allowlisted files; not a complete patch'
        report['restricted_change_list_sha256'] = digest(diff)
        report['restricted_change_list_bytes'] = len(diff)
        report['change_list_hash_comparable_to_provenance_diff_hash'] = False
        add('source-current-change-list.txt', diff)
        # Read only relevant changed/untracked worktree files; the recorded
        # revision snapshot remains separate. Symlinks are never followed.
        selected = set()
        queries = [('ls-files', '--others', '--exclude-standard', '-z', '--', *DIRS, *EXTRA)]
        if tracked: queries.append(('diff', '--name-only', '-z', '--no-ext-diff', '--no-textconv', 'HEAD', '--', *tracked))
        for args in queries:
            selected.update(x.decode('utf-8', 'strict') for x in git(*args).split(b'\0') if x)
        # When HEAD moved, include the current allowed files as a distinct
        # snapshot rather than presenting an empty HEAD diff as unchanged.
        if report.get('current_head') != P.get('revision'):
            selected.update(x.decode('utf-8', 'strict') for x in git('ls-files', '-z', '--', *DIRS, *EXTRA).split(b'\0') if x)
        for name in sorted(selected):
            if not permitted(name): continue
            safe(name)
            path = source.joinpath(*pathlib.PurePosixPath(name).parts)
            row = {'path': name}
            try:
                if (any(source.joinpath(*pathlib.PurePosixPath(name).parts[:i]).is_symlink()
                        for i in range(1, len(pathlib.PurePosixPath(name).parts) + 1))
                        or not within(path, source)): raise ValueError('symlink or external path')
                if not path.is_file(): row['missing_or_deleted'] = True
                else:
                    data = read(path)
                    row.update(bytes=len(data), sha256=digest(data), included=add('source-current-working/' + name, data))
            except (OSError, ValueError) as exc: row['error'] = str(exc)
            report['working_files'].append(row)
    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
        report['warnings'].append('Current restricted worktree inspection failed: ' + str(exc))

def shallow(base, maximum=256):
    if not base.is_dir(): return []
    result = [base]
    first = []
    try:
        for p in sorted(base.iterdir(), key=str):
            if p.is_dir() and not p.is_symlink() and within(p, base):
                first.append(p); result.append(p)
                if len(result) >= maximum: return result
        for p in first:
            for q in sorted(p.iterdir(), key=str):
                if q.is_dir() and not q.is_symlink() and within(q, base):
                    result.append(q)
                    if len(result) >= maximum: return result
    except OSError as exc: report['warnings'].append('Shallow discovery: ' + str(exc))
    return result

parent = pathlib.Path('/home/wangzekun/forever')
directories = shallow(parent)
if source and source.is_dir():
    directories = list(dict.fromkeys(directories + shallow(source)))
keys = {'CMAKE_HOME_DIRECTORY', 'CMAKE_CACHEFILE_DIR', 'CMAKE_BUILD_TYPE',
        'CMAKE_C_COMPILER', 'CMAKE_CXX_COMPILER', 'CMAKE_MAKE_PROGRAM',
        'CMAKE_GENERATOR', 'CMAKE_GENERATOR_PLATFORM', 'CMAKE_GENERATOR_TOOLSET',
        'CMAKE_INSTALL_PREFIX', 'CMAKE_TOOLCHAIN_FILE', 'CMAKE_SYSTEM_NAME',
        'CMAKE_CXX_FLAGS', 'CMAKE_CXX_FLAGS_RELEASE', 'CMAKE_CXX_FLAGS_DEBUG',
        'CMAKE_CXX_FLAGS_RELWITHDEBINFO', 'CMAKE_C_FLAGS', 'TOOLS'}
for directory in directories:
    cache = directory / 'CMakeCache.txt'
    if cache.is_file() and not cache.is_symlink() and within(cache, directory):
        try:
            fields = {}
            for line in read(cache, 4 * 1024 * 1024).decode('utf-8', 'replace').splitlines():
                match = re.match(r'^([A-Za-z0-9_]+):[^=]*=(.*)$', line)
                if match and match.group(1) in keys: fields[match.group(1)] = match.group(2)
            if source and fields.get('CMAKE_HOME_DIRECTORY') and pathlib.Path(fields['CMAKE_HOME_DIRECTORY']).resolve() == source:
                item = {'path': str(cache), 'variables': fields, 'raw_cache_included': False}
                commands = directory / 'compile_commands.json'
                if commands.is_file() and not commands.is_symlink() and within(commands, directory):
                    selected = []
                    for row in json.loads(read(commands).decode('utf-8-sig')):
                        file = pathlib.Path(row.get('file', ''))
                        if not file.is_absolute(): file = pathlib.Path(row.get('directory', directory)) / file
                        try: rel = file.resolve().relative_to(source).as_posix()
                        except (ValueError, OSError): continue
                        if permitted(rel):
                            selected.append({k: row[k] for k in ('directory', 'file', 'arguments', 'command') if k in row})
                        if len(selected) >= 512:
                            item['compile_commands_truncated'] = True; break
                    member = 'build/compile-commands-%02d.json' % len(report['cmake'])
                    item['compile_commands_member'] = member
                    item['compile_commands_included'] = add(member, json.dumps(selected, indent=2).encode('utf-8'))
                report['cmake'].append(item)
        except (OSError, ValueError, TypeError) as exc:
            report['warnings'].append('Build metadata unavailable: ' + str(cache) + ': ' + str(exc))
    if (P.get('locate_original') and within(directory, parent)
            and len(directory.resolve().relative_to(parent.resolve()).parts) <= 2):
        p = directory / 'provenance.json'
        if p.is_file() and not p.is_symlink() and within(p, parent):
            try:
                data = read(p)
                report['other_provenance_paths'].append({'path': str(p), 'bytes': len(data), 'sha256': digest(data)})
            except (OSError, ValueError) as exc: report['warnings'].append(str(exc))
if not report['cmake']:
    report['warnings'].append('No matching CMakeCache metadata found within the bounded discovery scope.')
report['discovery_scope'] = {'parent': str(parent), 'depth': 2, 'directory_limit_per_root': 256,
                             'source_root_also_checked': str(source) if source else None,
                             'full_filesystem_search': False}
archive.writestr('report.json', json.dumps(report, ensure_ascii=False, indent=2).encode('utf-8'))
archive.close()
sys.stdout.buffer.write(store.getvalue())
'''


def wsl_evidence(payload, distribution):
    program = 'PAYLOAD = ' + repr(payload) + '\n' + WSL_WORKER
    result = subprocess.run(['wsl.exe', '-d', distribution, '--exec', 'python3', '-B', '-'],
                            input=program.encode('utf-8'), stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=240)
    if result.returncode:
        raise RuntimeError('WSL worker failed: ' + result.stderr[:1600].decode('utf-8', 'replace'))
    if not result.stdout.startswith(b'PK\x03\x04'):
        raise RuntimeError('WSL worker did not return its evidence ZIP.')
    return result.stdout


def collect_case(root, distribution='Ubuntu'):
    root = Path(root).resolve()
    runtime = (root / 'runtime').resolve()
    if not runtime.is_relative_to(root):
        raise ValueError('runtime points outside project root.')
    manifest_path = inside(root, 'data/navigation/manifest.json')
    nav_root = manifest_path.parent
    manifest_bytes = read_limited(manifest_path)
    manifest = json.loads(manifest_bytes.decode('utf-8-sig'))
    if manifest.get('kind') != 'adrive-ground-navmesh' or manifest.get('mmap_version') != 16:
        raise ValueError('Unexpected navigation manifest format.')
    targets = [t for t in manifest['tiles'] if (t.get('c'), t.get('row'), t.get('col')) == TARGET]
    if len(targets) != 1 or targets[0].get('sha256') != TARGET_SHA:
        raise ValueError('Active target 0000_50_34 does not match the diagnosed tile; collection stopped.')
    target_record = targets[0]
    target = read_limited(inside(nav_root, target_record['path']))
    if sha(target) != TARGET_SHA or len(target) != target_record.get('bytes'):
        raise ValueError('Active target tile bytes/hash differ from the manifest.')
    provenance_path = inside(nav_root, manifest['provenance'])
    provenance_bytes = read_limited(provenance_path)
    provenance = json.loads(provenance_bytes.decode('utf-8-sig'))
    warnings = []
    candidates, matches = find_original_workdirs(root, provenance_bytes, TARGET_SHA, warnings)
    work = matches[0] if matches else None
    if not matches:
        warnings.append('No original work directory matched both published provenance and target tile hash in Windows runtime.')
    elif len(matches) > 1:
        warnings.append('Multiple equivalent work directories matched; only the first sorted directory supplies raw maps. See candidates.')
    report = {'schema': 1, 'purpose': 'generation evidence, not a complete reproduction input',
              'root': str(root), 'distribution': distribution,
              'active_manifest_sha256': sha(manifest_bytes), 'target_key': list(TARGET),
              'target_record': target_record, 'target_metadata': tile_metadata(target),
              'source_provenance_path': str(provenance_path),
              'source_provenance_sha256': sha(provenance_bytes),
              'runtime_search_depth': 2, 'runtime_candidates': candidates,
              'matched_work_directories': [str(p) for p in matches],
              'selected_work_directory': str(work) if work else None,
              'published_files': [], 'raw_maps': [], 'warnings': warnings,
              'generator_executed': False, 'build_executed': False,
              'original_inputs_complete': False}
    runtime.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    final = runtime / ('navmesh-generation-case-' + stamp + '-' + uuid.uuid4().hex[:8] + '.zip')
    fd, temporary_name = tempfile.mkstemp(prefix='.navmesh-generation-', suffix='.tmp', dir=runtime)
    temporary = Path(temporary_name)
    inventory, names, total = [], set(), 0
    try:
        with os.fdopen(fd, 'w+b') as stream:
            with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
                def add(name, data, role, essential=False):
                    nonlocal total
                    name = relative(name)
                    if name in names:
                        raise ValueError('Duplicate archive member: ' + name)
                    if len(data) > MAX_FILE or total + len(data) > MAX_TOTAL - (0 if essential else 2 * 1024 * 1024):
                        warnings.append('Archive byte limit: skipped ' + name)
                        return False
                    archive.writestr(name, data)
                    names.add(name)
                    total += len(data)
                    inventory.append({'path': name, 'bytes': len(data), 'sha256': sha(data), 'role': role})
                    return True

                add('README.md', README.encode('utf-8'), 'scope')
                add('published/manifest.json', manifest_bytes, 'active-manifest')
                add('published/sources/provenance.json', provenance_bytes, 'published-provenance')
                for name in SOURCE_NAMES:
                    row = {'name': name}
                    try:
                        path = inside(provenance_path.parent, name)
                        data, info = limited_log(path)
                        row.update(info)
                        row['included'] = add('published/sources/' + name, data, 'source-log-or-excerpt')
                        if name == 'nav-0.log':
                            excerpt, excerpt_info = target_log_excerpt(path)
                            row['target_excerpt'] = excerpt_info
                            row['target_excerpt']['included'] = add('published/sources/nav-0-target-excerpt.txt',
                                                                  excerpt, 'bounded-target-log-excerpt')
                    except (OSError, ValueError) as exc:
                        row['error'] = str(exc)
                        warnings.append('Published source file unavailable: ' + name)
                    report['published_files'].append(row)

                if work:
                    add('original-work/provenance.json', read_limited(inside(work, 'provenance.json')), 'matched-work-provenance')
                    client_version = str(provenance.get('client', {}).get('version', ''))
                    expected_build = int(client_version.rsplit('.', 1)[-1]) if client_version.rsplit('.', 1)[-1].isdigit() else None
                    report['raw_input_historical_hashes_available'] = False
                    for row in range(49, 52):
                        for col in range(33, 36):
                            name = '0000_%02d_%02d.map' % (row, col)
                            info = {'name': name, 'c': 0, 'row': row, 'col': col}
                            try:
                                data = read_limited(inside(work, 'maps/' + name))
                                info.update(bytes=len(data), sha256=sha(data), first_64_bytes_hex=data[:64].hex())
                                build = struct.unpack_from('<I', data, 8)[0] if len(data) >= 12 else None
                                info.update(header_client_build=build, expected_client_build=expected_build,
                                            matches_published_client=(build == expected_build) if expected_build is not None else None)
                                if expected_build is None or build != expected_build:
                                    warnings.append('Raw terrain client build cannot be confirmed or differs: ' + name)
                                info['included'] = add('original-work/maps/' + name, data, 'raw-terrain')
                            except (OSError, ValueError) as exc:
                                info['error'] = str(exc)
                                warnings.append('Raw terrain file unavailable: ' + name)
                            report['raw_maps'].append(info)
                    add('original-work/vmaps-listing.json', json_bytes(vmap_listing(work, warnings)), 'collision-file-listing')

                server = provenance.get('server', {})
                if not isinstance(server, dict):
                    server = {}
                tool_hashes = server.get('tools_sha256', provenance.get('tools_sha256', {}))
                if not isinstance(tool_hashes, dict):
                    tool_hashes = {}
                payload = {'source': server.get('source') or provenance.get('server_source'),
                           'revision': server.get('revision') or provenance.get('server_revision'),
                           'tools': provenance.get('tools') or '/home/wangzekun/forever/server-integrated/bin',
                           'tool_sha256': tool_hashes.get('mmaps_generator'),
                           'banner': server.get('tool_banner_revision') or provenance.get('tool_banner_revision'),
                           'locate_original': not matches, 'budget': MAX_TOTAL - total - 2 * 1024 * 1024}
                print('Reading bounded WSL source/tool evidence; no generator or build will run...', flush=True)
                try:
                    worker_bytes = wsl_evidence(payload, distribution)
                    with zipfile.ZipFile(io.BytesIO(worker_bytes)) as worker:
                        entries = worker.infolist()
                        if len(entries) > 10000 or sum(i.file_size for i in entries) > MAX_TOTAL:
                            raise ValueError('WSL evidence exceeds archive limits.')
                        for item in entries:
                            if item.is_dir():
                                continue
                            name = relative(item.filename)
                            if item.file_size > MAX_FILE:
                                raise ValueError('WSL member exceeds file limit: ' + name)
                            data = worker.read(item)
                            if name == 'report.json':
                                worker_report = json.loads(data.decode('utf-8'))
                                report['wsl_warnings'] = worker_report.get('warnings', [])
                            add('wsl/' + name, data, 'wsl-read-only-evidence')
                    report['wsl_collection_returned'] = True
                except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired, zipfile.BadZipFile) as exc:
                    report['wsl_collection_returned'] = False
                    report['wsl_error'] = str(exc)
                    warnings.append('WSL evidence is missing or incomplete; see wsl_error.')

                if read_limited(manifest_path) != manifest_bytes:
                    raise RuntimeError('Active manifest changed during collection; no ZIP was published.')
                if read_limited(provenance_path) != provenance_bytes:
                    raise RuntimeError('Published provenance changed during collection; no ZIP was published.')
                if not add('case.json', json_bytes(report), 'collection-report', essential=True):
                    raise ValueError('Could not include collection report within limits.')
                listing = json_bytes({'schema': 1, 'members': inventory,
                                      'uncompressed_bytes_excluding_inventory': total})
                if len(listing) > MAX_FILE or total + len(listing) > MAX_TOTAL:
                    raise ValueError('Inventory exceeds archive limits.')
                archive.writestr('inventory.json', listing)
            stream.flush()
            os.fsync(stream.fileno())
        # Windows rename refuses an existing destination. On POSIX, hard-link
        # publication provides the same no-overwrite guarantee for local tests.
        if os.name == 'nt':
            os.rename(temporary, final)
        else:
            os.link(temporary, final)
            temporary.unlink()
        return {'path': str(final), 'warnings': warnings,
                'wsl_warnings': report.get('wsl_warnings', []),
                'wsl_collection_returned': report.get('wsl_collection_returned', False),
                'matched_work_directory': str(work) if work else None}
    finally:
        temporary.unlink(missing_ok=True)


TILE_DEBUG_README = """# Isolated single-tile baseline: 0000_50_34

Explicit --tile-debug executes the hash-verified installed mmaps_generator
once, using the original generation parameters plus single-tile/debug flags.
Input is an existing work directory verified against published provenance and
the diagnosed output tile hash. All new generation files and logs go to this
new runtime directory. No active manifest, published tile, input, source, DLL,
or generator installation is deliberately modified. No sweep is performed.

case.json and runner-result.json record the command, hashes, timeout, errors,
required outputs and baseline comparisons. Exit status zero alone is not
success. Differences and missing artifacts remain evidence, not a replacement
navigation pack. The final manifest/provenance snapshot checks are recorded.

The .mesh retains exact float32 input geometry. OBJ/.mesh combine liquid then
solid geometry but do NOT store their split or liquid area flags. The supplied
input evidence is needed to reconstruct those semantics. Default debug output
does not contain hf/chf/cs snapshots. Debug pmesh quantized horizontal vertices
have an extra borderSize=5 offset; account for it before geometric comparison.

inventory.json lists every archive member except itself, with SHA256 and bytes.
Files above 64 MiB or beyond the 192 MiB total budget are explicitly skipped.
The archive is diagnostic evidence only; it is never automatically installed.
"""


TILE_DEBUG_WORKER = r'''
import hashlib, json, os, pathlib, signal, struct, subprocess, sys, time
P = PAYLOAD
out = pathlib.Path(P['output'])
original = pathlib.Path(P['input'])
tool = pathlib.Path(P['tool'])
R = {'schema': 1, 'mode': 'isolated-single-tile-baseline', 'actual_input': str(original),
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
                           ('mmaps/0000_50_34.mmtile', P['target_sha256'])):
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
    command = [str(tool), '0', '--tile', '50,34', '--input', str(original), '--output', str(out),
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
                    if name == 'mmaps/0000_50_34.mmtile':
                        with path.open('rb') as f: header = f.read(120)
                        valid = (len(header) >= 120
                                 and struct.unpack_from('<III', header) == (0x4d4d4150, 7, 16)
                                 and struct.unpack_from('<I', header, 12)[0] == row['bytes'] - 20
                                 and struct.unpack_from('<II', header, 20) == (0x444e4156, 7))
                        row['header_valid'] = valid
                        row['matches_published_sha256'] = row['sha256'] == P['target_sha256']
                        row['first_120_bytes_hex'] = header.hex()
                    elif name == 'mmaps/0000.mmap':
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
                and digest_file(original / 'mmaps/0000_50_34.mmtile')['sha256'] == P['target_sha256'])
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
            for name in ('mmaps/0000.mmap', 'mmaps/0000_50_34.mmtile'))
        R['verified_baseline'] = (R['generator_started'] and R['returncode'] == 0 and not R['timed_out']
                                  and not R.get('error') and R['required_outputs_verified']
                                  and R['baseline_matches_published'] and R['input_unchanged'] is True
                                  and R['tool_unchanged'] is True)
        try: save_report('runner-result.json')
        except OSError as exc: R['report_write_error'] = str(exc)
    print(json.dumps(R, ensure_ascii=False))
'''


WSL_PATH_WORKER = r'''
import json, subprocess, sys
payload = json.load(sys.stdin)
result = subprocess.run(['wslpath', '-a', payload['windows_path']],
                        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE, timeout=20)
sys.stdout.buffer.write(result.stdout)
sys.stderr.buffer.write(result.stderr)
raise SystemExit(result.returncode)
'''


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


def run_tile_debug_worker(payload, distribution):
    program = 'PAYLOAD = ' + repr(payload) + '\n' + TILE_DEBUG_WORKER
    result = subprocess.run(['wsl.exe', '-d', distribution, '--exec', 'python3', '-B', '-'],
                            input=program.encode('utf-8'), stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=DEBUG_TIMEOUT + 120)
    if result.returncode:
        raise RuntimeError('WSL tile runner failed: ' + result.stderr[:1600].decode('utf-8', 'replace'))
    return json.loads(result.stdout.decode('utf-8', 'strict'))


# This block is merged into collect_navmesh_generation.py for single-file delivery.
# It uses that file's Path, PurePosixPath, struct, json_bytes, inside, read_limited,
# relative and sha helpers. It never executes tools or changes input files.

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


def collect_tile_input_evidence(work, add, warnings):
    """Copy the declared target's collision dependencies plus bounded metadata.

    add(name, bytes, role) returns whether an archive member was included.
    This is target-specific input evidence, not a replacement full input tree.
    Snapshot checks describe current bytes, not their original extraction date.
    """
    work = Path(work).resolve()
    report = {'schema': 1, 'source_directory': str(work), 'target': [0, 50, 34],
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
    for row in range(49, 52):
        for col in range(33, 36):
            name = 'maps/0000_%02d_%02d.map' % (row, col)
            data = record(name, 'raw-terrain', required=(row == 50 or col == 34))
            if data is not None:
                entry = report['files'][-1]
                entry['header_client_build'] = struct.unpack_from('<I', data, 8)[0] if len(data) >= 12 else None
                entry['matches_case_client_build'] = entry['header_client_build'] == 70658
                if not entry['matches_case_client_build']:
                    warnings.append('Target raw map client build differs: ' + name)
    record('maps/0000_50_34.road', 'optional-road-mask')
    for name in ('world_metadata.csv', 'offmesh.txt'):
        record(name, 'optional-generator-input')

    # Map discovery depends on tile lists and VMAP filenames. Preserve both inventories.
    discovery = {'tilelists': [], 'map0_vmap_files': [], 'dbc_locale_directories': []}
    try:
        maps = inside(work, 'maps', must_exist=False)
        candidates = sorted(p for p in maps.iterdir() if p.suffix == '.tilelist' and p.is_file())
        if len(candidates) > 1024:
            raise ValueError('Too many tilelist files for this diagnostic scope.')
        for path in candidates:
            name = path.relative_to(work).as_posix()
            discovery['tilelists'].append(path.name)
            record(name, 'map-tile-discovery', required=path.name.startswith('0000'))
    except (OSError, ValueError) as exc:
        report['errors'].append(str(exc))
        warnings.append('Could not collect complete tilelist discovery evidence.')
    try:
        directory = inside(work, 'vmaps/0000', must_exist=False)
        items = sorted(directory.iterdir(), key=lambda p: p.name)
        if len(items) > 20000:
            raise ValueError('Too many map-0 VMAP files for diagnostic inventory.')
        for path in items:
            if path.is_file():
                resolved = inside(work, 'vmaps/0000/' + path.name)
                discovery['map0_vmap_files'].append({'name': path.name, 'bytes': resolved.stat().st_size})
    except (OSError, ValueError) as exc:
        report['errors'].append(str(exc))
        warnings.append('Could not collect complete map-0 VMAP discovery evidence.')
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
    tree = record('vmaps/0000/0000.vmtree', 'vmap-tree', required=True)
    tile = record('vmaps/0000/0000_50_34.vmtile', 'vmap-target-spawns', required=True)
    idx = record('vmaps/0000/0000_50_34.vmtileidx', 'vmap-target-indices', required=True)
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
    add('inputs/collection.json', json_bytes(report), 'target-input-collection-report')
    return report


def verify_tile_input_evidence(work, report, warnings):
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
        directory = inside(work, 'vmaps/0000', must_exist=False)
        now = []
        for p in sorted(directory.iterdir(), key=lambda p: p.name):
            if p.is_file():
                resolved = inside(work, 'vmaps/0000/' + p.name)
                now.append({'name': p.name, 'bytes': resolved.stat().st_size})
        if now != discovery.get('map0_vmap_files', []):
            result['changes_or_errors'].append('Map-0 VMAP discovery inventory changed during generation.')
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


def collect_tile_debug(root, distribution='Ubuntu'):
    root = Path(root).resolve()
    if not root.is_dir():
        raise ValueError('Project root does not exist.')
    runtime = (root / 'runtime').resolve()
    if not runtime.is_relative_to(root):
        raise ValueError('runtime points outside project root.')
    runtime.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = runtime / ('navmesh-tile-debug-' + stamp + '-' + uuid.uuid4().hex[:8])
    output.mkdir(exist_ok=False)
    warnings, references = [], {}
    staged, staged_names, staged_total = [], set(), 0
    stage_root = output / 'input-evidence'
    target_path, published_mmap_path = None, None
    work, manifest_path, provenance_path = None, None, None
    report = {'schema': 1, 'mode': 'isolated-single-tile-baseline', 'root': str(root),
              'transport_revision': TRANSPORT_REVISION,
              'output_directory': str(output), 'distribution': distribution,
              'warnings': warnings, 'generator_started': False, 'verified_baseline': False,
              'production_activated': False, 'parameter_sweep': False,
              'input_evidence': None, 'file_limit': DEBUG_MAX_FILE, 'total_limit': DEBUG_MAX_TOTAL}
    try:
        manifest_path = inside(root, 'data/navigation/manifest.json')
        manifest_bytes = read_limited(manifest_path)
        references['published/manifest.json'] = manifest_bytes
        manifest = json.loads(manifest_bytes.decode('utf-8-sig'))
        if manifest.get('kind') != 'adrive-ground-navmesh' or manifest.get('mmap_version') != 16:
            raise ValueError('Unexpected navigation manifest format.')
        targets = [t for t in manifest['tiles'] if (t.get('c'), t.get('row'), t.get('col')) == TARGET]
        if len(targets) != 1 or targets[0].get('sha256') != TARGET_SHA:
            raise ValueError('Active target does not match the diagnosed 0000_50_34 tile.')
        target_path = inside(manifest_path.parent, targets[0]['path'])
        target = read_limited(target_path)
        if sha(target) != TARGET_SHA or len(target) != targets[0].get('bytes'):
            raise ValueError('Active target tile bytes/hash do not match its manifest.')
        report['target_record'] = targets[0]
        report['target_metadata'] = tile_metadata(target)
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
        candidates, matches = find_original_workdirs(root, provenance_bytes, TARGET_SHA, warnings)
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
        if sha(read_limited(inside(work, 'mmaps/' + TARGET_NAME))) != TARGET_SHA:
            raise ValueError('Matched work target changed before the run.')
        references['original-work/provenance.json'] = original_provenance
        maps = [m for m in manifest['maps'] if m.get('c') == 0]
        if len(maps) != 1:
            raise ValueError('Published map-0 header is missing or ambiguous.')
        published_mmap_path = inside(manifest_path.parent, maps[0]['path'])
        mmap = read_limited(published_mmap_path)
        if len(mmap) < 40 or len(mmap) != maps[0]['bytes'] or sha(mmap) != maps[0]['sha256']:
            raise ValueError('Published map-0 header bytes/hash do not match its manifest.')
        references['published/mmaps/0000.mmap'] = mmap
        report['actual_input'] = str(work)
        report['input_provenance_sha256'] = sha(original_provenance)
        report['expected_target_sha256'] = TARGET_SHA
        report['expected_mmap_sha256'] = sha(mmap)
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

        report['input_collection_phase'] = 'before_generator'
        report['input_collection_started_utc'] = datetime.now(timezone.utc).isoformat()
        report['input_evidence'] = collect_tile_input_evidence(work, stage_add, warnings)
        report['input_collection_finished_utc'] = datetime.now(timezone.utc).isoformat()
        payload = {'input': wsl_path(work, distribution), 'output': wsl_path(output, distribution),
                   'tool': '/home/wangzekun/forever/server-integrated/bin/mmaps_generator',
                   'tool_sha256': DEBUG_TOOL_SHA, 'target_sha256': TARGET_SHA,
                   'input_provenance_sha256': sha(original_provenance),
                   'mmap_sha256': sha(mmap), 'mmap_header_hex': mmap[:40].hex(),
                   'products': list(DEBUG_PRODUCTS), 'timeout': DEBUG_TIMEOUT, 'file_limit': DEBUG_MAX_FILE}
        report['worker_payload'] = payload
        print('Running one isolated tile baseline with debug output (up to 10 minutes)...', flush=True)
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
            report['input_stability'] = verify_tile_input_evidence(work, report['input_evidence'], warnings)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            report['input_stability'] = {'unchanged': False, 'error': str(exc)}
            warnings.append('Original input stability verification failed.')
        report['verified_baseline'] = bool(report['verified_baseline'] and report['input_stability'].get('unchanged'))
        report['input_verification_finished_utc'] = datetime.now(timezone.utc).isoformat()

    final = output / 'navmesh-tile-debug-case.zip'
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

                add('README.md', TILE_DEBUG_README.encode('utf-8'), 'scope')
                for name, data in references.items():
                    add(name, data, 'verified-reference')
                report['packaged_products'] = []
                for name in ('runner-status.json', 'runner-result.json', 'generator.log',
                             *DEBUG_PRODUCTS, 'meshes/map0000.map'):
                    row = {'path': name, 'included': False}
                    try:
                        path = inside(output, name)
                        data = read_limited(path, DEBUG_MAX_FILE)
                        row.update(bytes=len(data), sha256=sha(data),
                                   included=add(name, data, 'isolated-generation-output'))
                    except (OSError, ValueError) as exc:
                        row['error'] = str(exc)
                        if name in DEBUG_PRODUCTS or name == 'generator.log':
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
                        same = sha(after) == TARGET_SHA and len(after) == report['target_record']['bytes']
                        report['final_snapshot_checks']['published_target'] = {
                            'checked': True, 'unchanged': same, 'after_sha256': sha(after)}
                        if not same:
                            unchanged = False
                            warnings.append('Published target tile changed during the run; baseline is not verified.')
                    except (OSError, ValueError) as exc:
                        unchanged = False
                        report['final_snapshot_checks']['published_target'] = {'checked': False, 'error': str(exc)}
                if published_mmap_path is not None and 'published/mmaps/0000.mmap' in references:
                    try:
                        after = read_limited(published_mmap_path)
                        same = after == references['published/mmaps/0000.mmap']
                        report['final_snapshot_checks']['published_mmap'] = {'checked': True, 'unchanged': same}
                        if not same:
                            unchanged = False
                            warnings.append('Published map-0 header changed during the run; baseline is not verified.')
                    except (OSError, ValueError) as exc:
                        unchanged = False
                        report['final_snapshot_checks']['published_mmap'] = {'checked': False, 'error': str(exc)}
                report['published_snapshots_unchanged'] = unchanged
                report['verified_baseline'] = bool(report['verified_baseline'] and unchanged)
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
                'generator_started': report['generator_started'],
                'error': report.get('error') or report.get('runner', {}).get('error'),
                'warnings': warnings}
    finally:
        temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(r'D:\wangzekun\Documents\AutonomousDriving'))
    parser.add_argument('--distribution', default='Ubuntu')
    parser.add_argument('--tile-debug', action='store_true',
                        help='Run one verified baseline tile into a NEW isolated directory; never activate it.')
    args = parser.parse_args()
    try:
        result = (collect_tile_debug(args.root, args.distribution) if args.tile_debug
                  else collect_case(args.root, args.distribution))
    except (OSError, ValueError, KeyError, RuntimeError, TypeError) as exc:
        parser.exit(1, 'Collection failed: ' + str(exc) + '\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result.get('error'):
        print('ERROR =', result['error'])
    print('Upload this ZIP. Missing evidence and excerpts are explicitly recorded inside it.')
    print('ZIP =', result['path'])


if __name__ == '__main__':
    main()
