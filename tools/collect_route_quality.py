#!/usr/bin/env python3
"""Read-only source/road/log capture for the current route-quality investigation.

Does not import project code, load a DLL, plan a route, or contact a device.
Only the new runtime/route-quality-* output directory is written.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import stat
import sys
import uuid
import zipfile

VERSION = 'route-quality-1'
FILE_LIMIT = 32 * 1024 * 1024
LOG_LIMIT = 8 * 1024 * 1024
PACKAGE_LIMIT = 96 * 1024 * 1024
PAYLOAD_LIMIT = PACKAGE_LIMIT - 1024 * 1024
CACHE_NAMES = {'__pycache__', '.pytest_cache', '.mypy_cache', '.ruff_cache', '.venv'}


def say(message):
    print(message, flush=True)


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode('utf-8')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def within(base, path):
    base = base.resolve()
    resolved = path.resolve()
    if not resolved.is_relative_to(base):
        raise ValueError('path resolves outside the permitted directory')
    current = base
    for part in path.relative_to(base).parts:
        current = current / part
        try:
            details = current.lstat()
        except FileNotFoundError:
            break
        if stat.S_ISLNK(details.st_mode) or getattr(details, 'st_file_attributes', 0) & stat.FILE_ATTRIBUTE_REPARSE_POINT:
            raise ValueError('path contains a symbolic link or Windows reparse point')
    return resolved


def log_tail(path):
    """Return at most LOG_LIMIT bytes, retaining only newline-terminated lines."""
    with path.open('rb') as stream:
        before = os.fstat(stream.fileno())
        size = before.st_size
        lower = max(0, size - LOG_LIMIT)
        stream.seek(max(0, lower - 1))
        if lower:
            previous = stream.read(1)
        else:
            previous = b'\n'
        data = stream.read(size - lower)
        if len(data) != size - lower:
            raise ValueError('log shrank while being read')
        start = lower
        if lower and previous != b'\n':
            cut = data.find(b'\n')
            if cut < 0:
                start, data = size, b''
            else:
                start += cut + 1
                data = data[cut + 1:]
        last = data.rfind(b'\n')
        data = data[:last + 1] if last >= 0 else b''
        end = start + len(data)
        after = os.fstat(stream.fileno())
    return data, {
        'source_size_bytes': size,
        'source_mtime_ns': before.st_mtime_ns,
        'observed_size_after_read_bytes': after.st_size,
        'source_changed_during_read': (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns),
        'source_byte_range': {'start_inclusive': start, 'end_exclusive': end},
        'dropped_prefix_bytes': start,
        'dropped_suffix_bytes': size - end,
        'truncated': start != 0 or end != size,
        'complete_newline_terminated_lines': data.count(b'\n'),
        'range_note': 'Offsets are bytes in the source snapshot. A non-newline-terminated final line is omitted, even if it might be valid JSON.',
    }


class Capture:
    def __init__(self, root, archive):
        self.root, self.archive = root, archive
        self.inventory, self.warnings = [], []
        self.used, self.seen = 0, set()
        self.manifest_data = None

    def relative(self, path):
        return path.relative_to(self.root).as_posix()

    def warning(self, path, status, detail):
        row = {'path': self.relative(path), 'status': status, 'detail': str(detail)}
        self.inventory.append(row)
        self.warnings.append(row)
        say('WARNING: ' + row['path'] + ' — ' + status + ': ' + str(detail))

    def safe(self, path):
        try:
            return within(self.root, path)
        except (OSError, ValueError, RuntimeError) as exc:
            self.warning(path, 'rejected_path', exc)
            return None

    def add(self, path, *, tail=False, required=False):
        name = self.relative(path)
        if name in self.seen:
            return
        self.seen.add(name)
        actual = self.safe(path)
        if actual is None:
            return
        if path.is_symlink():
            self.warning(path, 'skipped_link_file', 'Linked input files are not copied, including links inside the project.')
            return
        if not actual.is_file():
            self.warning(path, 'missing_required' if required else 'missing_optional', 'File is not available; existing evidence will still be packaged.')
            return
        try:
            if tail:
                data, metadata = log_tail(actual)
            else:
                with actual.open('rb') as stream:
                    before = os.fstat(stream.fileno())
                    if before.st_size > FILE_LIMIT:
                        self.warning(path, 'skipped_file_limit', 'Source exceeds the 32 MiB file limit.')
                        return
                    data = stream.read(FILE_LIMIT + 1)
                    after = os.fstat(stream.fileno())
                if len(data) > FILE_LIMIT:
                    self.warning(path, 'skipped_file_limit', 'Source grew beyond the 32 MiB file limit.')
                    return
                if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns) or len(data) != before.st_size:
                    self.warning(path, 'source_changed', 'Source changed while being read; its bytes were not included.')
                    return
                metadata = {'source_size_bytes': before.st_size, 'source_mtime_ns': before.st_mtime_ns,
                            'source_byte_range': {'start_inclusive': 0, 'end_exclusive': len(data)}, 'truncated': False}
            if self.used + len(data) > PAYLOAD_LIMIT:
                self.warning(path, 'skipped_package_limit', 'Including this file would exceed the package payload limit.')
                return
            self.archive.writestr(name, data)
            self.used += len(data)
            self.inventory.append({'path': name, 'status': 'captured', 'bytes': len(data),
                                   'captured_sha256': digest(data), 'capture_mode': 'complete_lines_tail' if tail else 'whole_file', **metadata})
            if tail and not data:
                self.warnings.append({'path': name, 'status': 'no_complete_log_lines', 'detail': 'No complete newline-terminated lines were available in the selected tail.'})
                say('WARNING: ' + name + ' — 日志尾部没有可保留的完整行。')
            if name == 'data/navigation/manifest.json':
                self.manifest_data = data
        except (OSError, ValueError, RuntimeError) as exc:
            self.warning(path, 'read_failed', exc)

    def source_tree(self):
        folder = self.root / 'adrive'
        actual = self.safe(folder)
        if actual is None or not actual.is_dir():
            if actual is not None:
                self.warning(folder, 'missing_required', 'Python application source directory is missing.')
            return
        if folder.is_symlink():
            self.warning(folder, 'skipped_link_directory', 'Linked source directories are not traversed.')
            return
        def failed_walk(exc):
            self.warning(Path(exc.filename or folder), 'source_directory_unreadable', exc)
        for current, directories, files in os.walk(folder, topdown=True, followlinks=False, onerror=failed_walk):
            parent = Path(current)
            retained = []
            for name in sorted(directories):
                path = parent / name
                if name in CACHE_NAMES:
                    continue
                if self.safe(path) is None:
                    continue
                if path.is_symlink():
                    self.warning(path, 'skipped_link_directory', 'Linked source directories are not traversed.')
                    continue
                retained.append(name)
            directories[:] = retained
            for name in sorted(files):
                if name.endswith('.py'):
                    self.add(parent / name)

    def recent(self, relative, suffix, count):
        folder = self.root / relative
        actual = self.safe(folder)
        if actual is None:
            return []
        if folder.is_symlink():
            self.warning(folder, 'skipped_link_directory', 'Linked log/report directories are not traversed.')
            return []
        if not actual.is_dir():
            self.warning(folder, 'missing_optional', 'No directory is available; no existing records can be copied.')
            return []
        candidates = []
        for path in folder.glob('*' + suffix):
            candidate = self.safe(path)
            if candidate is None:
                continue
            try:
                if candidate.is_file():
                    candidates.append((candidate.stat().st_mtime_ns, path.name, path))
            except OSError as exc:
                self.warning(path, 'unreadable_metadata', exc)
        if not candidates:
            self.warning(folder, 'no_existing_records', 'No matching existing records were found.')
        return [row[2] for row in sorted(candidates, reverse=True)[:count]]

    def native_hash(self):
        if self.manifest_data is None:
            return {'status': 'unavailable', 'detail': 'Navigation manifest was not captured; active DLL was not located.'}
        try:
            entry = json.loads(self.manifest_data.decode('utf-8-sig'))['library']
            relative = entry['path']
            if not isinstance(relative, str) or '\\' in relative or ':' in relative:
                raise ValueError('Invalid relative DLL path')
            name = PurePosixPath(relative)
            if name.is_absolute() or not name.parts or any(p in ('.', '..') for p in name.parts) or name.suffix.lower() != '.dll':
                raise ValueError('Unsafe DLL path')
            nav = within(self.root, self.root / 'data/navigation')
            path = within(nav, nav.joinpath(*name.parts))
            with path.open('rb') as stream:
                before = os.fstat(stream.fileno())
                if before.st_size > FILE_LIMIT:
                    raise ValueError('DLL exceeds the 32 MiB read limit')
                hasher, total = hashlib.sha256(), 0
                while True:
                    data = stream.read(min(1024 * 1024, FILE_LIMIT + 1 - total))
                    if not data:
                        break
                    total += len(data)
                    if total > FILE_LIMIT:
                        raise ValueError('DLL grew beyond the read limit')
                    hasher.update(data)
                after = os.fstat(stream.fileno())
            if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns) or total != before.st_size:
                raise ValueError('DLL changed while hashing')
            return {'status': 'hashed_only', 'project_relative_path': path.relative_to(self.root).as_posix(),
                    'bytes': total, 'sha256': hasher.hexdigest(), 'copied': False, 'executed': False,
                    'matches_manifest': total == entry.get('bytes') and hasher.hexdigest() == entry.get('sha256')}
        except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
            self.warnings.append({'path': 'data/navigation/manifest.json:library', 'status': 'dll_hash_unavailable', 'detail': str(exc)})
            say('WARNING: 无法核对活动 DLL 摘要：' + str(exc))
            return {'status': 'unavailable', 'detail': str(exc), 'copied': False, 'executed': False}


def collect(root):
    root = Path(root).resolve()
    if not root.is_dir():
        raise ValueError('Project root is not an existing directory')
    runtime = within(root, root / 'runtime')
    runtime.mkdir(exist_ok=True)
    now = datetime.now(timezone.utc)
    directory = runtime / ('route-quality-' + now.strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8])
    directory.mkdir()
    target = directory / 'route-quality-case.zip'
    temporary = directory / 'route-quality-case.zip.part'
    say('输出目录：' + str(directory))
    try:
        with zipfile.ZipFile(temporary, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            capture = Capture(root, archive)
            say('阶段 1/3：只读采集程序源码、道路数据和导航清单 ...')
            capture.source_tree()
            capture.add(root / 'adrive/navmesh.py', required=True)
            for name in ['data/bundle.json', 'data/navigation/manifest.json', 'native/navmesh/bridge.cpp']:
                capture.add(root / name, required=True)
            capture.add(root / 'README.md')
            say('阶段 2/3：采集现有日志尾部及最近一次修复报告 ...')
            for path in capture.recent('runtime/logs', '.jsonl', 3):
                capture.add(path, tail=True)
            for path in capture.recent('runtime/bridge-fix-reports', '.json', 1):
                capture.add(path)
            say('阶段 3/3：只计算活动 DLL 摘要，写入采集清单 ...')
            native = capture.native_hash()
            case = {
                'schema': 1, 'collector_version': VERSION, 'created_at_utc': now.isoformat(),
                'project_root': str(root), 'capture_status': 'partial' if capture.warnings else 'captured',
                'purpose': 'Capture existing application source, road data and logs for route-quality analysis.',
                'route_reproduction_complete': False,
                'evidence_limits': [
                    'Existing logs may not contain the current preview, its guide, or all route points. This collector does not claim they do.',
                    'No project code is imported or executed. No route is calculated and no DLL is loaded.',
                    'At most three newest jsonl logs are included, each with at most 8 MiB of complete newline-terminated tail lines. Inventory records source byte ranges and truncation.',
                ],
                'not_collected': ['runtime/config.json', 'environment variables', 'account settings', 'navigation tiles', 'CASC/game assets', 'DLL bytes'],
                'limits_bytes': {'source_file': FILE_LIMIT, 'each_log_tail': LOG_LIMIT, 'archive_and_uncompressed_total': PACKAGE_LIMIT},
                'active_library': native, 'source_payload_bytes': capture.used,
                'warnings': capture.warnings,
            }
            inventory = encoded({'schema': 1, 'files': capture.inventory})
            metadata = encoded(case)
            if capture.used + len(inventory) + len(metadata) > PACKAGE_LIMIT:
                raise ValueError('Package metadata would exceed the 96 MiB total limit')
            archive.writestr('inventory.json', inventory)
            archive.writestr('case.json', metadata)
        if temporary.stat().st_size > PACKAGE_LIMIT:
            raise ValueError('ZIP exceeds the 96 MiB total limit')
        temporary.replace(target)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise
    result = {'path': str(target), 'output_directory': str(directory), 'capture_status': case['capture_status'],
              'warning_count': len(capture.warnings), 'source_payload_bytes': capture.used, 'zip_bytes': target.stat().st_size,
              'route_reproduction_complete': False}
    say('ZIP = ' + str(target))
    say('采集结束。请上传这个 ZIP；现有日志是否包含本次预览，将在检查后确认。')
    return result


def main():
    say('ROUTE QUALITY ' + VERSION + '：启动，只读采集现有资料。')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, default=Path.cwd())
    args = parser.parse_args()
    try:
        result = collect(args.project_root)
    except Exception as exc:
        say('ERROR = ' + type(exc).__name__ + ': ' + str(exc))
        return 1
    say(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
