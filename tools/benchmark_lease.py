r"""Compare firmware host DLLs without sockets, USB devices, or game input.

Example (PowerShell, from the repository root)::

    .\.venv\Scripts\python.exe tools\benchmark_lease.py `
      --baseline runtime/tmp/lease-diagnostics-benchmark/hid_lease_host_baseline.dll `
      --candidate ../hid_usb/build-host-tests/hid_lease_host.dll `
      --output runtime/tmp/lease-diagnostics-benchmark/comparison.json

The timed region is the same legacy host API in both libraries: exchange()
(which includes a pre-request tick) and one explicit post-request tick().
Request construction, session setup, validation, and counter reads are outside
that region. Results include Python/ctypes overhead and host scheduling; they
are not RP2350/Pico 2 W, Wi-Fi, physical USB completion, or end-to-end latency measurements.
"""
from __future__ import annotations

import argparse
import ctypes
import gc
import hashlib
import json
import math
from pathlib import Path
import platform
import statistics
import struct
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BASELINE = ROOT / 'runtime/tmp/lease-diagnostics-benchmark/hid_lease_host_baseline.dll'
REQUEST = struct.Struct('<4sBBHHHIIIIIB6sB')
RESPONSE = struct.Struct('<4sBBBB7IHHB6sB')
QUERY, OPEN, SET, STOP, RENEW, DIAGNOSTIC = range(6)
DIAGNOSTIC_SIZE = 160
KEY_W, KEY_A = 26, 4
U8 = ctypes.c_uint8
U32 = ctypes.c_uint32


def percentile(values, fraction):
    """Nearest-rank percentile; keep its definition fixed across runs."""
    ordered = sorted(values)
    return ordered[max(0, math.ceil(len(ordered) * fraction) - 1)]


def distribution(samples_ns):
    return {
        'count': len(samples_ns),
        'p50_us': round(statistics.median(samples_ns) / 1000, 3),
        'p95_us': round(percentile(samples_ns, .95) / 1000, 3),
        'max_us': round(max(samples_ns) / 1000, 3),
    }


class HostLibrary:
    """Only the USB-free host adapter API is called; never create a socket."""

    def __init__(self, path):
        self.path = Path(path).resolve(strict=True)
        self.dll = ctypes.CDLL(str(self.path))
        self.dll.reset.argtypes = []
        self.dll.reset.restype = None
        self.dll.exchange.argtypes = [ctypes.POINTER(U8), U32, ctypes.POINTER(U8)]
        self.dll.exchange.restype = None
        self.dll.tick.argtypes = [U32]
        self.dll.tick.restype = None
        self.dll.copy_keyboard.argtypes = [ctypes.POINTER(U8)]
        self.dll.copy_keyboard.restype = None
        for name in ('keyboard_reports', 'keyboard_releases'):
            function = getattr(self.dll, name)
            function.argtypes = []
            function.restype = ctypes.c_uint
        self.response = (U8 * RESPONSE.size)()
        self.diagnostic_response = (U8 * DIAGNOSTIC_SIZE)()
        self.exchange_diagnostic = getattr(self.dll, 'exchange_diagnostic', None)
        if self.exchange_diagnostic is not None:
            self.exchange_diagnostic.argtypes = [ctypes.POINTER(U8), U32, ctypes.POINTER(U8)]
            self.exchange_diagnostic.restype = None
        self.complete_keyboard = getattr(self.dll, 'complete_keyboard', None)
        if self.complete_keyboard is not None:
            self.complete_keyboard.argtypes = [U32, ctypes.c_int]
            self.complete_keyboard.restype = None
        self.network_heartbeat = getattr(self.dll, 'network_heartbeat_us', None)
        if self.network_heartbeat is not None:
            self.network_heartbeat.argtypes = [ctypes.c_uint64]
            self.network_heartbeat.restype = None
        self.keyboard = (U8 * 7)()
        self.boot = self.epoch = 0
        self.session = 0x13572468

    def request(self, op, seq=0, keys=(), token=None):
        query = op in (QUERY, DIAGNOSTIC)
        packet = REQUEST.pack(
            b'ADRV', 3, op, 250 if op in (OPEN, SET, RENEW) else 0, 0, 0,
            (seq + op + 1) & 0xffffffff if token is None else token,
            0 if query else self.boot, 0 if query else self.epoch,
            0 if query else self.session, seq, 0,
            bytes(keys).ljust(6, b'\0'), 0,
        )
        return (U8 * REQUEST.size).from_buffer_copy(packet)

    def status(self):
        fields = RESPONSE.unpack(self.response)
        if fields[0] != b'ADRS' or fields[1] != 3 or fields[2] != 0:
            raise RuntimeError(f'Host adapter rejected benchmark request: {fields[:4]!r}')
        return fields

    def require_diagnostic(self):
        header = struct.unpack_from('<4sBBH', self.diagnostic_response)
        if header != (b'ADRD', 1, 0, DIAGNOSTIC_SIZE):
            raise RuntimeError(f'Unexpected diagnostic response header: {header!r}')

    def exchange(self, packet, now_ms):
        self.dll.exchange(packet, now_ms, self.response)
        return self.status()

    def prepare(self, held=False):
        # Reset/setup are intentionally outside every timed interval.
        self.dll.reset()
        status = self.exchange(self.request(QUERY), 10000)
        self.boot, self.epoch = status[6], status[7]
        status = self.exchange(self.request(OPEN), 10001)
        self.epoch = status[7]
        self.dll.tick(10002)
        if self.complete_keyboard is not None:
            self.complete_keyboard(10002, 1)
        status = self.exchange(self.request(QUERY), 10003)
        if not status[3] & 4 or status[3] & 8:
            raise RuntimeError('Host adapter did not open an active, ready session')
        if held:
            self.exchange(self.request(SET, 1, (KEY_W,)), 10004)
            self.dll.tick(10005)
            if self.complete_keyboard is not None:
                self.complete_keyboard(10005, 1)
            self.require_keys((KEY_W,))

    def counts(self):
        return self.dll.keyboard_reports(), self.dll.keyboard_releases()

    def require_keys(self, expected):
        self.dll.copy_keyboard(self.keyboard)
        actual = tuple(value for value in self.keyboard[1:] if value)
        if self.keyboard[0] or actual != expected:
            raise RuntimeError(f'Unexpected host keyboard state: {actual}, expected {expected}')

    def metadata(self):
        return {
            'path': str(self.path),
            'sha256': hashlib.sha256(self.path.read_bytes()).hexdigest(),
            'bytes': self.path.stat().st_size,
        }


def measure_workload(native, operation, samples, warmup):
    """Exercise valid state changes, rather than benchmarking rejected packets."""
    if operation == 'network_heartbeat':
        native.dll.reset()
        elapsed = []
        before = native.counts()
        for index in range(samples + warmup):
            now_us = 10000000 + index * 1000
            began = time.perf_counter_ns()
            native.network_heartbeat(now_us)
            duration = time.perf_counter_ns() - began
            if index >= warmup:
                elapsed.append(duration)
        if native.counts() != before:
            raise RuntimeError('Network heartbeat accounting generated a USB report')
        native.require_keys(())
        return elapsed, 0, 0, [], []
    if operation != 'stop':
        native.prepare(held=operation in ('renew', 'diagnostic'))
        if operation == 'diagnostic':
            # Observe a closed session; deliberately expiring an active lease
            # would add a correct release report unrelated to query overhead.
            native.exchange(native.request(STOP, 1), 10006)
            native.dll.tick(10007)
            if native.complete_keyboard is not None:
                native.complete_keyboard(10007, 1)
            native.require_keys(())
    elapsed, callback_elapsed, combined_elapsed = [], [], []
    report_total = release_total = 0
    for index in range(samples + warmup):
        if operation == 'stop':
            native.prepare(held=True)
            packet = native.request(STOP, 1)
            now_ms, expected = 10010, ()
        elif operation == 'set':
            expected = (KEY_W if index % 2 == 0 else KEY_A,)
            packet = native.request(SET, index + 1, expected)
            now_ms = 10010 + index * 2
        elif operation == 'renew':
            expected = (KEY_W,)
            packet = native.request(RENEW, index + 2)
            now_ms = 10010 + index * 2
        else:
            expected = ()
            packet = native.request(DIAGNOSTIC, token=index + 1)
            now_ms = 10010 + index * 2
        exchange = native.exchange_diagnostic if operation == 'diagnostic' else native.dll.exchange
        response = native.diagnostic_response if operation == 'diagnostic' else native.response
        reports_before, releases_before = native.counts()
        began = time.perf_counter_ns()
        exchange(packet, now_ms, response)
        native.dll.tick(now_ms + 1)
        submitted = time.perf_counter_ns()
        duration = submitted - began
        callback_duration = combined_duration = None
        if native.complete_keyboard is not None and operation in ('set', 'stop'):
            # Host completion is explicit and synchronous only in this fake.
            # Keep the old shared API timing comparable; additionally record
            # callback bookkeeping and the combined interval for the new DLL.
            native.complete_keyboard(now_ms + 1, 1)
            completed = time.perf_counter_ns()
            callback_duration, combined_duration = completed - submitted, completed - began
        if operation == 'diagnostic':
            native.require_diagnostic()
        else:
            native.status()
        native.require_keys(expected)
        reports_after, releases_after = native.counts()
        reports, releases = reports_after - reports_before, releases_after - releases_before
        required = (0, 0) if operation in ('renew', 'diagnostic') else ((1, 1) if operation == 'stop' else (1, 0))
        if (reports, releases) != required:
            raise RuntimeError(f'{operation}: {reports} USB reports/{releases} releases; expected {required}')
        if index >= warmup:
            elapsed.append(duration)
            if callback_duration is not None:
                callback_elapsed.append(callback_duration)
                combined_elapsed.append(combined_duration)
            report_total += reports
            release_total += releases
    return elapsed, report_total, release_total, callback_elapsed, combined_elapsed


def benchmark(libraries, *, samples, rounds, warmup):
    raw = {label: {op: [] for op in ('set', 'renew', 'stop')} for label in libraries}
    for label, native in libraries.items():
        if native.exchange_diagnostic is not None:
            raw[label]['diagnostic'] = []
        if native.network_heartbeat is not None:
            raw[label]['network_heartbeat'] = []
    totals = {label: {op: [0, 0] for op in raw[label]} for label in libraries}
    round_results = {label: {op: [] for op in raw[label]} for label in libraries}
    completion = {label: {op: [] for op in raw[label]} for label in libraries}
    combined = {label: {op: [] for op in raw[label]} for label in libraries}
    labels = list(libraries)
    gc_enabled = gc.isenabled()
    gc.disable()
    try:
        for round_index in range(rounds):
            # Alternate library order to reduce systematic warmup/drift bias.
            for label in labels if round_index % 2 == 0 else reversed(labels):
                for operation in raw[label]:
                    values, reports, releases, callbacks, complete_path = measure_workload(
                        libraries[label], operation, samples, warmup)
                    raw[label][operation].extend(values)
                    totals[label][operation][0] += reports
                    totals[label][operation][1] += releases
                    round_results[label][operation].append(distribution(values))
                    completion[label][operation].extend(callbacks)
                    combined[label][operation].extend(complete_path)
    finally:
        if gc_enabled:
            gc.enable()
    output = {}
    for label, native in libraries.items():
        workloads = {}
        for operation, values in raw[label].items():
            reports, releases = totals[label][operation]
            workloads[operation] = {
                **distribution(values),
                'usb_reports': reports,
                'usb_release_reports': releases,
                'unexpected_duplicate_reports': 0,
                'rounds': round_results[label][operation],
            }
            if operation == 'network_heartbeat':
                workloads[operation]['timed_region'] = 'one network_heartbeat_us host call; no exchange or tick'
                workloads[operation]['comparison_limit'] = 'No old-library export; absolute host cost only'
            if completion[label][operation]:
                workloads[operation]['completion_callback'] = distribution(completion[label][operation])
                workloads[operation]['including_completion_callback'] = distribution(combined[label][operation])
        output[label] = {**native.metadata(), 'workloads': workloads}
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--baseline', type=Path, default=DEFAULT_BASELINE)
    parser.add_argument('--candidate', type=Path)
    parser.add_argument('--samples', type=int, default=2000, help='Measured operations per workload per round')
    parser.add_argument('--rounds', type=int, default=5)
    parser.add_argument('--warmup', type=int, default=100, help='Unrecorded operations per workload per round')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 1 <= args.samples <= 100000 or not 1 <= args.rounds <= 30 or not 0 <= args.warmup <= 10000:
        parser.error('samples must be 1..100000, rounds 1..30, warmup 0..10000')
    libraries = {'baseline': HostLibrary(args.baseline)}
    if args.candidate:
        libraries['candidate'] = HostLibrary(args.candidate)
    results = benchmark(libraries, samples=args.samples, rounds=args.rounds, warmup=args.warmup)
    report = {
        'schema': 1,
        'created_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'environment': {'platform': platform.platform(), 'python': sys.version,
                        'timer': time.get_clock_info('perf_counter').implementation},
        'method': {
            'timed_region': 'host exchange (includes pre-request tick) + one post-request tick',
            'samples_per_round': args.samples, 'rounds': args.rounds, 'warmup_per_round': args.warmup,
            'percentile': 'p50 median, p95 nearest rank',
            'gc_during_measurement': 'disabled',
            'completion_callback': 'If the DLL exports complete_keyboard, submit and STOP workloads '
                                   'also measure that fake success callback separately and combined. '
                                   'Includes an extra ctypes call; this is not physical USB completion latency.',
            'limits': 'Host CPU + ctypes microbenchmark only; no sockets, physical USB, MCU cores, or game. '
                      'Does not establish or guarantee on-device or end-to-end latency.',
        },
        'control_wire_bytes': {'request': REQUEST.size, 'response': RESPONSE.size,
                               'responses_per_request': 1,
                               'scope': 'legacy exchange API; actual network traffic is not measured'},
        'optional_diagnostic_wire_bytes': {'request': REQUEST.size, 'response': DIAGNOSTIC_SIZE,
                                           'scope': 'only measured for DLLs exposing exchange_diagnostic'},
        'libraries': results,
    }
    if 'candidate' in results:
        comparisons = {}
        for operation in ('set', 'renew', 'stop'):
            before = results['baseline']['workloads'][operation]
            after = results['candidate']['workloads'][operation]
            comparisons[operation] = {
                f'{metric}_ratio': round(after[metric] / before[metric], 4)
                for metric in ('p50_us', 'p95_us', 'max_us')
            }
            comparisons[operation]['p50_delta_us'] = round(after['p50_us'] - before['p50_us'], 3)
        report['candidate_over_baseline'] = comparisons
    encoded = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + '\n', encoding='utf-8')
    print(encoded)


if __name__ == '__main__':
    main()
