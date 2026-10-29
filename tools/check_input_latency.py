"""Read-only, opt-in B/WASD/mouse correlation probe; never opens an input lease."""
import argparse
import ctypes
from ctypes import wintypes
import json
from pathlib import Path
import secrets
import statistics
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from adrive.device_diagnostics import decode_diagnostic
from adrive.lease import REQUEST, decode_status
from adrive.scheduling import LoopWaiter
import socket


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.11.7')
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', type=Path, default=Path('runtime/input-latency-probe.json'))
    args = parser.parse_args()
    if not 1 <= args.seconds <= 120:
        parser.error('seconds must be 1..120')
    keys = {'B': 0x42, 'W': 0x57, 'A': 0x41, 'S': 0x53, 'D': 0x44,
            'mouse_left': 1, 'mouse_right': 2, 'mouse_middle': 4}
    user = ctypes.WinDLL('user32', use_last_error=True)
    user.GetAsyncKeyState.argtypes = [ctypes.c_int]
    user.GetAsyncKeyState.restype = ctypes.c_short
    user.GetCursorPos.argtypes = [ctypes.POINTER(wintypes.POINT)]
    user.GetCursorPos.restype = wintypes.BOOL
    def packet(op, token):
        return REQUEST.pack(b'ADRV', 3, op, 0, 0, 0, token, 0, 0, 0, 0, 0, b'\0'*6, 0)
    rows = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((args.host, 8888))
        sock.settimeout(1)
        token = secrets.randbits(32) or 1
        sock.send(packet(0, token))
        status = decode_status(sock.recv(512), time.monotonic())
        if status.token != token or status.active or not status.can_open:
            raise RuntimeError('Pico 非空闲，请先手动停止自动行走；探针没有发送停止或输入')
        sock.setblocking(False)
        waiter = LoopWaiter()
        waiter.set_socket(sock)
        old_wait = threading.Event()
        previous = wintypes.POINT()
        user.GetCursorPos(ctypes.byref(previous))
        started = time.monotonic()
        print('只读测量开始：前5秒静置，随后按B/WASD和操作鼠标。', flush=True)
        try:
            while time.monotonic()-started < args.seconds:
                active = [name for name, vk in keys.items() if user.GetAsyncKeyState(vk) & 0x8000]
                point = wintypes.POINT()
                moved = bool(user.GetCursorPos(ctypes.byref(point)) and
                             (point.x, point.y) != (previous.x, previous.y))
                previous = point
                token = secrets.randbits(32) or 1
                mode = 'event_wait_old' if len(rows) % 2 == 0 else 'socket_wake_new'
                began = time.monotonic()
                row = {'t': began-started, 'mode': mode, 'held': active, 'mouse_moved': moved}
                sock.send(packet(5, token))
                while True:
                    if mode == 'event_wait_old':
                        old_wait.wait(1/60)
                    else:
                        waiter.wait(1/60)
                    now = time.monotonic()
                    try:
                        data = sock.recv(512)
                    except BlockingIOError:
                        if now-began < .180:
                            continue
                        row['timeout'] = True
                        break
                    # Ignore a response left over from a prior timed-out query.
                    if len(data) < 12 or int.from_bytes(data[8:12], 'little') != token:
                        if now-began < .180:
                            continue
                        row['timeout'] = True
                        break
                    d = decode_diagnostic(data, token, now)
                    row.update(rtt_ms=(now-began)*1000, over_180ms=now-began > .180,
                               boot=d.boot, device_uptime_ms=d.uptime_ms,
                               net_errors=d.net_send_errors, usb_errors=d.usb_errors,
                               core0_gap_us=d.core0_max_gap_us, core1_gap_us=d.core1_max_gap_us)
                    break
                rows.append(row)
                time.sleep(.05)
        finally:
            waiter.close()
    summary = {}
    for mode in ('event_wait_old', 'socket_wake_new'):
        for activity in (False, True):
            selected = [r for r in rows if r['mode'] == mode and bool(r['held'] or r['mouse_moved']) == activity]
            v = sorted(r['rtt_ms'] for r in selected if 'rtt_ms' in r)
            summary[f'{mode}:activity={activity}'] = {
                'count': len(selected), 'timeouts': sum(r.get('timeout', False) for r in selected),
                'over_180ms': sum(r.get('over_180ms', False) for r in selected),
                'median_ms': statistics.median(v) if v else None,
                'p95_ms': v[min(len(v)-1, int(len(v)*.95))] if v else None,
                'max_ms': max(v) if v else None}
    result = {'summary': summary, 'samples': rows,
              'limits': 'Only specified combined key/button states and cursor-change booleans sampled; short taps can be missed. No physical device attribution or game outcome.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
