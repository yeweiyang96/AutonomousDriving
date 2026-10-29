"""Read-only Pico health and optional game telemetry diagnostics."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from .device_diagnostics import FirmwareDiagnosticsClient, describe_diagnostic
from .eventlog import json_value
from .pico import PicoClient, PicoError


def main(argv=None):
    parser = argparse.ArgumentParser(description='只读检查 Pico，不发送键盘或鼠标动作')
    parser.add_argument('--host', default='192.168.11.7')
    parser.add_argument('--port', type=int, default=8888)
    parser.add_argument('--count', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=1)
    parser.add_argument('--firmware', action='store_true', help='读取固件版本、能力、停止原因和 USB 诊断')
    parser.add_argument('--output', type=Path, help='将只读诊断记录保存为 JSONL，便于刷机前后比较')
    parser.add_argument('--capture', action='store_true', help='额外尝试读取游戏色块 3 秒')
    parser.add_argument('--lease', action='store_true', help='只读查询 ADR v3 连续控制能力，不开启会话')
    parser.add_argument('--window', default='World of Warcraft')
    args = parser.parse_args(argv)
    if not 1 <= args.count <= 10 or not .05 <= args.timeout <= 3:
        parser.error('count 应为 1–10；timeout 应为 0.05–3 秒')
    successes = 0
    rows = []

    def emit(row):
        line = json.dumps(row, ensure_ascii=False, allow_nan=False)
        rows.append(line)
        print(line)

    if args.firmware:
        diagnostic = FirmwareDiagnosticsClient(args.host, args.port, timeout=args.timeout)
        try:
            for index in range(args.count):
                diagnostic.begin(time.monotonic(), context={'trigger': 'cli', 'index': index})
                while True:
                    result = diagnostic.poll(time.monotonic())
                    if result is not None:
                        break
                    time.sleep(.01)
                successes += result.kind != 'error'
                summary, details = describe_diagnostic(result)
                emit({'kind': 'firmware_diagnostic', 'result': json_value(result),
                      'summary': summary, 'details': details})
        finally:
            diagnostic.close()
    else:
        pico = PicoClient(args.host, args.port, timeout=args.timeout)
        for _ in range(args.count):
            start = time.monotonic()
            try:
                health = pico.query()
                successes += 1
                emit({'kind': 'health', 'rtt_ms': round((time.monotonic() - start) * 1000, 1),
                      'ready': health.ready, 'health': json_value(health)})
            except (OSError, PicoError) as exc:
                emit({'kind': 'health_error', 'message': str(exc)})
        pico.close()
    if args.lease:
        from .lease import LeaseClient
        client = LeaseClient(args.host, args.port)
        try:
            client.query_only(time.monotonic())
            while client.state == 'diagnosing':
                client.poll(time.monotonic())
                time.sleep(.01)
            emit({'kind': 'lease_status', 'supported': client.supported,
                  'status': json_value(client.status)})
        except (OSError, PicoError) as exc:
            emit({'kind': 'lease_error', 'message': str(exc)})
            successes = 0
        finally:
            client.close()
    if args.capture:
        from .capture import WindowsCapture
        capture = WindowsCapture(args.window)
        try:
            deadline = time.monotonic() + 3
            count = 0
            while time.monotonic() < deadline:
                t = capture.poll()
                if t:
                    count += 1
                    if count == 1:
                        emit({'kind': 'telemetry', 'data': json_value(t)})
                time.sleep(.03)
            emit({'kind': 'capture', 'frames': count, 'status': capture.status})
        finally:
            capture.close()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text('\n'.join(rows) + '\n', encoding='utf-8')
    return 0 if successes else 1


if __name__ == '__main__':
    raise SystemExit(main())
