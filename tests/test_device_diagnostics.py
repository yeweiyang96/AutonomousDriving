"""Read-only diagnostics cannot drive input or refresh control acknowledgements."""
from dataclasses import asdict, replace
import io
import json
from pathlib import Path
import queue
import struct
import tempfile
import unittest
from unittest.mock import Mock, patch

from adrive.app import Application
from adrive.controller import Calibration
from adrive.diagnostics import main as diagnostics_main
from adrive.device_diagnostics import (DIAGNOSTIC_OP, DIAGNOSTIC_SIZE, HEADER, DiagnosticResult,
                                       FirmwareDiagnosticsClient, decode_diagnostic, describe_diagnostic)
from adrive.lease import LeaseClient, LeaseStatus, REQUEST, RESPONSE
from adrive.models import Point
from adrive.pico import HEALTH_REQUEST, PicoError
from adrive.runtime import Runtime
from adrive.settings import ROOT, Settings
from tests.test_continuous import FakeCapture, FakeLease, FakePico


def diagnostic_packet(token=42, *, validity=31, capabilities=63,
                      lease=(100, 300), hold=(20, 200), protocol=(3, 3), boot=1234):
    data = bytearray(DIAGNOSTIC_SIZE)
    HEADER.pack_into(data, 0, b'ADRD', 1, 0, DIAGNOSTIC_SIZE, token, boot, 2100,
                     0x00050100, capabilities, (lease[1] << 16) | lease[0],
                     (hold[1] << 16) | hold[0], validity)
    struct.pack_into('<5I', data, 40, 2, 2000, 7, 100, 9)
    for offset, stamp in ((60, 1998), (76, 1999), (92, 2001), (108, 2003)):
        struct.pack_into('<4I', data, offset, 7, 100, 5, stamp)
    struct.pack_into('<6I', data, 124, 2, 3, 1000, 2000, 4, 5)
    struct.pack_into('<I', data, 148, (protocol[1] << 16) | protocol[0])
    return bytes(data)


class DiagnosticWireTests(unittest.TestCase):
    def test_golden_fields_versions_ranges_counters_and_correlated_times(self):
        d = decode_diagnostic(diagnostic_packet(), 42, 11.)
        self.assertEqual(d.version, '0.5.1')
        self.assertEqual((d.lease_min_ms, d.lease_max_ms, d.hold_min_ms, d.hold_max_ms), (100, 300, 20, 200))
        self.assertEqual(d.capability_names[0], '连续按键')
        self.assertEqual(d.compatibility_issues(), ())
        self.assertEqual((d.net_send_errors, d.usb_errors, d.core0_max_gap_us, d.core1_max_gap_us), (2, 3, 1000, 2000))
        self.assertEqual(d.submit_to_complete_ms(), 1)
        self.assertEqual(d.stop_to_release_complete_ms(), 3)  # STOP seq != SET seq is valid.

    def test_missing_or_overwritten_event_is_unknown_not_zero(self):
        d = decode_diagnostic(diagnostic_packet(validity=0), 42, 11.)
        self.assertIsNone(d.submit_to_complete_ms())
        self.assertIsNone(d.stop_to_release_complete_ms())
        summary, details = describe_diagnostic(DiagnosticResult('extended', '', diagnostic=d))
        self.assertIn('无可关联记录', details)
        self.assertIn('尚无有效停止记录', details)
        d = decode_diagnostic(diagnostic_packet(), 42, 11.)
        self.assertIsNone(replace(d, completed=replace(d.completed, seq=6)).submit_to_complete_ms())
        self.assertIsNone(replace(d, release_completed=replace(d.release_completed, session=101)).stop_to_release_complete_ms())

    def test_device_timestamp_wrap_is_directed_and_not_cross_session(self):
        d = decode_diagnostic(diagnostic_packet(), 42, 11.)
        d = replace(d, submitted=replace(d.submitted, time_ms=0xFFFFFFF0),
                    completed=replace(d.completed, time_ms=20),
                    stop=replace(d.stop, time_ms=0xFFFFFFF8),
                    release_completed=replace(d.release_completed, time_ms=1))
        self.assertEqual(d.submit_to_complete_ms(), 36)
        self.assertEqual(d.stop_to_release_complete_ms(), 9)
        self.assertIsNone(replace(d, completed=replace(d.completed, time_ms=0xFFFFFFEF)).submit_to_complete_ms())

    def test_incompatible_capability_or_range_has_explicit_chinese_explanation(self):
        d = decode_diagnostic(diagnostic_packet(capabilities=56, lease=(100, 200), protocol=(4, 4)), 42, 11.)
        self.assertGreaterEqual(len(d.compatibility_issues()), 5)
        summary, details = describe_diagnostic(DiagnosticResult('extended', '', diagnostic=d))
        self.assertIn('不兼容', summary)
        self.assertIn('250 ms', summary)
        self.assertIn('USB 完成不代表游戏已处理', details)

    def test_bad_length_token_reserved_version_and_bounds_are_rejected(self):
        base = diagnostic_packet()
        cases = [base[:-1], base[:8] + struct.pack('<I', 41) + base[12:],
                 base[:4] + b'\x02' + base[5:], base[:-1] + b'\x01',
                 diagnostic_packet(lease=(0, 300))]
        for data in cases:
            with self.subTest(data=data[:12]), self.assertRaises(PicoError):
                decode_diagnostic(data, 42, 11.)


class FakeSocket:
    def __init__(self):
        self.sent = []
        self.inbox = []
        self.closed = False

    def connect(self, address):
        self.address = address

    def setblocking(self, value):
        assert value is False

    def send(self, data):
        self.sent.append(data)
        return len(data)

    def recv(self, size):
        if not self.inbox:
            raise BlockingIOError()
        return self.inbox.pop(0)

    def close(self):
        self.closed = True


class DiagnosticTransportTests(unittest.TestCase):
    def setUp(self):
        self.socket = FakeSocket()
        patcher = patch('adrive.device_diagnostics.socket.socket', return_value=self.socket)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.client = FirmwareDiagnosticsClient()
        self.addCleanup(self.client.close)

    def begin(self):
        self.client.begin(10., context={'trigger': 'manual'})
        return REQUEST.unpack(self.socket.sent[-1])[6]

    def test_exact_read_only_request_and_new_reply_never_open_or_stop(self):
        token = self.begin()
        fields = REQUEST.unpack(self.socket.sent[-1])
        self.assertEqual(fields[:3], (b'ADRV', 3, DIAGNOSTIC_OP))
        self.assertTrue(all(value == 0 for value in fields[3:6] + fields[7:12]))
        self.assertEqual(fields[12:], (b'\0' * 6, 0))
        self.socket.inbox.append(diagnostic_packet(token))
        result = self.client.poll(10.02)
        self.assertEqual(result.kind, 'extended')
        self.assertEqual(result.context, {'trigger': 'manual'})
        self.assertAlmostEqual(result.rtt_ms, 20.)
        self.assertIsNone(self.client.poll(20.))
        self.assertEqual(len(self.socket.sent), 1)

    def test_old_adr_v3_invalid_command_response_does_not_need_fallback(self):
        token = self.begin()
        self.socket.inbox.append(RESPONSE.pack(b'ADRS', 3, 1, 35, 0, token, 1234, 7, 0, 0, 0, 1000,
                                              0, 0, 0, b'\0' * 6, 0))
        result = self.client.poll(10.02)
        self.assertEqual(result.kind, 'legacy_v3')
        self.assertIn('扩展诊断未提供', describe_diagnostic(result)[0])
        self.client.poll(12.)
        self.assertEqual(len(self.socket.sent), 1)

    def test_no_extended_reply_uses_one_legacy_read_only_health_query(self):
        self.begin()
        self.assertIsNone(self.client.poll(10.36))
        command, token = HEALTH_REQUEST.unpack(self.socket.sent[-1])
        self.assertEqual(command, 0)
        self.socket.inbox.append(struct.pack('<4sBBBBIIII', b'PICO', 2, 23, 0, 8, token, 500, 10, 0))
        result = self.client.poll(10.37)
        self.assertEqual(result.kind, 'legacy')
        self.assertTrue(result.health.ready)
        self.assertEqual(len(self.socket.sent), 2)

    def test_timeout_is_a_result_and_does_not_retry_input(self):
        self.begin()
        self.client.poll(10.36)
        result = self.client.poll(10.72)
        self.assertEqual(result.kind, 'error')
        self.assertFalse(self.client.active)
        self.client.poll(20.)
        self.assertEqual(len(self.socket.sent), 2)

    def test_foreign_or_cancelled_reply_never_becomes_diagnostic(self):
        token = self.begin()
        self.socket.inbox.append(diagnostic_packet((token + 1) & 0xFFFFFFFF))
        self.assertIsNone(self.client.poll(10.02))
        self.client.cancel()
        self.socket.inbox.append(diagnostic_packet(token))
        self.assertIsNone(self.client.poll(10.03))
        self.assertEqual(len(self.socket.sent), 1)

    def test_diagnostic_reply_cannot_refresh_real_control_ack_deadline(self):
        control_socket = FakeSocket()
        with patch('adrive.lease.socket.socket', return_value=control_socket):
            control = LeaseClient()
        self.addCleanup(control.close)
        control.state = 'active'
        control.last_ack_at = control.last_sent_at = 10.
        token = self.begin()
        self.socket.inbox.append(diagnostic_packet(token))
        self.assertEqual(self.client.poll(10.15).kind, 'extended')
        self.assertEqual(control.last_ack_at, 10.)
        with self.assertRaises(PicoError):
            control.poll(10.181)
        self.assertEqual(control.state, 'fault')


class FakeDiagnostic:
    def __init__(self, **kw):
        self.active = False
        self.begins = []
        self.result = None
        self.context = None

    def begin(self, now, *, context):
        self.begins.append((now, context))
        self.context = context
        self.active = True

    def poll(self, now):
        if self.result is None:
            return None
        result, self.result = self.result, None
        self.active = False
        return replace(result, context=self.context)

    def cancel(self):
        self.active = False
        self.result = None

    def close(self):
        self.cancel()


class DiagnosticRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        patcher = patch('adrive.runtime.time.monotonic', side_effect=lambda: self.now)
        patcher.start()
        self.addCleanup(patcher.stop)
        root = ROOT / 'runtime' / 'tmp'
        root.mkdir(parents=True, exist_ok=True)
        temp = tempfile.TemporaryDirectory(dir=root)
        self.addCleanup(temp.cleanup)
        self.runtime = Runtime(None, Settings(calibration=asdict(Calibration(3, 3))), queue.Queue(),
                               capture_factory=FakeCapture, pico_factory=lambda **kw: FakePico(lambda: self.now),
                               lease_factory=FakeLease, diagnostic_factory=FakeDiagnostic,
                               log_folder=Path(temp.name))
        self.runtime._connect_objects()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.runtime.close()
        self.runtime.pool.shutdown(wait=True, cancel_futures=True)
        self.runtime.diagnostic.close()
        self.runtime.log.close()

    def test_running_control_never_sends_extra_diagnostic_packets(self):
        self.runtime.state = 'running'
        self.runtime.command('diagnose')
        self.runtime._commands()
        for _ in range(100):
            self.now += .02
            self.runtime._diagnostic_step(self.now)
        self.assertEqual(self.runtime.diagnostic.begins, [])

    def test_fault_schedules_one_delayed_snapshot_without_changing_reason(self):
        self.runtime.state = 'running'
        self.runtime._halt('original ACK failure', 'fault')
        self.now = 10.34
        self.runtime._diagnostic_step(self.now)
        self.assertEqual(self.runtime.diagnostic.begins, [])
        self.now = 10.36
        self.runtime._diagnostic_step(self.now)
        self.assertEqual(len(self.runtime.diagnostic.begins), 1)
        self.runtime.diagnostic.result = DiagnosticResult('error', 'diagnostic timeout')
        self.runtime._diagnostic_step(11.)
        self.assertEqual((self.runtime.state, self.runtime.reason), ('fault', 'original ACK failure'))
        self.runtime._diagnostic_step(20.)
        self.assertEqual(len(self.runtime.diagnostic.begins), 1)
        rows = [json.loads(line) for line in self.runtime.log.path.read_text(encoding='utf-8').splitlines()]
        self.assertTrue(any(r['kind'] == 'diagnostic_error' for r in rows))

    def test_automatic_snapshots_are_rate_limited_after_repeated_stops(self):
        self.runtime.state = 'running'
        self.runtime._halt('first', 'paused')
        self.now = 10.36
        self.runtime.diagnostic.result = DiagnosticResult('error', 'fixture')
        self.runtime._diagnostic_step(self.now)
        self.runtime.state = 'running'
        self.now = 11.
        self.runtime._halt('second', 'paused')
        self.runtime._diagnostic_step(15.35)
        self.assertEqual(len(self.runtime.diagnostic.begins), 1)
        self.runtime._diagnostic_step(15.37)
        self.assertEqual(len(self.runtime.diagnostic.begins), 2)

    def test_new_arm_cancels_diagnostic_without_closing_control_client(self):
        self.runtime.diagnostic.begin(10., context={'trigger': 'manual'})
        self.runtime.command('arm', goal=Point(0, 100, 0))
        self.runtime._commands()
        self.assertFalse(self.runtime.diagnostic.active)
        self.assertEqual(self.runtime.state, 'armed')
        self.assertEqual(self.runtime.lease.state, 'idle')
        self.assertEqual(self.runtime.lease.actions, [])

    def test_explicit_idle_diagnostic_does_not_open_control_session(self):
        self.runtime.command('diagnose')
        self.runtime._commands()
        self.assertTrue(self.runtime.diagnostic.active)
        self.assertEqual(self.runtime.lease.actions, [])
        self.assertEqual(self.runtime.state, 'monitoring')

    def test_known_incompatibility_is_checked_only_for_same_device_boot(self):
        d = decode_diagnostic(diagnostic_packet(capabilities=0), 42, 10.)
        self.runtime.firmware_info = d
        self.runtime.lease.state = 'active'
        self.runtime.lease.poll(10.)
        self.runtime.lease.status = replace(self.runtime.lease.status, boot=d.boot)
        with patch.object(self.runtime.lease, 'poll', return_value=self.runtime.lease.status), self.assertRaisesRegex(PicoError, '不兼容'):
            self.runtime._poll_lease(10.)
        self.runtime.lease.status = replace(self.runtime.lease.status, boot=d.boot + 1)
        with patch.object(self.runtime.lease, 'poll', return_value=self.runtime.lease.status):
            self.runtime._poll_lease(10.)


class DiagnosticUiTests(unittest.TestCase):
    def test_device_summary_and_details_are_displayed_without_changing_control_status(self):
        app = Application.__new__(Application)
        app.closed, app.events = False, queue.Queue()
        app.device_diagnostic_var, app.root, app.map = Mock(), Mock(), Mock()
        app.log = Mock()
        app.state = 'fault'
        app.events.put({'kind': 'device_diagnostic', 'summary': '固件 0.5.1', 'details': 'USB 完成记录'})
        app.poll()
        app.device_diagnostic_var.set.assert_called_once_with('固件 0.5.1')
        self.assertEqual(app.state, 'fault')
        self.assertIn('USB 完成记录', app.log.call_args.args[0])

    def test_reconnect_clears_previous_device_version(self):
        app = Application.__new__(Application)
        app.closed, app.events = False, queue.Queue()
        app.device_diagnostic_var, app.root, app.map, app.log = Mock(), Mock(), Mock(), Mock()
        app.events.put({'kind': 'device_diagnostic_reset'})
        app.poll()
        app.device_diagnostic_var.set.assert_called_once_with('尚未读取固件版本与能力')


class DiagnosticCliTests(unittest.TestCase):
    def test_read_only_cli_saves_same_structured_records_as_stdout(self):
        result = DiagnosticResult('extended', 'fixture', diagnostic=decode_diagnostic(diagnostic_packet(), 42, 11.))
        client = Mock()
        client.poll.return_value = result
        (ROOT / 'runtime' / 'tmp').mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT / 'runtime' / 'tmp') as folder:
            target = Path(folder) / 'firmware.jsonl'
            stdout = io.StringIO()
            with patch('adrive.diagnostics.FirmwareDiagnosticsClient', return_value=client), \
                    patch('adrive.diagnostics.PicoClient', side_effect=AssertionError('unexpected legacy control client')), \
                    patch('sys.stdout', stdout):
                code = diagnostics_main(['--firmware', '--count', '1', '--output', str(target)])
            self.assertEqual(code, 0)
            self.assertEqual(target.read_text(encoding='utf-8'), stdout.getvalue())
            row = json.loads(stdout.getvalue())
            self.assertEqual(row['result']['diagnostic']['net_send_errors'], 2)
            self.assertIn('0.5.1', row['summary'])
            client.begin.assert_called_once()
            client.close.assert_called_once()

    def test_cli_failure_is_recorded_without_any_input_fallback(self):
        client = Mock()
        client.poll.return_value = DiagnosticResult('error', 'fixture timeout')
        with patch('adrive.diagnostics.FirmwareDiagnosticsClient', return_value=client), \
                patch('sys.stdout', new_callable=io.StringIO) as stdout:
            code = diagnostics_main(['--firmware', '--count', '1'])
        self.assertEqual(code, 1)
        self.assertEqual(json.loads(stdout.getvalue())['result']['kind'], 'error')


if __name__ == '__main__':
    unittest.main()
