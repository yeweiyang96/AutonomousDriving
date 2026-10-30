"""Actual firmware diagnostics and callbacks; host DLL has no USB or network."""
import ctypes
import os
import unittest
from unittest.mock import patch

from adrive.device_diagnostics import FirmwareDiagnosticsClient
from adrive.lease import REQUEST, OPEN, QUERY, SET, STOP, decode_status
from tests.test_native_lease import DLL


class DiagnosticSocket:
    def __init__(self, native):
        self.native, self.now = native, 1000
        self.inbox, self.sent = [], []

    def connect(self, address):
        pass

    def setblocking(self, blocking):
        assert not blocking

    def send(self, data):
        self.sent.append(data)
        request = (ctypes.c_uint8 * 40).from_buffer_copy(data)
        response = (ctypes.c_uint8 * 160)()
        self.native.exchange_diagnostic(request, self.now, response)
        self.inbox.append(bytes(response))
        return len(data)

    def recv(self, size):
        if not self.inbox:
            raise BlockingIOError()
        return self.inbox.pop(0)

    def close(self):
        pass


@unittest.skipUnless(os.name == 'nt' and DLL.is_file(), '可选的生产固件 C 主机 DLL 尚未构建')
class NativeDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.native = ctypes.CDLL(str(DLL))
        u8p = ctypes.POINTER(ctypes.c_uint8)
        for name in ('exchange', 'exchange_diagnostic'):
            method = getattr(self.native, name)
            method.argtypes, method.restype = [u8p, ctypes.c_uint32, u8p], None
        self.native.reset.argtypes, self.native.reset.restype = [], None
        self.native.tick.argtypes, self.native.tick.restype = [ctypes.c_uint32], None
        self.native.complete_keyboard.argtypes = [ctypes.c_uint32, ctypes.c_int]
        self.native.complete_keyboard.restype = None
        self.native.complete_report.argtypes = [ctypes.c_uint32, ctypes.c_uint8, u8p, ctypes.c_int]
        self.native.complete_report.restype = None
        self.native.note_send_error.argtypes, self.native.note_send_error.restype = [], None
        self.native.network_heartbeat_us.argtypes = [ctypes.c_uint64]
        self.native.network_heartbeat_us.restype = None
        self.native.reset()
        self.socket = DiagnosticSocket(self.native)
        with patch('adrive.device_diagnostics.socket.socket', return_value=self.socket):
            self.client = FirmwareDiagnosticsClient()
        self.addCleanup(self.client.close)
        self.status = None

    def exchange(self, op, now, *, seq=0, key=0):
        status = self.status
        packet = REQUEST.pack(b'ADRV', 3, op, 250 if op in (OPEN, SET) else 0, 0, 0, 1,
                              status.boot if status else 0, status.epoch if status else 0,
                              100 if op != QUERY else 0, seq, 0, bytes([key, 0, 0, 0, 0, 0]), 0)
        request = (ctypes.c_uint8 * 40).from_buffer_copy(packet)
        response = (ctypes.c_uint8 * 48)()
        self.native.exchange(request, now, response)
        self.status = decode_status(bytes(response), now / 1000)
        return self.status

    def activate_and_press(self):
        self.exchange(QUERY, 1000)
        self.assertEqual(self.exchange(OPEN, 1000).result, 0)
        self.native.tick(1001)
        self.native.complete_keyboard(1002, 1)  # OPEN barrier's zero report.
        self.assertEqual(self.exchange(SET, 1003, seq=1, key=26).result, 0)
        self.native.tick(1004)

    def diagnostic(self, now):
        self.socket.now = now
        self.client.begin(now / 1000)
        result = self.client.poll(now / 1000 + .001)
        self.assertEqual(result.kind, 'extended', result.message)
        return result.diagnostic

    def test_native_capabilities_ranges_and_read_only_query_does_not_open(self):
        d = self.diagnostic(1000)
        self.assertEqual(d.version, '0.6.0')
        self.assertEqual(d.capabilities, 127)
        self.assertEqual(d.compatibility_issues(), ())
        self.assertFalse(d.stop.valid)
        self.assertFalse(d.submitted.valid)
        self.assertTrue(self.exchange(QUERY, 1001).can_open)
        self.assertFalse(self.status.active)
        self.assertEqual([REQUEST.unpack(p)[2] for p in self.socket.sent], [5])

    def test_native_submission_is_not_usb_completion_and_stop_release_retains_identity(self):
        self.activate_and_press()
        d = self.diagnostic(1005)
        self.assertEqual((d.submitted.seq, d.submitted.time_ms), (1, 1004))
        self.assertIsNone(d.submit_to_complete_ms())
        self.native.complete_keyboard(1006, 1)
        d = self.diagnostic(1010)
        self.assertEqual(d.submit_to_complete_ms(), 2)
        old_epoch = self.status.epoch
        self.exchange(STOP, 1011, seq=2)
        self.native.tick(1012)
        d = self.diagnostic(1013)
        self.assertEqual(d.stop.reason, 1)
        self.assertEqual((d.stop.epoch, d.stop.session), (old_epoch, 100))
        self.assertIsNone(d.stop_to_release_complete_ms())
        self.native.complete_keyboard(1014, 1)
        d = self.diagnostic(1015)
        self.assertEqual((d.release_completed.epoch, d.release_completed.session), (old_epoch, 100))
        self.assertEqual(d.stop_to_release_complete_ms(), 3)
        self.assertEqual(d.submit_to_complete_ms(), 2)
        self.assertTrue(d.submitted.release)
        self.assertTrue(d.completed.release)

    def test_native_diagnostics_cannot_renew_lease_or_overwrite_initial_expiry_reason(self):
        self.activate_and_press()
        self.native.complete_keyboard(1006, 1)
        self.diagnostic(1200)
        d = self.diagnostic(1254)
        self.assertEqual(d.stop.reason, 2)
        self.assertEqual(d.stop.time_ms, 1254)
        self.native.tick(1255)  # Core 0 publishes cancellation before zero submission.
        self.native.complete_keyboard(1256, 1)
        self.exchange(STOP, 1257, seq=2)
        later = self.diagnostic(1258)
        self.assertEqual(later.stop, d.stop)
        self.assertEqual(later.stop_to_release_complete_ms(), 2)

    def test_native_error_counters_and_loop_gaps_are_observable(self):
        self.activate_and_press()
        self.native.complete_keyboard(1006, 1)
        self.native.note_send_error()
        self.native.note_send_error()
        self.native.network_heartbeat_us(1000000)
        self.native.network_heartbeat_us(1002500)
        self.native.complete_report(1007, 0, (ctypes.c_uint8 * 6)(7, 0, 0, 0, 0, 0), 1)
        d = self.diagnostic(1010)
        self.assertEqual(d.net_send_errors, 2)
        self.assertEqual(d.usb_unmatched_completions, 1)
        self.assertEqual(d.core1_max_gap_us, 2500)
        self.assertGreaterEqual(d.core0_max_gap_us, 6000)
        self.native.complete_keyboard(1011, 0)
        d = self.diagnostic(1012)
        self.assertEqual(d.stop.reason, 5)
        self.assertEqual(d.usb_errors, 1)


if __name__ == '__main__':
    unittest.main()
