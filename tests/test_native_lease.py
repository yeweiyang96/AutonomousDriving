"""Python client <-> production firmware C, via a USB-free host DLL.

Build ../hid_usb/scripts/build_host_lease.ps1 first. The DLL links production
bridge/protocol code; only time, the legacy idle engine and USB sink are fakes.
"""
import ctypes
import os
from pathlib import Path
import unittest
from unittest.mock import patch

from adrive.lease import LeaseClient, REQUEST, RESPONSE, SET, RENEW
from adrive.pico import PicoError
from adrive.settings import ROOT

DLL = Path(os.environ.get('ADRV_LEASE_TEST_DLL', ROOT.parent / 'hid_usb/build-host-tests/hid_lease_host.dll'))


class NativeSocket:
    def __init__(self, native):
        self.native = native
        self.now = 10.
        self.sent, self.inbox = [], []
        self.drop_request = None
        self.drop_reply = False

    def connect(self, address):
        pass

    def setblocking(self, value):
        assert not value

    def send(self, data):
        self.sent.append(data)
        if REQUEST.unpack(data)[2] == self.drop_request:
            self.drop_request = None
            return len(data)
        request = (ctypes.c_uint8 * 40).from_buffer_copy(data)
        response = (ctypes.c_uint8 * 48)()
        self.native.exchange(request, round(self.now * 1000), response)
        if not self.drop_reply:
            self.inbox.append(bytes(response))
        self.drop_reply = False
        return len(data)

    def recv(self, count):
        if not self.inbox:
            raise BlockingIOError()
        return self.inbox.pop(0)

    def close(self):
        pass


@unittest.skipUnless(os.name == 'nt' and DLL.is_file(), '可选的生产固件 C 主机 DLL 尚未构建')
class NativeLeaseTests(unittest.TestCase):
    def setUp(self):
        self.native = ctypes.CDLL(str(DLL))
        self.native.reset.argtypes = []
        self.native.reset.restype = None
        self.native.exchange.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint8)]
        self.native.exchange.restype = None
        self.native.tick.argtypes = [ctypes.c_uint32]
        self.native.tick.restype = None
        self.native.copy_keyboard.argtypes = [ctypes.POINTER(ctypes.c_uint8)]
        self.native.copy_keyboard.restype = None
        self.native.keyboard_releases.restype = ctypes.c_uint
        self.native.keyboard_reports.restype = ctypes.c_uint
        self.native.reset()
        self.socket = NativeSocket(self.native)
        patcher = patch('adrive.lease.socket.socket', return_value=self.socket)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.client = LeaseClient()
        self.addCleanup(self.client.close)

    def tick(self, now, *, poll=True):
        self.socket.now = now
        self.native.tick(round(now * 1000))
        if poll:
            self.client.poll(now)

    def keys(self):
        report = (ctypes.c_uint8 * 7)()
        self.native.copy_keyboard(report)
        return tuple(k for k in report[1:] if k)

    def activate(self):
        self.client.begin(10.)
        for i in range(12):
            self.tick(10. + i * .01)
        self.assertTrue(self.client.is_active)

    def test_native_wire_read_only_and_open(self):
        self.client.query_only(10.)
        self.client.poll(10.)
        self.assertTrue(self.client.status.can_open)
        self.assertEqual(self.client.status.boot, 0x12345678)
        self.activate()
        self.assertEqual(self.keys(), ())

    def test_native_continuous_forward_no_periodic_release_and_two_lost_acks(self):
        self.activate()
        baseline = self.native.keyboard_releases()
        self.socket.now = 10.12
        self.client.set_keys((26,), 10.12)
        self.tick(10.13)
        for i in range(60):
            now = 10.16 + i * .05
            self.socket.now = now
            if i in (20, 21):
                self.socket.drop_reply = True
            self.client.renew(now)
            self.tick(now + .01)
            self.assertEqual(self.keys(), (26,))
        self.assertEqual(self.native.keyboard_releases(), baseline)
        self.assertEqual(self.client.status.applied_seq, 1)
        self.assertGreater(self.client.status.accepted_seq, 50)

    def test_native_short_effect_is_not_restarted_by_renewal(self):
        self.activate()
        self.socket.now = 10.12
        self.client.set_keys((4,), 10.12, hold_ms=60)
        self.tick(10.13)
        self.assertEqual(self.keys(), (4,))
        self.socket.now = 10.16
        self.client.renew(10.16)
        self.tick(10.17)
        self.tick(10.191)
        self.assertEqual(self.keys(), ())
        self.socket.now = 10.21
        self.client.renew(10.21)
        self.tick(10.22)
        self.assertTrue(self.client.status.hold_expired)
        self.assertEqual(self.client.status.applied_seq, 1)

    def test_native_lost_set_is_not_acknowledged_by_renewal(self):
        self.activate()
        self.socket.now = 10.12
        self.client.set_keys((26,), 10.12)
        self.tick(10.13)
        self.socket.drop_request = SET
        self.socket.now = 10.16
        self.client.set_keys((7,), 10.16, hold_ms=50)
        self.socket.now = 10.20
        self.client.renew(10.20)
        self.tick(10.21)
        self.assertEqual(self.client.status.accepted_seq, 3)
        self.assertEqual(self.client.status.applied_seq, 1)
        self.assertEqual(self.keys(), (26,))

    def test_native_firmware_expires_when_client_process_stops_polling(self):
        self.activate()
        self.socket.now = 10.12
        self.client.set_keys((26,), 10.12)
        self.tick(10.13)
        stale = self.socket.sent[-1]
        self.tick(10.371, poll=False)
        self.tick(10.372, poll=False)
        self.assertEqual(self.keys(), ())
        self.socket.send(stale)
        self.native.tick(10373)
        self.assertEqual(self.keys(), ())
        with self.assertRaises(PicoError):
            self.client.poll(10.374)

    def test_native_stop_repeated_packet_cannot_cancel_new_session(self):
        self.activate()
        self.socket.now = 10.12
        self.client.set_keys((26,), 10.12)
        self.tick(10.13)
        self.client.stop(10.14)
        stop = self.socket.sent[-1]
        self.tick(10.15)
        self.client.begin(10.16)
        for i in range(12):
            self.tick(10.16 + i * .01)
        self.assertTrue(self.client.is_active)
        self.client.set_keys((26,), 10.28)
        self.tick(10.29)
        self.socket.send(stop)
        self.tick(10.30)
        self.assertTrue(self.client.is_active)
        self.assertEqual(self.keys(), (26,))


if __name__ == '__main__':
    unittest.main()
