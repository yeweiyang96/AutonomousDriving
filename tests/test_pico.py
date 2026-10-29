from dataclasses import replace
import socket
import struct
import unittest
from unittest.mock import patch

from adrive.models import Flags, Telemetry
from adrive.pico import ActionGate, Health, PicoClient, PicoError, PicoTimeout, decode_health


def health(**kwargs):
    d = dict(version=2, flags=1 | 2 | 4 | 16, queue_depth=0, queue_capacity=8,
             uptime_ms=10000, accepted=100, rejected=0, received_at=10.0)
    d.update(kwargs)
    return Health(**d)


def telemetry(**kwargs):
    d = dict(session=1, seq=10, c=0, ui_map=1429, x=0., y=0., facing=0., speed=7.,
             flags=Flags.VALID, captured_at=10.0)
    d.update(kwargs)
    return Telemetry(**d)


class FakeSocket:
    def __init__(self, mode="ok"):
        self.sent = []
        self.mode = mode
        self.reads = 0
        self.closed = False

    def connect(self, address):
        self.address = address

    def settimeout(self, timeout):
        self.timeout = timeout

    def send(self, data):
        self.sent.append(data)
        return len(data)

    def recv(self, count):
        self.reads += 1
        if self.mode == "timeout":
            raise socket.timeout()
        token = struct.unpack("<BI", self.sent[-1])[1]
        if self.mode == "stale" and self.reads == 1:
            token = (token + 1) & 0xFFFFFFFF
        if self.mode == "short" and self.reads == 1:
            return b"PICO"
        return struct.pack("<4sBBBBIIII", b"PICO", 2, 23, 0, 8, token, 123, 9, 0)

    def close(self):
        self.closed = True


class PicoClientTests(unittest.TestCase):
    def test_health_query_golden_bytes_and_stale_response(self):
        fake = FakeSocket("stale")
        with patch("adrive.pico.socket.socket", return_value=fake), patch("adrive.pico.secrets.randbits", return_value=0x12345678):
            p = PicoClient("127.0.0.1", 9999)
            result = p.query()
            self.assertEqual(fake.sent, [bytes.fromhex("0078563412")])
            self.assertEqual(fake.reads, 2)
            self.assertTrue(result.ready)
            self.assertEqual(result.accepted, 9)
            p.close()

    def test_bad_length_packet_ignored(self):
        fake = FakeSocket("short")
        with patch("adrive.pico.socket.socket", return_value=fake):
            with PicoClient() as p:
                self.assertTrue(p.query().ready)
        self.assertEqual(fake.reads, 2)

    def test_press_and_release_golden_bytes(self):
        fake = FakeSocket()
        with patch("adrive.pico.socket.socket", return_value=fake):
            with PicoClient() as p:
                p.press(26, 200)
                p.release_all()
        self.assertEqual(fake.sent, [bytes.fromhex("011ac800"), b"\x0a"])
        self.assertTrue(fake.closed)

    def test_query_timeout_no_retry(self):
        fake = FakeSocket("timeout")
        with patch("adrive.pico.socket.socket", return_value=fake):
            with PicoClient() as p:
                with self.assertRaises(PicoTimeout):
                    p.query()
        self.assertEqual(len(fake.sent), 1)

    def test_input_bounds_and_types(self):
        fake = FakeSocket()
        with patch("adrive.pico.socket.socket", return_value=fake):
            with PicoClient() as p:
                for key, duration in ((3, 50), (26, -20), (26, 19), (26, 201), (True, 50), (26, 20.5)):
                    with self.subTest(key=key, duration=duration), self.assertRaises(ValueError):
                        p.press(key, duration)
        self.assertEqual(fake.sent, [])

    def test_decode_rejects_foreign_token_version_and_capacity(self):
        for version, capacity, token in ((3, 8, 42), (2, 9, 42), (2, 8, 1)):
            data = struct.pack("<4sBBBBIIII", b"PICO", version, 23, 0, capacity, token, 1, 0, 0)
            with self.subTest(version=version, capacity=capacity, token=token), self.assertRaises(PicoError):
                decode_health(data, 42)


class ActionGateTests(unittest.TestCase):
    def gate(self, base=None, initial=None):
        gate = ActionGate()
        gate.begin(base or health(), initial or telemetry(), 10.0, 200)
        return gate

    def test_idle_counter_is_not_enough_without_post_action_frame(self):
        gate = self.gate()
        h = health(accepted=101, uptime_ms=10050)
        self.assertEqual(gate.observe(h, telemetry(seq=11, captured_at=10.05), 10.05), "waiting")
        self.assertEqual(gate.observe(replace(h, uptime_ms=10220), telemetry(captured_at=10.22), 10.22), "waiting")
        self.assertEqual(gate.observe(replace(h, uptime_ms=10230), telemetry(seq=11, captured_at=10.23), 10.23), "complete")
        self.assertFalse(gate.active)

    def test_busy_then_idle_waits(self):
        gate = self.gate()
        self.assertEqual(gate.observe(health(accepted=101, flags=31, uptime_ms=10220), telemetry(seq=11, captured_at=10.22), 10.22), "waiting")
        self.assertEqual(gate.observe(health(accepted=101, uptime_ms=10240), telemetry(seq=12, captured_at=10.24), 10.24), "complete")

    def test_lost_action_times_out_no_resend(self):
        gate = self.gate()
        self.assertEqual(gate.observe(health(uptime_ms=10300), telemetry(seq=11, captured_at=10.3), 10.3), "waiting")
        with self.assertRaises(PicoError):
            gate.observe(health(uptime_ms=11210), telemetry(seq=20, captured_at=11.21), 11.21)
        self.assertFalse(gate.active)

    def test_duplicate_or_concurrent_input_stops(self):
        gate = self.gate()
        with self.assertRaisesRegex(PicoError, "重复"):
            gate.observe(health(accepted=102, uptime_ms=10220), telemetry(seq=11, captured_at=10.22), 10.22)

    def test_rejection_after_accept_stops(self):
        gate = self.gate()
        with self.assertRaisesRegex(PicoError, "拒绝"):
            gate.observe(health(accepted=101, rejected=1, uptime_ms=10220), telemetry(seq=11, captured_at=10.22), 10.22)

    def test_reset_and_out_of_order_health_stops(self):
        gate = self.gate()
        with self.assertRaisesRegex(PicoError, "重启"):
            gate.observe(health(accepted=0, uptime_ms=20), telemetry(seq=11, captured_at=10.22), 10.22)

    def test_counter_uptime_and_sequence_wrap_are_valid(self):
        gate = self.gate(health(accepted=0xFFFFFFFF, uptime_ms=0xFFFFFF00), telemetry(seq=0xFFFFFFFF))
        h = health(accepted=0, uptime_ms=20)
        self.assertEqual(gate.observe(h, telemetry(seq=0, captured_at=10.28), 10.28), "complete")

    def test_plugin_reload_and_reordered_frame_stop(self):
        for t in (telemetry(session=2, seq=11, captured_at=10.22), telemetry(seq=9, captured_at=10.22)):
            with self.subTest(t=t), self.assertRaises(PicoError):
                self.gate().observe(health(accepted=101, uptime_ms=10220), t, 10.22)

    def test_stale_frame_does_not_complete(self):
        gate = self.gate()
        self.assertEqual(gate.observe(health(accepted=101, uptime_ms=10600), telemetry(seq=11, captured_at=10.22), 10.6), "waiting")

    def test_second_action_cannot_begin(self):
        gate = self.gate()
        with self.assertRaises(PicoError):
            gate.begin(health(), telemetry(), 10, 100)

    def test_old_health_cannot_authorize_input(self):
        with self.assertRaisesRegex(PicoError, "健康状态已过期"):
            ActionGate().begin(health(received_at=9.7), telemetry(), 10, 100)

    def test_usb_or_wifi_loss_stops(self):
        for flags in (22, 7):
            with self.subTest(flags=flags), self.assertRaises(PicoError):
                self.gate().observe(health(flags=flags), telemetry(seq=11, captured_at=10.22), 10.22)


if __name__ == "__main__":
    unittest.main()
