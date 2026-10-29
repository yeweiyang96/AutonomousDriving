"""Loss/reordering and lease lifecycle, entirely offline (no physical socket)."""
import unittest
from unittest.mock import patch

from adrive.lease import ACK_TIMEOUT, OPEN, QUERY, RENEW, REQUEST, RESPONSE, SET, STOP, LeaseClient, decode_status
from adrive.pico import PicoError


class Socket:
    def __init__(self):
        self.sent, self.inbox = [], []
        self.closed = False

    def connect(self, address):
        self.address = address

    def setblocking(self, value):
        assert value is False

    def send(self, data):
        self.sent.append(data)
        return len(data)

    def recv(self, count):
        if not self.inbox:
            raise BlockingIOError()
        return self.inbox.pop(0)

    def close(self):
        self.closed = True


class LeaseTests(unittest.TestCase):
    def setUp(self):
        self.socket = Socket()
        patcher = patch('adrive.lease.socket.socket', return_value=self.socket)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.client = LeaseClient()
        self.addCleanup(self.client.close)

    def reply(self, packet=None, *, flags=7, epoch=11, seq=None, applied=None, keys=(),
              session=None, boot=42, uptime=10000, result=0, hold=0):
        f = REQUEST.unpack(packet or self.socket.sent[-1])
        seq = f[10] if seq is None else seq
        session = (self.client._session if f[2] == QUERY else f[9]) if session is None else session
        data = RESPONSE.pack(b'ADRS', 3, result, flags, 0, f[6], boot, epoch, session,
                             seq, seq if applied is None else applied, uptime, 250 if flags & 4 else 0,
                             hold, 0, bytes(keys).ljust(6, b'\0'), 0)
        self.socket.inbox.append(data)
        return data

    def activate(self):
        self.client.begin(10.)
        self.reply(flags=35, epoch=10, session=0)
        self.client.poll(10.01)
        self.assertEqual(REQUEST.unpack(self.socket.sent[-1])[2], OPEN)
        self.reply()
        self.client.poll(10.02)
        self.assertTrue(self.client.is_active)

    def test_wire_sizes_and_initial_request_is_empty_query(self):
        self.assertEqual((REQUEST.size, RESPONSE.size), (40, 48))
        self.client.begin(10.)
        f = REQUEST.unpack(self.socket.sent[-1])
        self.assertEqual(f[:6], (b'ADRV', 3, QUERY, 0, 0, 0))
        self.assertEqual(f[11:], (0, b'\0' * 6, 0))
        self.assertEqual(f[7:11], (0, 0, 0, 0))

    def test_read_only_diagnostic_never_opens_or_stops(self):
        self.client.query_only(10.)
        self.reply(flags=35, epoch=10, session=0)
        self.client.poll(10.01)
        self.assertEqual(self.client.state, 'idle')
        self.assertTrue(self.client.supported)
        self.client.close()
        self.assertEqual([REQUEST.unpack(p)[2] for p in self.socket.sent], [QUERY])

    def test_read_only_diagnostic_timeout_never_stops_other_sessions(self):
        self.client.query_only(10.)
        self.client.poll(10.1)
        with self.assertRaises(PicoError):
            self.client.poll(10.61)
        self.client.close()
        self.assertEqual({REQUEST.unpack(p)[2] for p in self.socket.sent}, {QUERY})

    def test_open_requires_released_usb_before_active(self):
        self.client.begin(10.)
        self.reply(flags=35, epoch=10, session=0)
        self.client.poll(10.01)
        self.reply(flags=15)
        self.client.poll(10.02)
        self.assertFalse(self.client.is_active)
        with self.assertRaises(PicoError):
            self.client.set_keys((26,), 10.03)
        self.client.poll(10.1)
        self.reply()
        self.client.poll(10.11)
        self.assertTrue(self.client.is_active)

    def test_three_seconds_forward_has_one_set_no_key_release(self):
        self.activate()
        self.client.set_keys((26,), 10.03)
        for i in range(60):
            now = 10.04 + i * .05
            self.reply(keys=(26,), uptime=round(now * 1000))
            self.client.poll(now)
            self.client.renew(now + .01)
        commands = [REQUEST.unpack(p) for p in self.socket.sent]
        self.assertEqual([f[2] for f in commands].count(SET), 1)
        self.assertNotIn(STOP, [f[2] for f in commands])
        self.assertEqual(next(f[12] for f in commands if f[2] == SET), b'\x1a' + b'\0' * 5)
        self.assertTrue(self.client.is_active)

    def test_lost_ack_tolerated_by_next_new_state_ack(self):
        self.activate()
        self.client.set_keys((26,), 10.03)  # Its ACK is lost.
        self.client.poll(10.08)
        self.client.renew(10.09)
        self.reply(keys=(26,), uptime=10090)
        self.client.poll(10.1)
        self.assertTrue(self.client.is_active)
        self.assertEqual(self.client.last_ack_at, 10.1)
        self.assertEqual(sum(REQUEST.unpack(p)[2] == SET for p in self.socket.sent), 1)

    def test_silence_revokes_and_only_retries_stop(self):
        self.activate()
        self.client.set_keys((26,), 10.03)
        self.client.poll(10.12)
        with self.assertRaisesRegex(PicoError, '180'):
            self.client.poll(10.02 + ACK_TIMEOUT + .001)
        self.assertFalse(self.client.is_active)
        sent = len(self.socket.sent)
        self.client.poll(10.28)
        self.assertGreater(len(self.socket.sent), sent)
        self.assertEqual(REQUEST.unpack(self.socket.sent[-1])[2], STOP)
        with self.assertRaises(PicoError):
            self.client.renew(10.29)

    def test_no_renewals_is_reported_as_client_silence_not_device_nonresponse(self):
        self.activate()
        self.client.set_keys((26,), 10.03)
        self.reply(keys=(26,))
        self.client.poll(10.04)
        self.client.poll(10.14)
        with self.assertRaisesRegex(PicoError, '先前指令均已确认'):
            self.client.poll(10.23)
        detail = next(e for e in self.client.drain_events() if e['kind'] == 'lease_timeout')
        self.assertEqual(detail['cause'], 'renewal_not_sent')
        self.assertTrue(detail['all_acknowledged'])
        self.assertEqual(detail['sent_seq'], detail['acknowledged_seq'])

    def test_306ms_control_gap_preserves_stop_and_reports_scheduling_gap(self):
        self.activate()
        self.client.set_keys((26,), 10.03)
        self.reply(keys=(26,))
        self.client.poll(10.04)
        with self.assertRaisesRegex(PicoError, '控制循环间隔 306'):
            self.client.poll(10.346)
        detail = next(e for e in self.client.drain_events() if e['kind'] == 'lease_timeout')
        self.assertEqual(detail['cause'], 'client_poll_delayed')
        self.assertEqual(detail['poll_gap_ms'], 306.)
        self.assertFalse(self.client.is_active)
        self.assertEqual(REQUEST.unpack(self.socket.sent[-1])[2], STOP)

    def test_unacknowledged_request_keeps_ack_timeout_classification(self):
        self.activate()
        self.client.set_keys((26,), 10.03)
        self.client.poll(10.1)
        with self.assertRaisesRegex(PicoError, '距上次 Pico 状态确认'):
            self.client.poll(10.21)
        detail = next(e for e in self.client.drain_events() if e['kind'] == 'lease_timeout')
        self.assertEqual(detail['cause'], 'ack_wait_timeout')
        self.assertFalse(detail['all_acknowledged'])

    def test_duplicate_and_reordered_responses_do_not_refresh_ack(self):
        self.activate()
        self.client.set_keys((26,), 10.03)
        old_packet = self.socket.sent[-1]
        self.client.renew(10.07)
        self.reply(keys=(26,), uptime=10070)
        self.client.poll(10.08)
        ack = self.client.last_ack_at
        duplicate = self.reply(old_packet, keys=(26,), uptime=10030)
        self.client.poll(10.12)
        self.socket.inbox.append(duplicate)
        self.client.poll(10.15)
        self.assertEqual(self.client.last_ack_at, ack)

    def test_query_without_seq_progress_does_not_hide_lost_renewals(self):
        self.activate()
        self.client.set_keys((26,), 10.03)
        self.reply(keys=(26,), seq=0, applied=0)
        self.client.poll(10.15)
        self.assertEqual(self.client.last_ack_at, 10.02)
        with self.assertRaises(PicoError):
            self.client.poll(10.21)

    def test_timed_turn_renew_does_not_resend_effect(self):
        self.activate()
        self.client.set_keys((4,), 10.03, hold_ms=60)
        self.reply(keys=(4,), hold=60)
        self.client.poll(10.04)
        self.client.renew(10.08)
        f = REQUEST.unpack(self.socket.sent[-1])
        self.assertEqual((f[2], f[4], f[12]), (RENEW, 0, b'\0' * 6))
        self.reply(flags=23, uptime=10100)
        self.client.poll(10.1)
        self.assertTrue(self.client.status.hold_expired)
        self.assertEqual(self.client.status.keys, ())
        self.assertTrue(self.client.is_active)

    def test_expired_or_reset_session_requires_manual_begin(self):
        for values in ({'flags': 35, 'epoch': 12}, {'boot': 99}):
            with self.subTest(values=values):
                self.activate()
                self.client.renew(10.03)
                self.reply(**values)
                with self.assertRaises(PicoError):
                    self.client.poll(10.04)
                self.assertEqual(self.client.state, 'fault')

    def test_stopped_late_open_ack_cannot_activate(self):
        self.client.begin(10.)
        self.reply(flags=35, epoch=10, session=0)
        self.client.poll(10.01)
        opening = self.socket.sent[-1]
        self.client.stop(10.02)
        self.reply(opening)
        self.client.poll(10.03)
        self.assertFalse(self.client.is_active)
        self.assertEqual(self.client.state, 'stopped')

    def test_stale_probe_after_open_is_ignored(self):
        self.client.begin(10.)
        first = self.socket.sent[-1]
        self.client.poll(10.09)
        second = self.socket.sent[-1]
        self.reply(second, flags=35, epoch=10, session=0)
        self.client.poll(10.1)
        self.reply(first, flags=35, epoch=10, session=0)
        self.reply(self.socket.sent[-1])
        self.client.poll(10.11)
        self.assertTrue(self.client.is_active)

    def test_old_firmware_gets_no_set_and_no_fallback(self):
        self.client.begin(10.)
        self.client.poll(10.2)
        with self.assertRaisesRegex(PicoError, 'ADR v3'):
            self.client.poll(10.61)
        self.assertFalse(self.client.supported)
        self.assertEqual({REQUEST.unpack(p)[2] for p in self.socket.sent}, {QUERY})

    def test_malformed_matching_response_revokes(self):
        self.activate()
        self.client.renew(10.03)
        packet = bytearray(self.reply())
        self.socket.inbox.clear()
        packet[7] = 1
        self.socket.inbox.append(bytes(packet))
        with self.assertRaises(PicoError):
            self.client.poll(10.04)
        self.assertFalse(self.client.is_active)

    def test_unknown_tokens_are_ignored(self):
        self.activate()
        self.client.renew(10.03)
        packet = bytearray(self.reply())
        self.socket.inbox.clear()
        packet[8:12] = b'\xff' * 4
        self.socket.inbox.append(bytes(packet))
        self.client.poll(10.04)
        self.assertEqual(self.client.last_ack_at, 10.02)

    def test_packet_validation_rejects_invalid_keys_and_hold(self):
        self.activate()
        for keys, hold in (((26, 26), 0), ((3,), 0), ((26,), 19), ((26,), 201)):
            with self.subTest(keys=keys, hold=hold), self.assertRaises(ValueError):
                self.client.set_keys(keys, 10.03, hold_ms=hold)

    def test_processing_delay_after_poll_cannot_send_late_set_or_renew(self):
        for action in ('set', 'renew'):
            with self.subTest(action=action):
                self.activate()
                self.client.drain_events()
                self.client.poll(10.1)
                sequence = self.client.seq
                before = len(self.socket.sent)
                with self.assertRaisesRegex(PicoError, '180 ms'):
                    if action == 'set':
                        self.client.set_keys((26,), 10.21)
                    else:
                        self.client.renew(10.21)
                self.assertEqual(self.client.state, 'fault')
                self.assertEqual(self.client.seq, sequence)
                self.assertEqual([REQUEST.unpack(p)[2] for p in self.socket.sent[before:]], [STOP])
                timeout = next(e for e in self.client.drain_events() if e['kind'] == 'lease_timeout')
                self.assertEqual(timeout['phase'], 'dispatch')
                self.assertTrue(timeout['all_acknowledged'])
                self.assertAlmostEqual(timeout['ack_age_ms'], 190.)

    def test_dispatch_before_ack_deadline_remains_allowed(self):
        self.activate()
        self.client.set_keys((26,), 10.19)
        self.assertEqual(REQUEST.unpack(self.socket.sent[-1])[2], SET)
        self.reply(keys=(26,))
        self.client.poll(10.195)
        self.client.renew(10.35)
        self.assertEqual(REQUEST.unpack(self.socket.sent[-1])[2], RENEW)

    def test_queued_late_ack_cannot_reauthorize_after_observation_deadline(self):
        self.activate()
        self.client.set_keys((26,), 10.03)
        self.reply(keys=(26,))  # Queued before a long client scheduling stall.
        before = len(self.socket.sent)
        with self.assertRaisesRegex(PicoError, '180 ms'):
            self.client.poll(10.226)
        self.assertEqual(self.client.last_ack_at, 10.02)
        self.assertEqual(self.client.state, 'fault')
        self.assertEqual([REQUEST.unpack(p)[2] for p in self.socket.sent[before:]], [STOP])
        self.client.poll(10.23)  # Processing the late ACK only logs it now.
        with self.assertRaises(PicoError):
            self.client.renew(10.24)
        self.assertEqual(self.client.state, 'fault')
        self.assertEqual(self.client.last_ack_at, 10.02)


if __name__ == '__main__':
    unittest.main()
