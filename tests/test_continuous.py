"""Continuous control tests use no screen capture, sockets, or physical HID."""
from dataclasses import asdict, replace
import math
from pathlib import Path
import queue
import tempfile
import threading
import unittest
from unittest.mock import patch

from adrive.continuous_controller import ContinuousController
from adrive.controller import Calibration
from adrive.lease import LeaseStatus
from adrive.models import Flags, Point, Route, Segment, Telemetry
from adrive.pico import Health, PicoError
from adrive.runtime import Runtime
from adrive.settings import ROOT, Settings


def route(*coords):
    points = tuple(Point(0, x, y) for x, y in coords)
    return Route(points[0], points[-1], tuple(
        Segment((a, b), 'fixture', 'reviewed', f'edge-{i}')
        for i, (a, b) in enumerate(zip(points, points[1:]))))


def sample(x=0., y=0., facing=0., now=10., seq=1, speed=7., flags=Flags.VALID):
    return Telemetry(7, seq, 0, 1429, x, y, facing, speed, flags, now)


class ContinuousControllerTests(unittest.TestCase):
    def make_controller(self, planned=None):
        ctl = ContinuousController(Calibration(math.pi, math.pi))
        ctl.set_route(planned or route((0, 0), (100, 0)))
        return ctl

    def test_long_straight_keeps_same_key_for_over_three_seconds(self):
        ctl = self.make_controller()
        for i in range(121):
            t = sample(x=i * 7 / 30, now=10 + i / 30, seq=i + 1)
            d = ctl.update(t, t.captured_at)
            self.assertEqual((d.kind, d.key, d.duration_ms), ('keys', 26, 0))
            self.assertIsNone(ctl.pending)

    def test_turn_and_final_approach_remain_device_timed(self):
        ctl = self.make_controller(route((0, 0), (0, 100)))
        d = ctl.update(sample(), 10)
        self.assertEqual((d.kind, d.key, d.duration_ms), ('keys', 4, 200))
        self.assertIsNotNone(ctl.pending)
        ctl = self.make_controller()
        d = ctl.update(sample(x=97), 10)
        self.assertEqual((d.kind, d.key), ('keys', 26))
        self.assertTrue(20 <= d.duration_ms <= 200)

    def test_two_seconds_without_displacement_blocks_current_edge(self):
        ctl = self.make_controller()
        for i in range(100):
            t = sample(now=10 + i / 30, seq=i + 1)
            d = ctl.update(t, t.captured_at)
            if d.kind != 'keys':
                break
        self.assertEqual(d.kind, 'replan')
        self.assertEqual(d.blocked_edge, 'edge-0')
        self.assertGreaterEqual(t.captured_at, 12)
        self.assertLess(t.captured_at, 12.1)

    def test_no_progress_is_not_counted_from_duplicate_frames(self):
        ctl = self.make_controller()
        original = sample()
        ctl.update(original, 10)
        for _ in range(100):
            self.assertEqual(ctl.update(original, 10.1).kind, 'none')
        self.assertEqual(ctl._forward_ms, 0)

    def test_corners_speed_changes_and_turn_timing(self):
        planned = route((0, 0), (30, 0), (30, 30), (65, 30))
        ctl = self.make_controller(planned)
        t = sample()
        trace = [t.point]
        straight_frames = 0
        for step in range(2000):
            d = ctl.update(t, t.captured_at)
            if d.kind == 'arrived':
                break
            self.assertEqual(d.kind, 'keys')
            elapsed = d.duration_ms / 1000 if d.duration_ms else 1 / 30
            speed = 7 if step < 90 else 14
            heading, x, y = t.facing, t.x, t.y
            if d.key == 26:
                x += math.cos(heading) * speed * elapsed
                y += math.sin(heading) * speed * elapsed
                straight_frames += d.duration_ms == 0
            else:
                heading += math.pi * elapsed * (1 if d.key == 4 else -1)
            # Screenshot follows a timed release; no movement in that extra time.
            t = sample(x, y, heading % math.tau, t.captured_at + elapsed + (.02 if d.duration_ms else 0),
                       t.seq + 1, speed)
            trace.append(t.point)
            if d.duration_ms:
                self.assertTrue(ctl.on_action_complete(t, t.captured_at))
        self.assertEqual(ctl.state, 'arrived')
        self.assertGreater(straight_frames, 90)
        for corner in planned.points[1:-1]:
            self.assertLessEqual(min(p.distance(corner) for p in trace), .61)
        self.assertLessEqual(t.point.distance(planned.goal), 2)


class FakeCapture:
    def __init__(self, **kw):
        self.focused = True
        self.status = 'fixture'

    def foreground(self):
        return self.focused

    def close(self):
        pass


class FakePico:
    def __init__(self, clock):
        self.clock = clock
        self.actions = []

    def query(self):
        return Health(2, 23, 0, 8, 1000, 0, 0, self.clock())

    def press(self, key, duration):
        self.actions.append(('press', key, duration))

    def release_all(self):
        self.actions.append(('release',))

    def close(self):
        pass


class FakeLease:
    """Device state model, with an independently expiring timed key effect."""
    def __init__(self, **kw):
        self.state = 'idle'
        self.status = None
        self.seq = self.applied = 0
        self.last_sent_at = self.last_ack_at = 0
        self.keys = ()
        self.hold_due = None
        self.hold_expired = False
        self.actions = []
        self.drop_set = self.drop_ack = False
        self.on_poll = None

    @property
    def is_active(self):
        return self.state == 'active'

    def begin(self, now):
        self.state = 'opening'
        self.last_sent_at = now
        self.actions.append(('begin', now))

    def query_only(self, now):
        self.state = 'diagnosing'
        self.actions.append(('query', now))

    def poll(self, now):
        if self.on_poll:
            self.on_poll()
        if self.state == 'opening':
            self.state = 'active'
            self.last_ack_at = now
        if self.state == 'diagnosing':
            self.state = 'idle'
            self.status = LeaseStatus(0, 35, 1, 1, 2, 0, 0, 0, round(now * 1000),
                                      0, 0, 0, (), now)
        if self.is_active:
            if self.drop_ack and now - self.last_ack_at > .18:
                self.stop(now)
                raise PicoError('fixture missing ACK')
            if self.hold_due is not None and now >= self.hold_due:
                self.keys = ()
                self.hold_expired = True
            if not self.drop_ack:
                self.last_ack_at = now
                self.status = LeaseStatus(0, 7 | (16 if self.hold_expired else 0), 1, 1, 2, 3,
                                          self.seq, self.applied, round(now * 1000), 250, 0, 0,
                                          self.keys, now)
        return self.status

    def set_keys(self, keys, now, hold_ms=0, modifier=0):
        self.seq += 1
        self.last_sent_at = now
        self.actions.append(('keys', keys, hold_ms, now))
        if not self.drop_set:
            self.applied = self.seq
            self.keys = keys
            self.hold_expired = False
            self.hold_due = now + hold_ms / 1000 if hold_ms else None
        return self.seq

    def renew(self, now):
        self.seq += 1
        self.last_sent_at = now
        self.actions.append(('renew', now))
        return self.seq

    def stop(self, now=None):
        self.actions.append(('stop', now))
        self.state = 'stopped'
        self.keys = ()

    def drain_events(self):
        return []

    def close(self):
        self.state = 'stopped'


class ContinuousRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.clock_patch = patch('adrive.runtime.time.monotonic', side_effect=lambda: self.now)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        root = ROOT / 'runtime' / 'tmp'
        root.mkdir(parents=True, exist_ok=True)
        temp = tempfile.TemporaryDirectory(dir=root)
        self.addCleanup(temp.cleanup)
        settings = Settings(calibration=asdict(Calibration(math.pi, math.pi)))
        self.runtime = Runtime(None, settings, queue.Queue(), capture_factory=FakeCapture,
                               pico_factory=lambda **kw: FakePico(lambda: self.now),
                               lease_factory=FakeLease, diagnostic_factory=None, log_folder=Path(temp.name))
        self.runtime._connect_objects()
        self.addCleanup(self.cleanup)
        self.runtime.controller.set_route(route((0, 0), (100, 0)))
        self.runtime.goal = Point(0, 100, 0)
        self.runtime.state = 'running'
        self.runtime.lease.begin(self.now)
        self.seq = 0

    def cleanup(self):
        self.runtime.close()
        self.runtime.pool.shutdown(wait=True, cancel_futures=True)
        self.runtime.lease.close()
        self.runtime.log.close()

    def feed(self, *, x=0, y=0, facing=0, flags=Flags.VALID, advance=1 / 30):
        self.now += advance
        self.seq += 1
        t = sample(x, y, facing, self.now, self.seq, flags=flags)
        self.assertTrue(self.runtime.tracker.accept(t, self.now))
        return t

    def tick(self, **kw):
        t = self.feed(**kw)
        self.runtime._active_step(t, self.now)
        return t

    def test_runtime_continuous_for_four_seconds_uses_one_set_and_no_releases(self):
        for i in range(121):
            self.tick(x=i * 7 / 30)
        lease = self.runtime.lease
        sets = [a for a in lease.actions if a[0] == 'keys']
        self.assertEqual(len(sets), 1)
        self.assertEqual(sets[0][1:3], ((26,), 0))
        self.assertGreater(len([a for a in lease.actions if a[0] == 'renew']), 40)
        self.assertFalse(any(a[0] == 'stop' for a in lease.actions))
        self.assertEqual(self.runtime.pico.actions, [])

    def test_early_new_frame_authorizes_one_due_renewal_without_extra_frame_delay(self):
        self.tick()
        before = len(self.runtime.lease.actions)
        early = self.tick(x=.2, advance=.03)
        self.assertEqual(len(self.runtime.lease.actions), before)
        self.now += .015
        self.assertAlmostEqual(self.runtime._wait_seconds(self.now, self.now), .005)
        self.now += .006
        self.runtime._active_step(early, self.now)
        self.assertEqual(self.runtime.lease.actions[-1][0], 'renew')
        self.assertEqual(len(self.runtime.lease.actions), before + 1)
        # Socket/timer wakeups cannot reuse that same frame for more renewals.
        self.now += .06
        self.runtime._active_step(early, self.now)
        self.assertEqual(len(self.runtime.lease.actions), before + 1)

    def test_command_and_close_wake_control_wait_immediately(self):
        from unittest.mock import Mock
        self.runtime._waiter = Mock()
        self.runtime.command('stop')
        self.assertTrue(self.runtime.stop_requested.is_set())
        self.runtime._waiter.wake.assert_called_once()
        self.runtime.close()
        self.assertEqual(self.runtime._waiter.wake.call_count, 2)

    def test_timed_turn_renewal_does_not_restart_effect(self):
        self.runtime.controller.set_route(route((0, 0), (0, 100)))
        t = self.tick()
        due = self.runtime.lease.hold_due
        for i in range(1, 8):
            self.tick(facing=min(i / 30, .2) * math.pi)
            self.assertEqual(self.runtime.lease.hold_due, due)
        self.assertIsNone(self.runtime.pending)
        self.assertTrue(self.runtime.lease.status.hold_expired)
        self.assertEqual(self.runtime.lease.keys, ())
        self.assertEqual(len([a for a in self.runtime.lease.actions if a[0] == 'keys']), 1)

    def test_timed_completion_requires_frame_after_observed_usb_release(self):
        self.runtime.controller.set_route(route((0, 0), (0, 100)))
        self.tick()
        # Simulate a late USB submission: effect ends later than send + 200 ms.
        self.runtime.lease.hold_due += .10
        for i in range(1, 9):
            self.tick(facing=min(i / 30, .2) * math.pi)
        self.assertIsNotNone(self.runtime.pending)
        while not self.runtime.lease.hold_expired:
            self.tick(facing=.2 * math.pi)
        observed_frame = self.runtime.pending['release_frame']
        self.assertEqual(observed_frame, (7, self.seq))
        self.assertIsNotNone(self.runtime.pending)
        self.tick(facing=.2 * math.pi)
        self.assertIsNone(self.runtime.pending)

    def test_calibration_uses_only_v3_timed_effects_and_revokes_after_four_samples(self):
        self.runtime.state = 'monitoring'
        self.runtime.lease.state = 'idle'
        self.runtime.settings.calibration = None
        self.runtime.command('calibrate')
        self.runtime._commands()
        heading = 0.
        with patch.object(Settings, 'save') as save, patch.object(self.runtime.pico, 'query', side_effect=AssertionError('legacy query')):
            for _ in range(200):
                lease = self.runtime.lease
                elapsed = .02
                if lease.keys and lease.hold_due is not None:
                    actual = min(elapsed, max(0, lease.hold_due - self.now))
                    heading += actual * math.pi * (1 if lease.keys == (4,) else -1)
                self.tick(facing=heading % math.tau, advance=elapsed)
                if len(self.runtime.calibration_samples) == 4:
                    break
            save.assert_called_once()
        self.assertEqual(len(self.runtime.calibration_samples), 4)
        self.assertEqual(self.runtime.state, 'monitoring')
        self.assertEqual(self.runtime.lease.state, 'stopped')
        self.assertEqual(self.runtime.pico.actions, [])
        states = [a for a in self.runtime.lease.actions if a[0] == 'keys']
        self.assertEqual([a[1] for a in states], [(4,), (7,), (4,), (7,)])
        self.assertTrue(all(a[2] == 100 for a in states))
        self.assertAlmostEqual(self.runtime.settings.calibration['left_rate_rad_s'], math.pi)
        self.assertAlmostEqual(self.runtime.settings.calibration['right_rate_rad_s'], math.pi)

    def test_pause_only_sends_session_stop_never_global_legacy_release(self):
        self.tick()
        self.runtime.command('pause')
        self.runtime._commands()
        self.assertEqual(self.runtime.lease.state, 'stopped')
        self.assertEqual(self.runtime.pico.actions, [])

    def test_active_diagnosis_never_blocks_on_legacy_query(self):
        self.tick()
        before = list(self.runtime.lease.actions)
        with patch.object(self.runtime.pico, 'query', side_effect=AssertionError('legacy query')):
            self.runtime.command('diagnose')
            self.runtime._commands()
        self.assertEqual(self.runtime.lease.actions, before)
        self.assertEqual(self.runtime.state, 'running')

    def test_idle_diagnosis_is_read_only_and_does_not_open_session(self):
        self.runtime.state = 'monitoring'
        self.runtime.lease.state = 'idle'
        self.runtime.lease.actions.clear()
        with patch.object(self.runtime.pico, 'query', side_effect=AssertionError('legacy query')):
            self.runtime.command('diagnose')
            self.runtime._commands()
            self.runtime._poll_lease(self.now)
        self.assertEqual([a[0] for a in self.runtime.lease.actions], ['query'])
        self.assertEqual(self.runtime.lease.state, 'idle')

    def test_stop_arriving_during_poll_prevents_new_state(self):
        self.runtime.lease.on_poll = lambda: self.runtime.command('stop')
        t = self.feed()
        with self.assertRaisesRegex(PicoError, '停止请求'):
            self.runtime._active_step(t, self.now)
        self.assertFalse(any(a[0] == 'keys' for a in self.runtime.lease.actions))
        self.runtime._commands()
        self.assertEqual(self.runtime.state, 'stopped')

    def _assert_stop_during_blocked_foreground(self, *, renew):
        self.runtime.lease.state = 'active'
        t = self.feed()
        entered = threading.Event()
        release_query = threading.Event()
        stop_queued = threading.Event()
        send_finished = threading.Event()
        failures = []

        def foreground():
            entered.set()
            release_query.wait(3)
            return True

        def send():
            try:
                self.runtime._lease_send(t, self.now, key=26, renew=renew)
            except Exception as exc:
                failures.append(exc)
            finally:
                send_finished.set()

        def stop():
            self.runtime.command('stop')
            stop_queued.set()

        sender = threading.Thread(target=send)
        stopper = threading.Thread(target=stop)
        with patch.object(self.runtime.capture, 'foreground', side_effect=foreground):
            sender.start()
            try:
                self.assertTrue(entered.wait(1))
                stopper.start()
                # Foreground remains blocked until the finally below. STOP
                # must already be latched and queued, not waiting for its lock.
                self.assertTrue(stop_queued.wait(1))
                self.assertTrue(self.runtime.stop_requested.is_set())
                self.assertFalse(send_finished.is_set())
            finally:
                release_query.set()
                sender.join(1)
                if stopper.ident is not None:
                    stopper.join(1)
        self.assertTrue(send_finished.is_set())
        self.assertEqual(len(failures), 1)
        self.assertIsInstance(failures[0], PicoError)
        self.assertIn('停止请求', str(failures[0]))
        self.assertFalse(any(a[0] in ('keys', 'renew') for a in self.runtime.lease.actions))
        self.runtime._commands()
        self.assertEqual(self.runtime.state, 'stopped')

    def test_stop_latches_while_foreground_is_blocked_before_set(self):
        self._assert_stop_during_blocked_foreground(renew=False)

    def test_stop_latches_while_foreground_is_blocked_before_renew(self):
        self._assert_stop_during_blocked_foreground(renew=True)

    def test_slow_live_foreground_check_revalidates_current_time_before_send(self):
        self.runtime.lease.state = 'active'
        t = self.feed()

        def slow_foreground():
            self.now += .26
            return True

        with patch.object(self.runtime.capture, 'foreground', side_effect=slow_foreground):
            with self.assertRaisesRegex(PicoError, '遥测已失效'):
                self.runtime._lease_send(t, t.captured_at, key=26)
        self.assertFalse(any(a[0] in ('keys', 'renew') for a in self.runtime.lease.actions))

    def test_slow_telemetry_logging_cannot_use_old_control_loop_time(self):
        t = sample(now=self.now)
        original_write = self.runtime.log.write

        def slow_log(kind, **values):
            if kind == 'telemetry':
                self.now += .26
            original_write(kind, **values)

        with patch.object(self.runtime.log, 'write', side_effect=slow_log):
            self.runtime._accept_sample(t, self.now)
        self.runtime._active_step(t, t.captured_at)
        self.assertEqual(self.runtime.state, 'paused')
        self.assertIn('250 ms', self.runtime.reason)
        self.assertFalse(any(a[0] in ('keys', 'renew') for a in self.runtime.lease.actions))

    def test_timed_pending_uses_actual_dispatch_time_after_focus_check(self):
        self.runtime.state = 'calibrating'
        self.runtime.lease.state = 'active'
        t = self.feed()

        def delayed_foreground():
            self.now += .08
            return True

        with patch.object(self.runtime.capture, 'foreground', side_effect=delayed_foreground):
            self.runtime._lease_calibrating_step(t, t.captured_at)
        self.assertEqual(self.runtime.pending['sent'], self.now)
        self.assertEqual(self.runtime.lease.last_sent_at, self.now)

    def test_accepted_renewal_does_not_mask_a_lost_set(self):
        self.runtime.lease.drop_set = True
        self.tick()
        with self.assertRaisesRegex(PicoError, '180 ms'):
            for i in range(8):
                self.tick(x=i * .2)
        self.assertEqual(self.runtime.lease.applied, 0)
        self.assertGreater(self.runtime.lease.seq, 1)

    def test_wrong_applied_keys_are_rejected(self):
        self.tick()
        self.runtime.lease.keys = (4,)
        with self.assertRaisesRegex(PicoError, '状态与目标不一致'):
            self.tick(x=.2)

    def test_missing_acks_stop_renewal(self):
        self.tick()
        self.runtime.lease.drop_ack = True
        with self.assertRaisesRegex(PicoError, 'missing ACK'):
            for i in range(10):
                self.tick(x=i * .2)
        self.assertFalse(self.runtime.lease.is_active)

    def test_duplicate_telemetry_never_renews_and_stale_stops(self):
        t = self.tick()
        before = list(self.runtime.lease.actions)
        self.now += .1
        self.runtime._active_step(t, self.now)
        self.assertEqual(self.runtime.lease.actions, before)
        self.now += .16
        self.runtime._active_step(t, self.now)
        self.assertEqual(self.runtime.state, 'paused')
        self.assertEqual(self.runtime.lease.keys, ())

    def test_focus_loss_and_combat_stop_without_automatic_resume(self):
        for flag in (Flags.COMBAT, Flags.DEAD, Flags.SWIMMING, Flags.INPUT_FOCUS):
            self.runtime.state = 'running'
            self.runtime.stop_requested.clear()
            self.runtime.lease.state = 'active'
            self.tick(flags=Flags.VALID | flag)
            self.assertEqual(self.runtime.state, 'paused')
            self.tick()
            self.assertEqual(self.runtime.state, 'paused')
        self.runtime.state = 'running'
        self.runtime.stop_requested.clear()
        self.runtime.capture.focused = False
        self.tick()
        self.assertEqual(self.runtime.state, 'paused')


if __name__ == '__main__':
    unittest.main()
