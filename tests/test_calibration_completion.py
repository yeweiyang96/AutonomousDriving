"""Regression for game motion continuing after Pico's USB zero report."""
from dataclasses import asdict, replace
import math
from pathlib import Path
import queue
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

from adrive.controller import Calibration
from adrive.models import Flags, Point, Telemetry
from adrive.motion_protocol import MotionClient
from adrive.pico import PicoError
from adrive.runtime import Runtime
from adrive.settings import Settings


class CalibrationCompletionTests(unittest.TestCase):
    def setUp(self):
        self.runtime = Runtime.__new__(Runtime)
        r = self.runtime
        r.settings = Settings(left_key='Q', right_key='E', strafe_left_key='A', strafe_right_key='D')
        r.lease = Mock(spec=MotionClient)
        r.controller = Mock()
        r.log = Mock(); r._emit = Mock(); r._status = Mock(); r._lease_renew = Mock()
        r._release = Mock(); r.state = 'calibrating'
        r.stop_requested = threading.Event(); r.quit = threading.Event(); r.dispatch_lock = threading.Lock()
        r._calibration_epoch = r.control_epoch = 0
        r.calibration_samples = []; r.movement_samples = []; r._calibration_candidate = None
        self.frame = Telemetry(1, 1, 0, 1, 10, 10, 2.175377, 0, Flags.VALID, 100,
                               z=0, protocol_version=4, ground_speed=7)
        self.status = Mock(applied_seq=10, hold_expired=True, pending=False, keys=(), modifier=0)
        r.pending = dict(key=20, duration=100, before=self.frame, sent=100.,
                         lease_seq=10, calibration=True, first_change=None)

    def feed(self, seq, at, facing, **kw):
        t = replace(self.frame, seq=seq, captured_at=at, facing=facing, **kw)
        self.runtime._lease_pending_step(t, at, self.status)
        return t

    def test_recorded_late_rotation_is_included_before_next_action(self):
        # Actual angles from 20261030-104611, Q at t=368.185412:
        # old code accepted 2.420430, next game frame reached 2.489459.
        r = self.runtime
        self.feed(2, 100.102, 2.420430)  # USB zero first observed
        self.feed(3, 100.114, 2.420430)  # old code completed here
        self.assertEqual(r.calibration_samples, [])
        self.feed(4, 100.153, 2.489459)
        self.feed(5, 100.186, 2.489459)
        self.assertIsNotNone(r.pending)
        self.feed(6, 100.219, 2.489459)
        self.assertIsNone(r.pending)
        self.assertAlmostEqual(r.calibration_samples[0][1], 3.14082, places=5)
        r._lease_renew.assert_called()
        r.lease.set_keys.assert_not_called()

    def test_duplicate_frames_cannot_prove_stationarity(self):
        self.feed(2, 100.11, 2.489459)
        self.feed(3, 100.15, 2.489459)
        for _ in range(8): self.feed(3, 100.15, 2.489459)
        self.assertIsNotNone(self.runtime.pending)
        self.feed(4, 100.19, 2.489459)
        self.assertIsNotNone(self.runtime.pending)
        self.feed(5, 100.23, 2.489459)
        self.assertIsNone(self.runtime.pending)

    def test_live_speed_blocks_false_quantized_translation_plateau(self):
        self.runtime.pending.update(translation=True, keys=(4,))
        self.runtime._finish_calibration_sample = Mock()
        for seq in range(2, 7):
            self.feed(seq, 100 + seq * .04, self.frame.facing, x=10.25, speed=.5)
        self.runtime._finish_calibration_sample.assert_not_called()
        for seq in range(7, 10):
            self.feed(seq, 100 + seq * .04, self.frame.facing, x=10.25, speed=0)
        self.runtime._finish_calibration_sample.assert_called_once()

    def test_missing_game_effect_times_out_without_accepting_zero_rate(self):
        for seq in range(2, 20):
            self.feed(seq, 100 + seq * .04, self.frame.facing)
        self.assertEqual(self.runtime.calibration_samples, [])
        with self.assertRaisesRegex(PicoError, '观察超时'):
            self.feed(30, 101.2, self.frame.facing)

    def test_yaw_candidate_does_not_publish_partial_calibration(self):
        r = self.runtime
        with patch.object(Settings, 'save') as save:
            for i, key in enumerate((20, 8, 20, 8)):
                r.pending = dict(key=key, duration=100, before=self.frame, sent=100,
                                 first_change=100.03)
                after = replace(self.frame, facing=self.frame.facing + (.314 if key == 20 else -.314))
                r._finish_calibration_sample(after, 100.2)
            save.assert_not_called()
        self.assertIsNone(r.settings.calibration)
        self.assertIsNone(r.settings.movement_calibration)
        r.controller.set_calibration.assert_not_called()
        self.assertAlmostEqual(r._calibration_candidate.left_rate_rad_s, 3.14)

    def test_save_failure_does_not_publish_half_of_a_model(self):
        r = self.runtime; r._calibration_candidate = Calibration(math.pi, math.pi, .04)
        start = replace(self.frame, facing=0)
        with patch.object(Settings, 'save', side_effect=OSError('fixture disk full')):
            for direction in (math.pi/2, -math.pi/2, math.pi/4, -math.pi/4) * 2:
                r.pending = dict(before=start, translation=True)
                after = replace(start, x=10+.84*math.cos(direction), y=10+.84*math.sin(direction))
                if len(r.movement_samples) == 7:
                    with self.assertRaises(OSError): r._finish_translation_sample(after, 100.2)
                else: r._finish_translation_sample(after, 100.2)
        self.assertIsNone(r.settings.calibration)
        self.assertIsNone(r.settings.movement_calibration)
        r._emit.assert_not_called()

    def test_binding_error_mentions_actual_turn_and_strafe_keys(self):
        with self.assertRaisesRegex(PicoError, 'Q/E'):
            self.runtime._finish_calibration_sample(self.frame, 100.2)
        self.runtime._calibration_candidate = Calibration(math.pi, math.pi, .04)
        with self.assertRaisesRegex(PicoError, 'A/D'):
            self.runtime._finish_translation_sample(replace(self.frame, x=10.5, facing=2.4), 100.2)

    def test_stop_or_reconfigured_epoch_before_final_commit_discards_model(self):
        for cancellation in ('stop', 'epoch'):
            with self.subTest(cancellation=cancellation):
                self.setUp()
                r = self.runtime; r._calibration_candidate = Calibration(math.pi, math.pi, .04)
                r.movement_samples = [math.pi/2, -math.pi/2, math.pi/4, -math.pi/4,
                                      math.pi/2, -math.pi/2, math.pi/4]
                start = replace(self.frame, facing=0)
                r.pending = dict(before=start, translation=True)
                after = replace(start, x=10+.84/math.sqrt(2), y=10-.84/math.sqrt(2))
                def cancel_before_acceptance(*args, **kwargs):
                    if cancellation == 'stop': r.stop_requested.set()
                    else: r.control_epoch += 1
                    return replace(*args, **kwargs)
                with patch('adrive.runtime.replace', side_effect=cancel_before_acceptance), patch.object(Settings, 'save') as save:
                    r._finish_translation_sample(after, 100.2)
                save.assert_not_called()
                self.assertIsNone(r.settings.calibration)
                self.assertIsNone(r.settings.movement_calibration)
                r._emit.assert_not_called()

    def test_stop_during_lease_poll_cannot_complete_pending_sample(self):
        r = self.runtime
        def stop_during_poll(now):
            r.stop_requested.set()
            return self.status
        r._poll_lease = Mock(side_effect=stop_during_poll)
        r._lease_pending_step = Mock()
        r._lease_calibrating_step(self.frame, 100.2)
        r._lease_pending_step.assert_not_called()


class CalibrationArmingTests(unittest.TestCase):
    def runtime(self, settings):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        r = Runtime(None, settings, queue.Queue(), diagnostic_factory=None, log_folder=Path(temp.name))
        r.lease = Mock(spec=MotionClient)
        self.addCleanup(r.pool.shutdown)
        self.addCleanup(r.log.close)
        return r

    def test_arm_and_f8_reject_yaw_only_before_waiting_for_focus(self):
        for kind in ('arm', 'toggle'):
            with self.subTest(kind=kind):
                r = self.runtime(Settings(calibration=asdict(Calibration(math.pi, math.pi))))
                r.goal = Point(0, 10, 10)
                r.command(kind, goal=r.goal)
                r._commands()
                self.assertEqual(r.state, 'paused')
                self.assertIn('转向与平移校准', r.reason)
                self.assertTrue(r.stop_requested.is_set())
                r.lease.begin.assert_not_called()
                messages = list(r.events.queue)
                self.assertFalse(any('等待切回游戏' in e.get('text', '') for e in messages))

    def test_new_calibration_invalidates_previous_models_and_cancellation_discards_candidate(self):
        settings = Settings(calibration=asdict(Calibration(math.pi, math.pi)), movement_calibration={
            'strafe_left_angle': math.pi/2, 'strafe_right_angle': -math.pi/2,
            'diagonal_left_angle': math.pi/4, 'diagonal_right_angle': -math.pi/4, 'strafe_verified': True})
        r = self.runtime(settings)
        with patch.object(Settings, 'save') as save:
            r.command('calibrate'); r._commands()
            save.assert_called_once()
        self.assertIsNone(r.settings.calibration)
        self.assertIsNone(r.settings.movement_calibration)
        self.assertIn('calibration_reset', [e['kind'] for e in r.events.queue])
        r._calibration_candidate = Calibration(math.pi, math.pi)
        r.command('stop'); r._commands()
        self.assertIsNone(r._calibration_candidate)
        self.assertEqual(r.state, 'stopped')
