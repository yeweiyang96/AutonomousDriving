"""Runtime integration without screen access, hardware packets, or real input.

An immediate fake executor makes planning deterministic while retaining real
Future state. A monotonic fake clock keeps post-action and stale-frame cases
reproducible. Every side-effect destination is inside a temporary log folder.
"""
from concurrent.futures import Future
from dataclasses import asdict, replace
import math
from pathlib import Path
import queue
import tempfile
import unittest
from unittest.mock import patch

from adrive.controller import Calibration
from adrive.models import Flags, Point, Route, Segment, Telemetry
from adrive.pico import Health, PicoError, PicoTimeout
from adrive.runtime import Runtime
from adrive.settings import ROOT, Settings


class Clock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds
        return self.now


class FakeCapture:
    def __init__(self, window_title="", **kwargs):
        self.title = window_title
        self.focused = True
        self.status = "fake capture"
        self.closed = False

    def foreground(self):
        return self.focused

    def poll(self):
        return None

    def close(self):
        self.closed = True


class FakePico:
    def __init__(self, clock, **kwargs):
        self.clock = clock
        self.queries = 0
        self.actions = []
        self.accepted = 100
        self.rejected = 0
        self.on_query = None
        self.closed = False
        self.busy = False

    def query(self):
        self.queries += 1
        if self.on_query:
            self.on_query()
        return Health(2, 23 | (8 if self.busy else 0), 0, 8,
                      round(self.clock() * 1000), self.accepted, self.rejected, self.clock())

    def press(self, key, duration):
        self.actions.append(("press", key, duration))
        self.accepted += 1

    def release_all(self):
        self.actions.append(("release",))
        self.accepted += 1

    def close(self):
        self.closed = True


class ImmediateExecutor:
    def submit(self, function, *args, **kwargs):
        result = Future()
        try:
            result.set_result(function(*args, **kwargs))
        except Exception as exc:
            result.set_exception(exc)
        return result

    def shutdown(self, **kwargs):
        pass


class FakePlanner:
    def __init__(self, calls):
        self.calls = calls

    def plan(self, start, goal, **kwargs):
        self.calls.append((start, goal, kwargs))
        return direct_route(start, goal)


def direct_route(start=Point(0, 0, 0), goal=Point(0, 100, 0)):
    return Route(start, goal, (Segment((start, goal), "test", "reviewed", "ground-0"),))


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.clock_patch = patch("adrive.runtime.time.monotonic", side_effect=self.clock)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        temp_root = ROOT / "runtime" / "tmp"
        temp_root.mkdir(parents=True, exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(prefix="runtime-test-", dir=temp_root)
        self.addCleanup(self.temporary.cleanup)
        self.runtimes = []
        self.addCleanup(self.close_runtimes)

    def close_runtimes(self):
        for runtime in self.runtimes:
            runtime.close()
            runtime.pool.shutdown(wait=True, cancel_futures=True)
            if runtime.capture:
                runtime.capture.close()
            if runtime.pico:
                runtime.pico.close()
            runtime.log.close()

    def make_runtime(self, calibrated=True):
        settings = Settings(calibration=asdict(Calibration(math.pi, math.pi)) if calibrated else None)
        calls = []
        runtime = Runtime(calls, settings, queue.Queue(), capture_factory=FakeCapture,
                          pico_factory=lambda **kw: FakePico(self.clock, **kw),
                          lease_factory=None, diagnostic_factory=None, planner_factory=FakePlanner,
                          log_folder=Path(self.temporary.name))
        runtime.pool.shutdown(wait=True)
        runtime.pool = ImmediateExecutor()
        runtime._connect_objects()
        self.runtimes.append(runtime)
        return runtime

    def sample(self, seq=1, x=0., y=0., facing=0., flags=Flags.VALID, session=1):
        return Telemetry(session, seq, 0, 1429, x, y, facing, 7., flags, self.clock())

    def feed(self, runtime, **kw):
        t = self.sample(**kw)
        self.assertTrue(runtime.tracker.accept(t, self.clock()))
        return t

    def running(self, runtime):
        runtime.goal = Point(0, 100, 0)
        runtime.controller.set_route(direct_route())
        runtime.route = runtime.controller.route
        runtime.state = "running"
        runtime.task_new = False
        runtime.stop_requested.clear()
        return self.feed(runtime)

    def finish_plan(self, runtime, telemetry):
        runtime._active_step(telemetry, self.clock())
        self.assertEqual(runtime.state, "planning")
        runtime._active_step(telemetry, self.clock())
        self.assertEqual(runtime.state, "running")

    def test_default_monitoring_does_not_query_or_send(self):
        runtime = self.make_runtime()
        t = self.feed(runtime)
        for _ in range(3):
            runtime._active_step(t, self.clock())
        self.assertEqual(runtime.state, "monitoring")
        self.assertEqual(runtime.pico.actions, [])
        self.assertEqual(runtime.pico.queries, 0)
        self.assertEqual(runtime.bundle, [])

    def test_missing_height_rejects_precise_route_before_running(self):
        from concurrent.futures import Future
        runtime = self.make_runtime()
        t = self.feed(runtime)
        runtime.state = 'planning'
        runtime.future_generation = runtime.generation
        runtime.future = Future()
        runtime.future.set_result(Route(t.point, Point(0, 10, 0, 2), (
            Segment((Point(0, 0, 0, 2), Point(0, 10, 0, 2)), 'test', 'navmesh', 'precise'),)))
        runtime._active_step(t, self.clock())
        self.assertEqual(runtime.state, 'fault')
        self.assertIn('角色高度', runtime.reason)
        self.assertFalse(any(a[0] == 'press' for a in runtime.pico.actions))

    def test_explicit_arm_plans_from_latest_not_preview_start(self):
        runtime = self.make_runtime()
        runtime.route = direct_route(Point(0, -20, 0))  # Old preview only.
        self.feed(runtime, x=3)
        runtime.command("arm", goal=Point(0, 100, 0), allow_candidates=True)
        runtime._commands()
        self.assertEqual(runtime.state, "armed")
        self.assertEqual(runtime.pico.actions, [])
        self.clock.advance(.03)
        latest = self.feed(runtime, seq=2, x=5)
        self.finish_plan(runtime, latest)
        self.assertEqual(runtime.bundle[0][0], Point(0, 5, 0))
        self.assertTrue(runtime.bundle[0][2]["allow_candidates"])
        self.assertEqual(runtime.route.start, Point(0, 5, 0))
        self.assertEqual(runtime.pico.actions, [])
        runtime._active_step(latest, self.clock())
        self.assertEqual(runtime.pico.actions[0][0], "press")

    def test_arm_waits_for_game_focus_without_input(self):
        runtime = self.make_runtime()
        t = self.feed(runtime)
        runtime.capture.focused = False
        runtime.command("arm", goal=Point(0, 100, 0))
        runtime._commands()
        runtime._active_step(t, self.clock())
        self.assertEqual(runtime.state, "armed")
        self.assertEqual(runtime.bundle, [])
        self.assertEqual(runtime.pico.actions, [])

    def test_missing_calibration_never_arms(self):
        runtime = self.make_runtime(calibrated=False)
        self.feed(runtime)
        runtime.command("arm", goal=Point(0, 100, 0))
        runtime._commands()
        self.assertEqual(runtime.state, "paused")
        self.assertEqual(runtime.pico.actions, [])
        self.assertEqual(runtime.bundle, [])

    def test_stop_during_health_query_prevents_new_press(self):
        runtime = self.make_runtime()
        t = self.running(runtime)
        runtime.pico.on_query = lambda: runtime.command("stop")
        with self.assertRaisesRegex(PicoError, "停止请求"):
            runtime._send(26, 100, t)
        self.assertEqual(runtime.pico.actions, [])
        self.assertIsNone(runtime.pending)
        runtime._commands()
        self.assertEqual(runtime.state, "stopped")
        self.assertTrue(runtime.stop_requested.is_set())
        self.assertNotIn("press", [a[0] for a in runtime.pico.actions])

    def test_pause_during_health_query_prevents_new_press(self):
        runtime = self.make_runtime()
        t = self.running(runtime)
        runtime.pico.on_query = lambda: runtime.command("pause")
        with self.assertRaises(PicoError):
            runtime._send(26, 100, t)
        self.assertEqual(runtime.pico.actions, [])

    def test_slow_health_query_cannot_use_stale_visual_frame(self):
        runtime = self.make_runtime()
        t = self.running(runtime)
        runtime.pico.on_query = lambda: self.clock.advance(.3)
        with self.assertRaisesRegex(PicoError, "遥测已失效"):
            runtime._send(26, 100, t)
        self.assertEqual(runtime.pico.actions, [])

    def test_focus_loss_stops_and_does_not_auto_resume(self):
        runtime = self.make_runtime()
        t = self.running(runtime)
        runtime.capture.focused = False
        runtime._active_step(t, self.clock())
        self.assertEqual(runtime.state, "paused")
        self.assertEqual(runtime.pico.actions, [("release",)])
        runtime.capture.focused = True
        self.clock.advance(.03)
        t = self.feed(runtime, seq=2)
        runtime._active_step(t, self.clock())
        self.assertEqual(runtime.state, "paused")
        self.assertEqual(runtime.pico.actions, [("release",)])

    def test_frozen_sequence_stops_after_250ms(self):
        runtime = self.make_runtime()
        t = self.running(runtime)
        self.clock.advance(.251)
        self.assertFalse(runtime.tracker.accept(replace(t, captured_at=self.clock()), self.clock()))
        runtime._active_step(runtime.tracker.latest, self.clock())
        self.assertEqual(runtime.state, "paused")
        self.assertIn("250", runtime.reason)
        self.assertEqual(runtime.pico.actions, [("release",)])

    def test_unsafe_states_stop_before_sending(self):
        for flag in Flags:
            if flag == Flags.VALID:
                continue
            with self.subTest(flag=flag):
                runtime = self.make_runtime()
                t = self.running(runtime)
                self.clock.advance(.03)
                t = self.feed(runtime, seq=2, flags=Flags.VALID | flag)
                runtime._active_step(t, self.clock())
                self.assertEqual(runtime.state, "paused")
                self.assertEqual(runtime.pico.actions, [("release",)])

    def test_health_timeout_is_propagated_without_input(self):
        runtime = self.make_runtime()
        t = self.running(runtime)
        def timeout():
            raise PicoTimeout("fixture timeout")
        runtime.pico.on_query = timeout
        with self.assertRaises(PicoTimeout):
            runtime._send(26, 100, t)
        self.assertEqual(runtime.pico.actions, [])

    def test_repeated_control_tick_does_not_queue_second_action(self):
        runtime = self.make_runtime()
        t = self.running(runtime)
        runtime._active_step(t, self.clock())
        self.assertEqual(len(runtime.pico.actions), 1)
        self.clock.advance(.04)
        t = self.feed(runtime, seq=2, x=.3)
        runtime._active_step(t, self.clock())
        self.assertIsNotNone(runtime.pending)
        self.assertEqual(len(runtime.pico.actions), 1)
        self.clock.advance(.18)
        t = self.feed(runtime, seq=3, x=1.4)
        runtime._active_step(t, self.clock())
        self.assertIsNone(runtime.pending)
        self.assertEqual(len(runtime.pico.actions), 1)
        runtime._active_step(t, self.clock())
        self.assertEqual(len(runtime.pico.actions), 2)

    def test_stop_invalidates_pending_plan(self):
        runtime = self.make_runtime()
        t = self.feed(runtime)
        runtime.command("arm", goal=Point(0, 100, 0))
        runtime._commands()
        runtime._active_step(t, self.clock())
        old_future = runtime.future
        old_generation = runtime.generation
        runtime.command("stop")
        runtime._commands()
        self.assertIsNone(runtime.future)
        self.assertGreater(runtime.generation, old_generation)
        self.assertTrue(runtime.plan_cancel.is_set())
        self.assertTrue(old_future.done())
        runtime._active_step(t, self.clock())
        self.assertEqual(runtime.state, "stopped")
        self.assertNotIn("press", [a[0] for a in runtime.pico.actions])

    def test_displacement_while_planning_replans_latest_position(self):
        runtime = self.make_runtime()
        t = self.feed(runtime)
        runtime.command("arm", goal=Point(0, 100, 0))
        runtime._commands()
        runtime._active_step(t, self.clock())
        self.clock.advance(.03)
        latest = self.feed(runtime, seq=2, x=8)
        runtime._active_step(latest, self.clock())
        self.assertEqual(runtime.state, "planning")
        self.assertEqual(len(runtime.bundle), 2)
        self.assertEqual(runtime.bundle[-1][0], latest.point)
        self.assertEqual(runtime.pico.actions, [])

    def test_explicit_continue_accepts_new_session_preserving_budget(self):
        runtime = self.make_runtime()
        t = self.running(runtime)
        runtime.controller.update(t, self.clock())  # Establish old controller session.
        runtime.controller.replan_count = 1
        runtime.controller.blocked_edges.add("failed-road")
        runtime._halt("plugin reload")
        self.clock.advance(.03)
        latest = self.feed(runtime, seq=1, session=2)
        runtime.command("toggle")
        runtime._commands()
        self.finish_plan(runtime, latest)
        self.assertEqual(runtime.controller.replan_count, 1)
        self.assertEqual(runtime.bundle[-1][2]["blocked_edges"], frozenset({"failed-road"}))
        runtime._active_step(latest, self.clock())
        self.assertEqual(runtime.pico.actions[-1][0], "press")

    def calibration_sample(self, runtime, left, rate):
        old = self.sample(seq=10, facing=0.)
        runtime.pending = {"key": 4 if left else 7, "duration": 100, "before": old,
                           "sent": self.clock(), "first_change": self.clock() + .03}
        self.clock.advance(.12)
        new = self.sample(seq=11, facing=(rate * .1 * (1 if left else -1)) % math.tau)
        runtime._finish_calibration_sample(new, self.clock())

    def test_unstable_calibration_is_rejected_without_saving(self):
        runtime = self.make_runtime(calibrated=False)
        runtime.state = "calibrating"
        with patch.object(Settings, "save") as save:
            for left, rate in ((True, 3), (False, 3), (True, 5)):
                self.calibration_sample(runtime, left, rate)
            with self.assertRaisesRegex(PicoError, "25%"):
                self.calibration_sample(runtime, False, 3)
            save.assert_not_called()
        self.assertIsNone(runtime.settings.calibration)
        self.assertIsNone(runtime.controller.calibration)

    def test_stable_calibration_persists_rates(self):
        runtime = self.make_runtime(calibrated=False)
        runtime.state = "calibrating"
        with patch.object(Settings, "save") as save:
            for left, rate in ((True, 3), (False, 4), (True, 3), (False, 4)):
                self.calibration_sample(runtime, left, rate)
            save.assert_called_once()
        self.assertEqual(runtime.state, "monitoring")
        self.assertAlmostEqual(runtime.settings.calibration["left_rate_rad_s"], 3)
        self.assertAlmostEqual(runtime.settings.calibration["right_rate_rad_s"], 4)
        self.assertEqual(runtime.pico.actions, [])  # Samples above were simulated.

    def test_wrong_direction_calibration_rejected(self):
        runtime = self.make_runtime(calibrated=False)
        with self.assertRaisesRegex(PicoError, "正确转向"):
            self.calibration_sample(runtime, True, -3)

    def test_binding_change_invalidates_previous_calibration(self):
        runtime = self.make_runtime()
        replacement = replace(runtime.settings, left_key="Q")
        runtime.command("configure", settings=replacement)
        runtime._commands()
        self.assertIsNone(runtime.settings.calibration)
        self.assertIsNone(runtime.controller.calibration)
        self.assertEqual(runtime.state, "monitoring")

    def test_older_arm_cannot_clear_a_later_stop_request(self):
        runtime = self.make_runtime()
        runtime.command("arm", goal=Point(0, 100, 0))
        old_epoch = runtime.control_epoch
        runtime.command("stop")
        self.assertGreater(runtime.control_epoch, old_epoch)
        self.assertTrue(runtime.stop_requested.is_set())
        with patch.object(runtime.stop_requested, "clear", wraps=runtime.stop_requested.clear) as clear:
            runtime._commands()
            # Check the intermediate authorization decision, not merely the
            # final state after the queued stop happens to be processed.
            clear.assert_not_called()
        self.assertEqual(runtime.state, "stopped")
        self.assertTrue(runtime.stop_requested.is_set())
        self.assertEqual(runtime.pico.actions, [])

    def test_older_calibrate_cannot_clear_a_later_stop_request(self):
        runtime = self.make_runtime()
        runtime.command("calibrate")
        runtime.command("stop")
        with patch.object(runtime.stop_requested, "clear", wraps=runtime.stop_requested.clear) as clear:
            runtime._commands()
            clear.assert_not_called()
        self.assertEqual(runtime.state, "stopped")
        self.assertTrue(runtime.stop_requested.is_set())
        self.assertEqual(runtime.pico.actions, [])

    def test_older_toggle_cannot_clear_a_later_stop_request(self):
        runtime = self.make_runtime()
        runtime.goal = Point(0, 100, 0)
        runtime.command("toggle")
        runtime.command("stop")
        with patch.object(runtime.stop_requested, "clear", wraps=runtime.stop_requested.clear) as clear:
            runtime._commands()
            clear.assert_not_called()
        self.assertEqual(runtime.state, "stopped")
        self.assertTrue(runtime.stop_requested.is_set())

    def test_new_explicit_arm_after_stop_can_authorize_again(self):
        runtime = self.make_runtime()
        runtime.command("stop")
        epoch = runtime.control_epoch
        runtime.command("arm", goal=Point(0, 70, 0))
        with patch.object(runtime.stop_requested, "clear", wraps=runtime.stop_requested.clear) as clear:
            runtime._commands()
            clear.assert_called_once()
        self.assertEqual(runtime.control_epoch, epoch)
        self.assertEqual(runtime.state, "armed")
        self.assertFalse(runtime.stop_requested.is_set())
        latest = self.feed(runtime, x=4)
        self.finish_plan(runtime, latest)
        self.assertEqual(runtime.route.goal, Point(0, 70, 0))
        self.assertEqual(runtime.route.start, latest.point)

    def test_foreground_unsafe_armed_states_pause_without_automatic_restart(self):
        for kind in ("arm", "calibrate"):
            for flags in (Flags.VALID | Flags.COMBAT, Flags(0)):
                with self.subTest(kind=kind, flags=flags):
                    runtime = self.make_runtime()
                    unsafe = self.feed(runtime, flags=flags)
                    runtime.capture.focused = False
                    runtime.command(kind, **({"goal": Point(0, 100, 0)} if kind == "arm" else {}))
                    runtime._commands()
                    expected_armed = "armed" if kind == "arm" else "calibration_armed"
                    self.assertEqual(runtime.state, expected_armed)
                    runtime._active_step(unsafe, self.clock())
                    self.assertEqual(runtime.state, expected_armed)
                    runtime.capture.focused = True
                    runtime._active_step(unsafe, self.clock())
                    self.assertEqual(runtime.state, "paused")
                    self.assertTrue(runtime.stop_requested.is_set())
                    self.clock.advance(.03)
                    recovered = self.feed(runtime, seq=2, flags=Flags.VALID)
                    runtime._active_step(recovered, self.clock())
                    self.assertEqual(runtime.state, "paused")
                    self.assertEqual(runtime.bundle, [])
                    self.assertNotIn("press", [a[0] for a in runtime.pico.actions])


if __name__ == "__main__":
    unittest.main()
