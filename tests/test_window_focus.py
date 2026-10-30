import queue
from pathlib import Path
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from adrive.app import Application
from adrive.capture import CaptureError
from adrive.models import Point
from adrive.runtime import Runtime
from adrive.settings import Settings
from adrive.window_focus import WindowFocusBackend, focus_game_window
from tests.test_ui_models import CALIBRATION, MOVEMENT_CALIBRATION


class FakeFocusBackend:
    def __init__(self):
        self.handles = [(33, '魔兽世界')]
        self.pid = 123
        self.active = 99
        self.calls = []
        self.denied = False

    def windows(self): return self.handles
    def valid(self, hwnd): return any(h == hwnd for h, _ in self.handles)
    def identity(self, hwnd): return self.pid, next((title for h, title in self.handles if h == hwnd), '')
    def foreground(self): return self.active
    def restore(self, hwnd): self.calls.append(('restore', hwnd))
    def activate(self, hwnd):
        self.calls.append(('activate', hwnd))
        if not self.denied: self.active = hwnd
        return not self.denied


class WindowFocusTests(unittest.TestCase):
    def test_exact_window_restored_and_activated_once(self):
        backend = FakeFocusBackend()
        request = focus_game_window('魔兽世界', _backend=backend)
        self.assertTrue(request.poll())
        self.assertEqual(backend.calls, [('restore', 33), ('activate', 33)])
        focus_game_window('魔兽世界', _backend=backend)
        self.assertEqual(backend.calls.count(('activate', 33)), 1)

    def test_missing_or_ambiguous_window_never_activated(self):
        for handles in ([], [(33, '魔兽世界 - Microsoft Edge')], [(33, '魔兽世界'), (34, '魔兽世界')]):
            backend = FakeFocusBackend(); backend.handles = handles
            with self.subTest(handles=handles), self.assertRaises(CaptureError):
                focus_game_window('魔兽世界', _backend=backend)
            self.assertEqual(backend.calls, [])

    def test_recycled_handle_during_restore_cannot_activate(self):
        backend = FakeFocusBackend()
        backend.restore = lambda hwnd: setattr(backend, 'pid', 456)
        with self.assertRaisesRegex(CaptureError, '发生变化'):
            focus_game_window('魔兽世界', _backend=backend)
        self.assertEqual(backend.calls, [])

    def test_rejected_activation_does_not_retry(self):
        backend = FakeFocusBackend(); backend.denied = True
        request = focus_game_window('魔兽世界', _backend=backend)
        for _ in range(5): self.assertFalse(request.poll())
        with self.assertRaisesRegex(CaptureError, '确认超时'):
            request.poll(now=request.deadline)
        self.assertEqual(backend.calls.count(('activate', 33)), 1)

    def test_accepted_request_still_requires_actual_foreground_and_same_identity(self):
        for mutation in ('foreground', 'pid'):
            backend = FakeFocusBackend()
            def activate(hwnd):
                if mutation == 'pid': backend.pid = 456; backend.active = hwnd
                return True
            backend.activate = activate
            with self.subTest(mutation=mutation):
                if mutation == 'pid':
                    with self.assertRaises(CaptureError): focus_game_window('魔兽世界', _backend=backend)
                else:
                    request = focus_game_window('魔兽世界', _backend=backend)
                    self.assertFalse(request.poll())
                    with self.assertRaises(CaptureError): request.poll(now=request.deadline)

    def test_false_api_return_with_actual_foreground_is_success(self):
        backend = FakeFocusBackend()
        backend.activate = lambda hwnd: setattr(backend, 'active', hwnd) or False
        self.assertTrue(focus_game_window('魔兽世界', _backend=backend).poll())

    def test_delayed_foreground_is_confirmed_without_another_activation(self):
        backend = FakeFocusBackend(); backend.denied = True
        request = focus_game_window('魔兽世界', _backend=backend)
        self.assertFalse(request.poll())
        backend.active = 33
        self.assertTrue(request.poll())
        self.assertEqual(backend.calls, [('restore', 33), ('activate', 33)])

    def test_deadline_or_changed_identity_cannot_authorize_late_foreground(self):
        for change in ('deadline', 'closed', 'pid', 'title'):
            backend = FakeFocusBackend(); backend.denied = True
            request = focus_game_window('魔兽世界', _backend=backend)
            backend.active = 33
            if change == 'closed': backend.handles = []
            if change == 'pid': backend.pid = 456
            if change == 'title': backend.handles = [(33, 'Other app')]
            with self.subTest(change=change), self.assertRaises(CaptureError):
                request.poll(now=request.deadline if change == 'deadline' else request.deadline - .5)

    def test_native_restore_only_changes_minimized_windows_without_waiting(self):
        backend = WindowFocusBackend.__new__(WindowFocusBackend)
        backend.user = Mock()
        for minimized in (False, True):
            backend.user.reset_mock(); backend.user.IsIconic.return_value = minimized
            backend.restore(33)
            if minimized: backend.user.ShowWindowAsync.assert_called_once_with(33, 9)
            else: backend.user.ShowWindowAsync.assert_not_called()


class FocusButtonTests(unittest.TestCase):
    def app(self):
        app = Application.__new__(Application)
        app.settings = Settings(window_title='魔兽世界', calibration=CALIBRATION.copy(),
                                movement_calibration=MOVEMENT_CALIBRATION.copy())
        app._apply_settings = Mock(return_value=app.settings)
        app.map = SimpleNamespace(goal=Point(0, 30, 40))
        app.state = 'paused'; app.status_var = Mock(); app.log = Mock(); app.cancel_preview = Mock()
        app.closed = False; app.root = Mock(); app._focus_pending = None
        app.candidates_var = Mock(get=Mock(return_value=False))
        app.runtime = Mock(); app.runtime.command.return_value = 5
        app.runtime.control_epoch = 5; app.runtime.quit.is_set.return_value = False
        return app

    def test_buttons_revoke_old_input_before_focus_then_dispatch(self):
        for method, command, state in (('start', 'arm', 'armed'), ('calibrate', 'calibrate', 'calibration_armed')):
            app = self.app(); calls = []
            app.runtime.command.side_effect = lambda kind, **values: calls.append(kind) or 5
            with self.subTest(method=method), patch('adrive.app.focus_game_window', side_effect=lambda title: calls.append('focus') or Mock(poll=Mock(return_value=True))) as focus:
                getattr(app, method)()
            focus.assert_called_once_with('魔兽世界')
            self.assertEqual(calls, ['pause', 'focus', command])
            self.assertEqual(app.state, state)

    def test_focus_failure_leaves_only_pause_and_keeps_calibration(self):
        for method in ('start', 'calibrate'):
            app = self.app()
            app.state = 'armed'  # An older pending intent must also be revoked.
            with self.subTest(method=method), patch('adrive.app.focus_game_window', side_effect=CaptureError('fixture focus denied')):
                getattr(app, method)()
            app.runtime.command.assert_called_once_with('pause')
            app.status_var.set.assert_called_with('fixture focus denied')
            self.assertEqual(app.state, 'paused')
            self.assertIsNotNone(app.settings.movement_calibration)

    def test_invalid_preflight_and_offline_buttons_do_not_change_focus(self):
        for mode in ('goal', 'calibration', 'offline_start', 'offline_calibrate', 'invalid_settings'):
            app = self.app()
            if mode == 'goal': app.map.goal = None
            if mode == 'calibration': app.settings.movement_calibration = None
            if mode.startswith('offline'): app.runtime = None
            if mode == 'invalid_settings': app._apply_settings.side_effect = ValueError('fixture invalid')
            with self.subTest(mode=mode), patch('adrive.app.focus_game_window') as focus:
                (app.calibrate if mode == 'offline_calibrate' else app.start)()
            focus.assert_not_called()
            if app.runtime: app.runtime.command.assert_not_called()

    def test_new_stop_during_activation_prevents_ui_rearming(self):
        for method in ('start', 'calibrate'):
            app = self.app()
            app.runtime.command.side_effect = [5, None]  # epoch was superseded
            with self.subTest(method=method), patch('adrive.app.focus_game_window'):
                getattr(app, method)()
            self.assertEqual(app.state, 'paused')
            self.assertIn('已取消', app.status_var.set.call_args.args[0])

    def test_runtime_rejects_late_focus_callback_after_stop_or_configure(self):
        for kind in ('stop', 'configure'):
            runtime = Runtime.__new__(Runtime)
            runtime.dispatch_lock = threading.Lock(); runtime.stop_requested = threading.Event()
            runtime.commands = queue.Queue(); runtime.control_epoch = 0; runtime._waiter = None
            epoch = runtime.command('pause')
            runtime.command(kind)
            self.assertIsNone(runtime.command('arm', _expected_epoch=epoch))
            self.assertTrue(runtime.stop_requested.is_set())
            self.assertEqual([row[0] for row in runtime.commands.queue], ['pause', kind])

    def pending_app(self, method='start'):
        app = self.app()
        request = Mock(); request.poll.return_value = False
        with patch('adrive.app.focus_game_window', return_value=request): getattr(app, method)()
        app.runtime.command.assert_called_once_with('pause')
        self.assertEqual(app.state, 'focusing')
        return app, request, app._focus_pending

    def test_later_confirmation_dispatches_immediately_without_fixed_wait(self):
        for method, command in (('start', 'arm'), ('calibrate', 'calibrate')):
            app, request, pending = self.pending_app(method)
            app.root.after.assert_called_once_with(20, app._poll_focus_command, pending)
            request.poll.return_value = True
            app._poll_focus_command(pending)
            self.assertEqual([c.args[0] for c in app.runtime.command.call_args_list], ['pause', command])
            app.root.after.assert_called_once()  # No extra post-success wait.
            self.assertIsNone(app._focus_pending)
            app.log.assert_not_called()

    def test_pending_stop_configure_close_or_runtime_change_cancels_before_poll(self):
        for change in ('epoch', 'closed', 'quit', 'runtime'):
            app, request, pending = self.pending_app()
            old_runtime = app.runtime; request.poll.reset_mock()
            if change == 'epoch': app.runtime.control_epoch += 1
            if change == 'closed': app.closed = True
            if change == 'quit': app.runtime.quit.is_set.return_value = True
            if change == 'runtime': app.runtime = Mock()
            with self.subTest(change=change): app._poll_focus_command(pending)
            request.poll.assert_not_called()
            old_runtime.command.assert_called_once_with('pause')
            self.assertIsNone(app._focus_pending)

    def test_async_timeout_or_identity_error_leaves_only_pause(self):
        app, request, pending = self.pending_app()
        request.poll.side_effect = CaptureError('fixture confirmation timeout')
        app._poll_focus_command(pending)
        app.runtime.command.assert_called_once_with('pause')
        self.assertEqual(app.state, 'paused')
        self.assertIsNone(app._focus_pending)
        app.log.assert_called_once_with('fixture confirmation timeout')

    def test_new_invalid_click_cancels_old_pending_before_preflight(self):
        app, request, pending = self.pending_app()
        app.map.goal = None
        app.start()
        app._poll_focus_command(pending)
        self.assertIsNone(app._focus_pending)
        self.assertTrue(all(c.args[0] == 'pause' for c in app.runtime.command.call_args_list))

    def test_old_callback_cannot_dispatch_or_clear_new_request(self):
        app, request, pending = self.pending_app()
        next_request = Mock(poll=Mock(return_value=False))
        with patch('adrive.app.focus_game_window', return_value=next_request): app.calibrate()
        replacement = app._focus_pending
        app._poll_focus_command(pending)
        self.assertIs(app._focus_pending, replacement)
        self.assertEqual(app.state, 'focusing')
        self.assertTrue(all(c.args[0] == 'pause' for c in app.runtime.command.call_args_list))

    def test_f8_during_focus_cancels_instead_of_resuming_an_old_goal(self):
        app, request, pending = self.pending_app()
        app.runtime.command.reset_mock()
        app.send('toggle')
        app.runtime.command.assert_called_once_with('pause')

    def test_f8_during_native_activation_also_cancels_the_pending_click(self):
        app = self.app()
        def command(kind, **values):
            if kind == 'pause': app.runtime.control_epoch += 1
            return app.runtime.control_epoch
        app.runtime.command.side_effect = command
        def activate(title):
            app.send('toggle')
            return Mock(poll=Mock(return_value=True))
        with patch('adrive.app.focus_game_window', side_effect=activate): app.start()
        self.assertEqual([c.args[0] for c in app.runtime.command.call_args_list], ['pause', 'pause'])
        self.assertEqual(app.state, 'paused')
        self.assertIsNone(app._focus_pending)

    def test_runtime_paused_status_cannot_unlock_route_editing_during_confirmation(self):
        app, request, pending = self.pending_app()
        app.events = queue.Queue()
        for state in ('paused', 'stopped', 'fault'):
            app.events.put(dict(kind='status', state=state, reason='old queued status', capture='fixture'))
        app.poll()
        self.assertEqual(app.state, 'focusing')
        self.assertIs(app._focus_pending, pending)

    def test_close_revokes_control_before_hotkey_cleanup_and_discards_callback(self):
        app, request, pending = self.pending_app()
        app.hotkeys = Mock(); app.executor = Mock(); app.replay_job = None
        calls = []
        app.runtime.close.side_effect = lambda: calls.append('runtime')
        app.hotkeys.close.side_effect = lambda: (calls.append('hotkeys'), app.send('toggle'))
        app.runtime.command.reset_mock()
        app.close()
        app._poll_focus_command(pending)
        self.assertEqual(calls, ['runtime', 'hotkeys'])
        app.runtime.command.assert_not_called()
        self.assertIsNone(app._focus_pending)

    def test_f8_after_dispatch_cancels_queued_start_even_with_stale_paused_ui(self):
        for method in ('start', 'calibrate'):
            with self.subTest(method=method), tempfile.TemporaryDirectory() as directory:
                app = self.app()
                runtime = Runtime(None, app.settings, queue.Queue(), diagnostic_factory=None, log_folder=Path(directory))
                app.runtime = runtime
                try:
                    with patch('adrive.app.focus_game_window', return_value=Mock(poll=Mock(return_value=True))):
                        getattr(app, method)()
                    self.assertIsNone(app._focus_pending)
                    app.state = 'paused'  # A stale queued status must not change F8's intent.
                    app.send('toggle')
                    runtime._commands()
                    self.assertEqual(runtime.state, 'paused')
                    self.assertTrue(runtime.stop_requested.is_set())
                    self.assertIsNone(runtime._input_intent)
                finally:
                    runtime.pool.shutdown(); runtime.log.close()

    def test_f8_during_start_preparation_cannot_restore_cancelled_intent(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Runtime(None, self.app().settings, queue.Queue(), diagnostic_factory=None, log_folder=Path(directory))
            try:
                runtime.command('arm', goal=Point(0, 30, 40))
                original = runtime._calibration_required_reason
                def interrupt_preparation():
                    runtime.command('toggle')
                    return original()
                runtime._calibration_required_reason = interrupt_preparation
                runtime._commands()
                self.assertEqual(runtime.state, 'paused')
                self.assertTrue(runtime.stop_requested.is_set())
                self.assertIsNone(runtime._input_intent)
            finally:
                runtime.pool.shutdown(); runtime.log.close()
