"""Single-owner control loop; UI commands never directly send physical input.

Only an explicit arm/calibrate command can enter an input-producing state.
Planning is cancellable and separate from screen/stop monitoring.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, replace
import math
from pathlib import Path
import queue
import statistics
import threading
import time

from .capture import BufferedCapture
from .height import height_status
from .controller import Calibration, Controller
from .continuous_controller import ContinuousController
from .device_diagnostics import FirmwareDiagnosticsClient, describe_diagnostic
from .eventlog import EventLog
from .lease import LeaseClient
from .motion_protocol import MotionClient
from .motion import MotionProfile
from .motion_worker import MotionWorker
from .models import Flags, Point, Route, Telemetry, angle_error
from .pico import ActionGate, PicoClient, PicoError, forward_sequence
from .planner import Planner
from .planning_worker import IsolatedPlanner
from .settings import ROOT, Settings, key_code, calibration_required_reason
from .scheduling import LoopWaiter
from .telemetry import FreshnessTracker


ACTIVE = {'armed', 'planning', 'running', 'settling', 'calibration_armed', 'calibrating'}
UNSAFE = Flags.COMBAT | Flags.DEAD | Flags.SWIMMING | Flags.FLYING | Flags.LOADING | \
    Flags.INPUT_FOCUS | Flags.TAXI | Flags.TRANSPORT


def unsafe_reason(t: Telemetry | None) -> str | None:
    if t is None or not t.flags & Flags.VALID:
        return '没有有效的角色位置'
    for flag, reason in ((Flags.COMBAT, '进入战斗'), (Flags.DEAD, '角色死亡'),
                         (Flags.SWIMMING, '进入水中'), (Flags.FLYING, '正在飞行'),
                         (Flags.LOADING, '正在加载'), (Flags.INPUT_FOCUS, '聊天或输入框获得焦点'),
                         (Flags.TAXI, '正在乘坐飞行交通'), (Flags.TRANSPORT, '正在使用交通工具')):
        if t.flags & flag:
            return reason
    if t.environment&8:return '角色离开地面；本轮不支持跳跃'
    return None


class Runtime:
    def __init__(self, bundle, settings: Settings, events: queue.Queue,
                 *, capture_factory=BufferedCapture, pico_factory=PicoClient,
                 lease_factory=MotionClient, diagnostic_factory=FirmwareDiagnosticsClient,
                 planner_factory=Planner,
                 log_folder: Path | None = None):
        self.bundle, self.settings, self.events = bundle, settings, events
        self.capture_factory, self.pico_factory = capture_factory, pico_factory
        # None is an explicit legacy adapter for offline fixtures only. The UI
        # always uses v4 motion windows; unsupported firmware never falls back.
        self.lease_factory = lease_factory
        self.diagnostic_factory = diagnostic_factory
        self.planner_factory = IsolatedPlanner if planner_factory is Planner else planner_factory
        self.commands: queue.Queue = queue.Queue()
        self.dispatch_lock = threading.Lock()
        self.stop_requested = threading.Event()
        self.control_epoch = 0
        self._input_intent = None
        self.quit = threading.Event()
        self._waiter = None
        self.thread: threading.Thread | None = None
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='route')
        self.plan_cancel = threading.Event()
        self.future = None
        self.generation = 0
        self.capture = None
        self.pico = None
        self.lease = None
        self.diagnostic = None
        self.last_diagnostic = None
        self.firmware_info = None
        self._diagnostic_scheduled = None
        self._last_diagnostic_started = None
        self._lease_target = ()
        self._lease_frame = None
        self._lease_logged_at = None
        self._lease_application = None
        self.motion_worker = None
        self._motion_sent_frame = None
        self._motion_last_window = None
        self._motion_event_pending = None
        self._motion_walk_attempted = False
        self._motion_usb_pending=None
        self.movement_samples = []
        self.tracker = FreshnessTracker()
        self.controller = self._controller()
        self.state = 'monitoring'
        self.reason = '只监测；选择终点并校准后开始'
        self.goal: Point | None = None
        self.route = None
        self.pending = None
        self.gate = None
        self.last_health = None
        self.last_query = 0.0
        self.last_emit = 0.0
        self.last_loop_timing = {}
        self._last_timing_emit = 0.
        self.task_new = True
        self.calibration_samples = []
        self._calibration_candidate = None
        self._calibration_epoch = None
        self._calibration_baseline = None
        self._calibration_baseline_count = 0
        self.log = EventLog(log_folder or ROOT / 'runtime' / 'logs')
        self.log.write('session', settings={k: v for k, v in asdict(settings).items() if k != 'calibration'})

    def _controller(self):
        calibration = None
        if self.settings.calibration:
            try:
                calibration = Calibration(**self.settings.calibration)
            except (TypeError, ValueError):
                pass
        cls = ContinuousController if self.lease_factory is not None else Controller
        return cls(calibration=calibration, forward_key=key_code(self.settings.forward_key),
                   left_key=key_code(self.settings.left_key), right_key=key_code(self.settings.right_key))

    def start(self):
        self.thread = threading.Thread(target=self._run, name='navigation', daemon=True)
        self.thread.start()

    def command(self, kind: str, **values):
        expected_epoch = values.pop('_expected_epoch', None)
        with self.dispatch_lock:
            # A UI focus request must not arm after a newer F12/configure.
            if expected_epoch is not None and expected_epoch != self.control_epoch:
                return None
            if kind == 'toggle' and getattr(self, '_input_intent', None) is not None:
                kind = 'pause'
            if kind in ('stop', 'pause', 'toggle', 'configure'):
                self.control_epoch += 1
                self.stop_requested.set()
                self._input_intent = None
            epoch = self.control_epoch
            if kind in ('arm', 'calibrate'):
                self._input_intent = object()
                values['_input_intent'] = self._input_intent
            self.commands.put((kind, {**values, '_epoch': epoch}))
        if self._waiter:
            self._waiter.wake()
        return epoch

    def _resume_input(self, epoch: int) -> bool:
        with self.dispatch_lock:
            if epoch != self.control_epoch or self.quit.is_set():
                return False
            self.stop_requested.clear()
            return True

    def close(self):
        with self.dispatch_lock:
            self.stop_requested.set()
        self.quit.set()
        if self._waiter:
            self._waiter.wake()
        self.plan_cancel.set()
        if self.thread:
            self.thread.join(timeout=2)

    def _emit(self, kind: str, **values):
        self.events.put({'kind': kind, **values})

    def _status(self, state: str, reason: str):
        if (state, reason) != (self.state, self.reason):
            self.state, self.reason = state, reason
            self.log.write('state', state=state, reason=reason)
            self._emit('message', text=reason)

    def _release(self):
        # v3 STOP bypasses the action queue and invalidates the old session.
        # Do not add a global legacy release: a delayed cmd10 could cancel a
        # later, unrelated v3 session after this scoped STOP has completed.
        if self.lease:
            try:
                self.lease.stop(time.monotonic())
                self.log.write('action', action='lease_stop')
            except (OSError, PicoError) as exc:
                self._emit('message', text=f'租约停止请求未能送达：{exc}；设备仍须按租约到期释放')
        self._lease_target = ()
        self._lease_frame = None
        self._lease_application = None
        if self.motion_worker:
            self.motion_worker.close()
            self.motion_worker = None
        self._motion_last_window = self._motion_event_pending = None
        self._motion_usb_pending=None
        self._motion_sent_frame = None
        # One best-effort release; movement packets are never retransmitted.
        if self.pico and self.lease is None:
            try:
                self.pico.release_all()
                self.log.write('action', action='release_all')
            except (OSError, PicoError) as exc:
                self._emit('message', text=f'释放请求未能送达：{exc}；未重发移动指令')

    def _halt(self, reason: str, state='paused'):
        with self.dispatch_lock:
            self.stop_requested.set()
        was_active = self.state in ACTIVE or self.pending is not None
        self.generation += 1
        self.plan_cancel.set()
        self.future = None
        self.pending = self.gate = None
        self._calibration_candidate = None
        self._calibration_epoch = None
        self._calibration_baseline = None
        self._calibration_baseline_count = 0
        self.controller.pause(reason)
        if was_active:
            self._release()
        self._status(state, reason)
        if was_active and self.diagnostic and state in ('paused', 'fault', 'stopped', 'arrived'):
            self._schedule_diagnostic(reason, state)

    def _connect_objects(self):
        if self._waiter:
            self._waiter.set_socket(None)
        if self.capture:
            self.capture.close()
        if self.pico:
            self.pico.close()
        if self.lease:
            self.lease.close()
        if self.diagnostic:
            self.diagnostic.close()
        capture_options = {'window_title': self.settings.window_title}
        if self.capture_factory is BufferedCapture:
            capture_options['asset_root'] = self.bundle.asset_root
        self.capture = self.capture_factory(**capture_options)
        if self._waiter and hasattr(self.capture, 'set_wakeup'):
            self.capture.set_wakeup(self._waiter.wake)
        self.pico = self.pico_factory(host=self.settings.host, port=self.settings.port, timeout=.12)
        self.lease = self.lease_factory(host=self.settings.host, port=self.settings.port) if self.lease_factory else None
        if self._waiter and self.lease:
            self._waiter.set_socket(getattr(self.lease, 'wait_socket', None))
        self.diagnostic = None
        self.last_diagnostic = None
        self.firmware_info = None
        self._diagnostic_scheduled = None
        self._last_diagnostic_started = None
        self._emit('device_diagnostic_reset')
        if self.diagnostic_factory:
            try:
                self.diagnostic = self.diagnostic_factory(host=self.settings.host, port=self.settings.port)
            except (OSError, PicoError) as exc:
                self.log.write('diagnostic_error', message=str(exc), phase='initialize')
                self._emit('message', text=f'扩展诊断连接不可用：{exc}')
        self._lease_target = ()
        self._lease_frame = None
        self._lease_logged_at = None
        self._lease_application = None
        self.last_health = None
        self.tracker = FreshnessTracker()

    def _query(self):
        began = time.monotonic()
        try:
            health = self.pico.query()
        except (OSError, PicoError) as exc:
            self.log.write('pico_query', ok=False, elapsed_ms=(time.monotonic() - began) * 1000,
                           phase='observation' if self.pending else 'before_action', error=str(exc))
            raise
        self.last_query = time.monotonic()
        self.last_health = health
        self.log.write('pico_query', ok=True, elapsed_ms=(self.last_query - began) * 1000,
                       phase='observation' if self.pending else 'before_action', health=health)
        return health

    def _ready(self, health):
        return health.usb_mounted and health.keyboard_ready and not health.busy and health.queue_depth == 0

    def _commands(self):
        while True:
            try:
                kind, values = self.commands.get_nowait()
            except queue.Empty:
                return
            if kind == 'stop':
                self._halt('已停止；重新开始将从当前位置规划', 'stopped')
            elif kind == 'pause':
                self._halt('已暂停；按继续后重新规划')
            elif kind == 'toggle':
                if self.state in ACTIVE:
                    self._halt('快捷键暂停')
                elif self.goal is not None:
                    reason = self._calibration_required_reason()
                    if reason:
                        self._halt(reason)
                        continue
                    self._cancel_diagnostic()
                    if not self._resume_input(values['_epoch']):
                        continue
                    self.controller.reset_observation_session()
                    self.task_new = False
                    self._status('armed', '等待游戏前台与新遥测；将从最新位置重新规划')
            elif kind == 'configure':
                self._halt('配置已更新', 'monitoring')
                updated = values['settings']
                # The walk toggle was not used to measure yaw/strafe. Changing
                # it still stops and rebuilds input, but preserves that model.
                binding_names=('forward_key','left_key','right_key','strafe_left_key','strafe_right_key','window_title')
                if tuple(getattr(updated,n) for n in binding_names)!=tuple(getattr(self.settings,n) for n in binding_names):
                    updated.calibration = None
                    updated.movement_calibration=None
                self.settings = updated
                self.controller = self._controller()
                self._connect_objects()
            elif kind == 'diagnose':
                if self.diagnostic:
                    if self.state in ACTIVE:
                        self._emit('message', text='运行中不增加诊断报文；请先暂停后查看连接诊断')
                    elif self.diagnostic.active:
                        self._emit('message', text='只读诊断正在进行')
                    else:
                        self._diagnostic_scheduled = (time.monotonic(), {'trigger': 'manual'})
                        self._diagnostic_step(time.monotonic())
                    continue
                if self.lease:
                    if self.state in ACTIVE:
                        self._emit('message', text='正在控制中；连接状态持续记录在日志，单独诊断请先暂停')
                    else:
                        try:
                            self.lease.query_only(time.monotonic())
                            self._emit('message', text='正在只读查询 ADR v3；不会开启输入会话')
                        except (OSError, PicoError) as exc:
                            self._emit('message', text=f'Pico 查询失败：{exc}')
                    continue
                try:
                    health = self._query()
                    self._emit('health', health=health)
                    self._emit('message', text=f'Pico 已回应；USB={health.usb_mounted}，等待队列={health.queue_depth}，忙={health.busy}')
                except (OSError, PicoError) as exc:
                    self._emit('message', text=f'Pico 查询失败：{exc}')
            elif kind in ('calibrate', 'arm'):
                try:
                    self._prepare_input(kind, values)
                finally:
                    # Keep the intent visible to F8 until preparation has
                    # finished, including its temporary monitoring state.
                    with self.dispatch_lock:
                        if self._input_intent is values.get('_input_intent'):
                            self._input_intent = None

    def _prepare_input(self, kind, values):
        if kind == 'calibrate':
            self._halt('准备校准', 'monitoring')
            self._cancel_diagnostic()
            if not self._resume_input(values['_epoch']):
                return
            self.calibration_samples = []
            self.movement_samples = []
            self._calibration_epoch = values['_epoch']
            if isinstance(self.lease, MotionClient):
                self.settings.calibration = self.settings.movement_calibration = None
                self.controller = self._controller()
                self.settings.save()
                self._emit('calibration_reset')
            self._status('calibration_armed', '请保持游戏在前台并松开键鼠；将在空旷地面短转并测试平移，F12 停止')
        else:
            self._halt('准备新路线', 'monitoring')
            self._cancel_diagnostic()
            self.goal = values['goal']
            self.settings.allow_candidates = values.get('allow_candidates', False)
            self.task_new = True
            self._motion_walk_attempted=False
            reason = self._calibration_required_reason()
            if reason:
                self._status('paused', reason)
            elif self._resume_input(values['_epoch']):
                self._status('armed', '等待游戏前台与新遥测；将从最新位置重新规划')

    def _calibration_required_reason(self):
        if isinstance(self.lease, MotionClient):
            return calibration_required_reason(self.settings)
        return None if self.settings.calibration else '请先完成转向校准'

    def _request_plan(self, position: Point):
        observed = self.tracker.latest
        if observed and observed.height_source in ('ambiguous', 'unavailable') and position == observed.point:
            self._status('armed', height_status(observed))
            return
        self.plan_cancel.set()
        self.plan_cancel = threading.Event()
        generation, cancel = self.generation, self.plan_cancel
        blocked = frozenset() if self.task_new else frozenset(self.controller.blocked_edges)
        goal = self.goal
        candidates = self.settings.allow_candidates
        self.log.write('plan_request', start=position, goal=goal, allow_candidates=candidates,
                       start_height_source=getattr(self.tracker.latest, 'height_source', 'raw'),
                       blocked_edges=sorted(blocked))
        def work():
            return self.planner_factory(self.bundle).plan(position, goal,
                allow_candidates=candidates, blocked_edges=blocked,
                cancel=cancel, max_expansions=500000)
        self.future = self.pool.submit(work)
        self.future_generation = generation
        self._status('planning', '正在从实时位置规划；此时不发送移动指令')

    def _send(self, key: int, duration: int, t: Telemetry, *, calibration=False):
        if self.lease and self.lease.is_active:
            raise PicoError('连续按键会话运行中，不能插入旧协议动作')
        before = self._query()
        if not self._ready(before):
            raise PicoError('设备未空闲或键盘未准备好，已暂停')
        # Health query takes time. Recheck focus and freshness at actual dispatch.
        now = time.monotonic()
        if not self.capture.foreground() or not self.tracker.fresh(now):
            raise PicoError('发令前窗口或遥测已失效')
        if unsafe_reason(t):
            raise PicoError(unsafe_reason(t))
        self.gate = ActionGate()
        self.gate.begin(before, t, now, duration)
        self.pending = {'key': key, 'duration': duration, 'before': t, 'sent': now,
                        'calibration': calibration, 'first_change': None}
        with self.dispatch_lock:
            if self.stop_requested.is_set() or self.quit.is_set():
                self.pending = self.gate = None
                raise PicoError('发令前收到停止请求，未发送按键')
            self.pico.press(key, duration)
        self.last_query = now
        self.log.write('action', action='press', key=key, duration_ms=duration,
                       seq=t.seq, session=t.session, calibration=calibration)

    def _poll_lease(self, now):
        before_state = self.lease.state
        try:
            status = self.lease.poll(now)
        finally:
            self._drain_lease_events()
        status = status or self.lease.status
        if status is not None and self.firmware_info:
            detail = self.firmware_info
            issues = detail.compatibility_issues(protocol=4 if isinstance(self.lease,MotionClient) else 3)
            if detail.boot == status.boot and issues and self.lease.is_active:
                raise PicoError('固件与当前连续导航不兼容：' + '；'.join(issues))
        if before_state == 'diagnosing' and self.lease.state == 'idle' and status is not None:
            self._emit('message', text=f'ADRV v{getattr(self.lease,"protocol_version",3)} 已回应；设备可用={status.available}，'
                       f'会话活动={status.active}，剩余租约={status.lease_remaining_ms} ms')
        if status is not None and status.received_at != self._lease_logged_at:
            self._lease_logged_at = status.received_at
            self.log.write('pico_lease', status=status, state=self.lease.state)
        if self._lease_application and status is not None:
            sequence, sent_at, keys, duration = self._lease_application
            applied = status.applied_seq == sequence
            if applied:
                actual_matches = status.keys == keys and not status.modifier
                expired_matches = duration and status.hold_expired and not status.keys and not status.modifier
                if not status.pending:
                    if not actual_matches and not expired_matches:
                        raise PicoError('Pico 已提交的按键状态与目标不一致，已停止')
                    self._lease_application = None
            if self._lease_application and now - sent_at >= .18:
                # A renewal ACK does not prove that an earlier SET reached USB.
                # In particular, a lost timed SET must not keep a previous W
                # state alive while waiting a full motion-observation timeout.
                raise PicoError('Pico 在 180 ms 内未确认目标按键已提交，已停止')
        return status

    def _drain_lease_events(self):
        if self.lease:
            for event in self.lease.drain_events():
                values = dict(event)
                kind = values.pop('kind')
                self.log.write(kind, **values)

    def _cancel_diagnostic(self):
        self._diagnostic_scheduled = None
        if self.diagnostic:
            self.diagnostic.cancel()

    def _schedule_diagnostic(self, reason, state):
        now = time.monotonic()
        status = self.lease.status if self.lease else None
        context = {'trigger': 'after_stop', 'client_state': state, 'client_reason': reason,
                   'client_stopped_at': now, 'boot': getattr(status, 'boot', None),
                   'epoch': getattr(status, 'epoch', None), 'session': getattr(status, 'session', None),
                   'sent_seq': self.lease.seq if self.lease else None}
        due = now + .35  # Let STOP/release complete before taking one snapshot.
        if self._last_diagnostic_started is not None:
            due = max(due, self._last_diagnostic_started + 5.)
        self._diagnostic_scheduled = (due, context)

    def _diagnostic_step(self, now):
        if not self.diagnostic:
            return
        if self.state in ACTIVE:
            self._cancel_diagnostic()
            return
        try:
            if self._diagnostic_scheduled and not self.diagnostic.active:
                due, context = self._diagnostic_scheduled
                if now >= due:
                    self._diagnostic_scheduled = None
                    self._last_diagnostic_started = now
                    self.diagnostic.begin(now, context=context)
                    if context.get('trigger') == 'manual':
                        self._emit('message', text='正在读取固件版本、能力与停止记录；不会开启输入会话')
            result = self.diagnostic.poll(now)
        except (OSError, PicoError) as exc:
            # Diagnostic failures must not replace the original motion fault,
            # reopen a session or generate another stop/diagnostic cycle.
            self.diagnostic.cancel()
            self._diagnostic_scheduled = None
            self.log.write('diagnostic_error', message=str(exc))
            self._emit('message', text=f'只读诊断未完成：{exc}')
            return
        if result is None:
            return
        self.last_diagnostic = result
        if result.diagnostic:
            self.firmware_info = result.diagnostic
        kind = 'diagnostic_error' if result.kind == 'error' else 'firmware_diagnostic'
        association = None
        if result.diagnostic and result.context and result.context.get('trigger') == 'after_stop':
            detail, context = result.diagnostic, result.context
            association = (detail.stop.valid and detail.boot == context.get('boot') and
                           detail.stop.epoch == context.get('epoch') and detail.stop.session == context.get('session'))
        self.log.write(kind, result=result, matches_stopped_session=association)
        summary, details = describe_diagnostic(result)
        self._emit('device_diagnostic', summary=summary, details=details)

    def _lease_send(self, t, now, *, key=None, keys=None, duration=0, renew=False):
        # Window title/identity calls can stall. Keep them outside the dispatch
        # lock so another thread can latch STOP while this check is in flight.
        # BufferedCapture.foreground also rechecks GetForegroundWindow after
        # identity inspection; this boolean is followed immediately by the
        # short locked authorization and nonblocking socket send below.
        foreground = self.capture.foreground()
        with self.dispatch_lock:
            if self.stop_requested.is_set() or self.quit.is_set():
                raise PicoError('发令前收到停止请求，未发送按键')
            now = time.monotonic()
            if not foreground or not self.tracker.fresh(now) or unsafe_reason(t):
                raise PicoError('发令前窗口或遥测已失效')
            if renew:
                self.lease.renew(now)
            else:
                keys = tuple(keys) if keys is not None else ((key,) if key is not None else ())
                self.lease.set_keys(keys, now, hold_ms=duration)
                self._lease_target = keys
                self._lease_application = (self.lease.seq, now, keys, duration)
        self._lease_frame = (t.session, t.seq)
        self.log.write('action', action='lease_renew' if renew else 'lease_keys',
                       key=key, duration_ms=duration, lease_seq=self.lease.seq,
                       seq=t.seq, session=t.session)
        return now

    def _lease_renew(self, t, now):
        if (t.session, t.seq) == self._lease_frame or now - self.lease.last_sent_at < .05:
            return
        self._lease_send(t, now, renew=True)

    @staticmethod
    def _calibration_pose_stable(before, after):
        # Only calibration waits for a measured plateau. USB completion alone
        # does not mean that the game has processed the release. Position is
        # quantized to 1/16 yd, so also require the live speed to be zero.
        return (before.session == after.session and before.c == after.c
                and before.speed <= .05 and after.speed <= .05
                and before.point.distance(after.point) < .04
                and abs(angle_error(after.facing, before.facing)) < math.radians(.2))

    def _calibration_settled(self, action, t):
        previous = action.get('settle_frame')
        if previous and (previous.session, previous.seq) == (t.session, t.seq):
            return False
        stable = previous is not None and self._calibration_pose_stable(previous, t)
        action['settle_count'] = action.get('settle_count', 0) + 1 if stable else 0
        action['settle_frame'] = t
        if not stable:
            action['settle_start_seq'] = t.seq
        # Two stable intervals (three distinct frames) after the zero report,
        # and a real observed effect, prevent delayed input or duplicate reads
        # from certifying a premature endpoint. The existing timeout bounds it.
        return action['first_change'] is not None and action['settle_count'] >= 2

    def _lease_pending_step(self, t, now, status):
        action = self.pending
        if action.get('calibration') and action['first_change'] is None and \
                (abs(angle_error(t.facing, action['before'].facing)) > math.radians(.2) or
                 (action.get('translation') and t.point.distance(action['before'].point)>.07)):
            action['first_change'] = now
        earliest = action['sent'] + action['duration'] / 1000 + .005
        if now > action['sent'] + action['duration'] / 1000 + 1:
            raise PicoError('定时按键后的观察超时；未重发动作')
        sequence = action['lease_seq']
        applied = status is not None and status.applied_seq == sequence
        released = applied and status.hold_expired and not status.pending and not status.keys and not status.modifier
        if released and action.get('release_observed_at') is None:
            action['release_observed_at'] = now
            action['release_frame'] = (t.session, t.seq)
        # Device timing starts at USB acceptance, which may be later than send.
        # Require a newly captured frame after we observed the zero report.
        complete = released and action.get('release_observed_at') is not None
        complete = complete and t.captured_at >= max(earliest, action['release_observed_at'])
        complete = complete and (t.session, t.seq) != action['release_frame']
        if complete and action.get('calibration') and isinstance(self.lease, MotionClient):
            complete = self._calibration_settled(action, t)
        if complete:
            if action.get('calibration'):
                self._finish_calibration_sample(t, now)
                self.log.write('calibration_sample', action_seq=action['before'].seq,
                    seq=t.seq, keys=action.get('keys', (action.get('key'),)),
                    duration_ms=action['duration'], first_change=action['first_change'],
                    release_observed_at=action['release_observed_at'],
                    settle_start_seq=action.get('settle_start_seq'), stable_intervals=action.get('settle_count'),
                    displacement=t.point.distance(action['before'].point),
                    angle=angle_error(t.facing, action['before'].facing))
            elif not self.controller.on_action_complete(t, now):
                raise PicoError('控制器未接受定时按键后的观察帧')
            self.log.write('observed', seq=t.seq, action_seq=action['before'].seq,
                           lease_seq=sequence, displacement=t.point.distance(action['before'].point),
                           angle=angle_error(t.facing, action['before'].facing))
            self.pending = None
            self._lease_target = ()
            if action.get('calibration') and self.state == 'monitoring':
                self._release()
                return
        self._lease_renew(t, now)

    def _lease_calibrating_step(self, t, now):
        if self.stop_requested.is_set():
            return
        status = self._poll_lease(now)
        if self.stop_requested.is_set() or self.quit.is_set():
            return
        if not self.lease.is_active:
            return
        if self.pending:
            self._lease_pending_step(t, now, status)
            return
        if isinstance(self.lease, MotionClient) and not self.calibration_samples:
            previous = self._calibration_baseline
            if previous is None or (previous.session, previous.seq) != (t.session, t.seq):
                stable = previous is not None and self._calibration_pose_stable(previous, t)
                self._calibration_baseline_count = self._calibration_baseline_count + 1 if stable else 0
                self._calibration_baseline = t
            if self._calibration_baseline_count < 2:
                self._status('calibrating', '正在确认角色静止；请松开游戏内的移动、转向按键和鼠标')
                self._lease_renew(t, now)
                return
        if isinstance(self.lease,MotionClient) and len(self.calibration_samples)>=4:
            return self._translation_calibrating_step(t,now)
        key = key_code(self.settings.left_key if len(self.calibration_samples) % 2 == 0 else self.settings.right_key)
        self._status('calibrating', f'正在测量转向 {len(self.calibration_samples) + 1}/4；F12 可停止')
        sent_at = self._lease_send(t, now, key=key, duration=100)
        self.pending = {'key': key, 'duration': 100, 'before': t, 'sent': sent_at,
                        'lease_seq': self.lease.seq, 'calibration': True, 'first_change': None}

    def _lease_running_step(self, t, now):
        if isinstance(self.lease, MotionClient):
            return self._motion_running_step(t, now)
        if self.stop_requested.is_set():
            return
        status = self._poll_lease(now)
        if not self.lease.is_active:
            return  # Capability discovery and session opening never press keys.
        self._status('running', '正在连续行走；F8 暂停，F12 停止')
        if self.pending:
            self._lease_pending_step(t, now, status)
            return
        decision = self.controller.update(t, now)
        if decision.kind == 'keys':
            target = (decision.key,)
            if decision.duration_ms == 0 and target == self._lease_target:
                self._lease_renew(t, now)
            else:
                sent_at = self._lease_send(t, now, key=decision.key, duration=decision.duration_ms)
                if decision.duration_ms:
                    self.pending = {'key': decision.key, 'duration': decision.duration_ms,
                                    'before': t, 'sent': sent_at, 'lease_seq': self.lease.seq}
        elif decision.kind == 'none' and self._lease_target:
            # A new safe frame can arrive before the 50 ms renewal interval.
            # Consume it once at the due time, rather than waiting for yet
            # another screenshot. _lease_frame prevents repeated authorization.
            self._lease_renew(t, now)
        elif decision.kind == 'replan':
            self._release()
            self.log.write('replan', reason=decision.reason, blocked_edge=decision.blocked_edge)
            self._request_plan(t.point)
        elif decision.kind == 'arrived':
            self._halt('已到达终点（2 码范围内）', 'arrived')
        elif decision.kind == 'stop':
            self._halt(decision.reason, 'fault')

    def _motion_bindings(self):
        return tuple(key_code(getattr(self.settings,n)) for n in
                     ('forward_key','left_key','right_key','strafe_left_key','strafe_right_key','walk_toggle_key'))

    def _motion_running_step(self,t,now):
        if self.stop_requested.is_set():return
        status=self._poll_lease(now)
        if not self.lease.is_active:return
        if self._motion_usb_pending:
            revision,sent=self._motion_usb_pending
            submitted=status.submitted_revision if status else 0
            if submitted==revision or forward_sequence(submitted,revision):self._motion_usb_pending=None
            elif now-sent>=.18:raise PicoError('Pico 在 180 ms 内未确认任何新窗口已提交 USB，已停止')
        if self.motion_worker is None:
            self._halt('连续运动进程未准备好','fault');return
        result=self.motion_worker.poll()
        if result:
            window,elapsed=result;source=window.source
            age=time.monotonic()-source.captured_at
            same=source.session==t.session and window.route_version==self.generation
            speed_same=abs(source.speed-t.speed)<=max(.1,t.speed*.04) and source.ground_speed==t.ground_speed
            # A window built from an old pose cannot be sent just because the
            # newest screen frame is safe. Bound processing drift explicitly.
            dt=t.captured_at-source.captured_at
            pose_same=(source.c==t.c and source.z is not None and t.z is not None and dt>=0
                and source.point.distance(t.point)<=max(source.speed,t.speed,source.ground_speed or 0)*dt+.10
                and abs(source.z-t.z)<=max(source.speed,t.speed)*dt+.15)
            usable=same and speed_same and pose_same and 0<=age<=.08 and abs(angle_error(source.facing,t.facing))<=.20
            if window.kind!='window' or not usable:
                self.log.write('motion_decision',decision_kind=window.kind,reason=window.reason,reason_code=window.reason_code,
                    source=source,route_version=window.route_version,model_version=window.model_version,
                    source_age_ms=age*1000,calculation_ms=elapsed,usable=usable,
                    blocked_edge=window.blocked_edge,navigation=self.motion_worker.identity)
            if usable and window.kind in ('stop','replan','arrived','settle'):
                if window.kind=='arrived':self._halt(window.reason,'arrived')
                elif window.kind=='settle':
                    self._release();self._settling_frame=(t.session,t.seq);self._settling_at=now
                    self._status('settling',window.reason)
                elif window.kind=='replan':
                    if window.blocked_edge:self.controller.blocked_edges.add(window.blocked_edge)
                    self.log.write('replan',reason=window.reason,reason_code=window.reason_code,
                        blocked_edge=window.blocked_edge,blocked_edges=sorted(self.controller.blocked_edges))
                    decision=self.controller._replan(window.reason)
                    if decision.kind=='stop':self._halt(decision.reason,'fault')
                    else:self._release();self._request_plan(t.point)
                else:self._halt(window.reason,'fault')
                return
            if usable and window.kind=='window' and window.frames:
                foreground=self.capture.foreground()
                with self.dispatch_lock:
                    now=time.monotonic()
                    if self.stop_requested.is_set() or self.quit.is_set():return
                    if not foreground or not self.tracker.fresh(now) or unsafe_reason(t) or now-source.captured_at>.08:
                        raise PicoError('运动窗口发令时已失效')
                    self.lease.set_window(window.frames,now,event_id=window.event_id,
                                          remaining_ms=max(20,min(200,round(200-age*1000))))
                    if self._motion_usb_pending is None:self._motion_usb_pending=(self.lease.revision,now)
                    self._lease_target=window.frames[0].keys
                    self._lease_frame=(t.session,t.seq)
                    self._motion_sent_frame=(source.session,source.seq)
                    self._motion_last_window=window
                    if window.event_id:
                        self._motion_event_pending=window.event_id;self._motion_walk_attempted=True
                self.log.write('motion_window',window=window,firmware_revision=self.lease.revision,
                               firmware_identity=dict(boot=status.boot,epoch=status.epoch,session=status.session),
                               calculation_ms=elapsed,source_age_ms=age*1000,navigation=self.motion_worker.identity)
                if window.trajectory:self._emit('motion_trajectory',points=window.trajectory)
                if window.reason:self._status('running',window.reason)
        # Do not replace a one-shot toggle before USB acceptance is observed.
        if self._motion_event_pending:
            if status and status.applied_event==self._motion_event_pending:self._motion_event_pending=None
        if not self._motion_event_pending:
            self.motion_worker.submit(t,status.keys if status else (),status.applied_event if status else 0)
        self._lease_renew(t,time.monotonic())

    def _finish_calibration_sample(self, t, now):
        action = self.pending
        if action.get('translation'):
            return self._finish_translation_sample(t,now)
        delta = angle_error(t.facing, action['before'].facing)
        left = action['key'] == key_code(self.settings.left_key)
        if abs(delta) < math.radians(1) or (delta > 0) != left:
            raise PicoError(f'校准未观察到正确转向；检查 {self.settings.left_key}/{self.settings.right_key} 是否绑定为转向、游戏焦点及鼠标右键状态')
        if t.point.distance(action['before'].point) > .12:
            raise PicoError('转向校准时角色发生了位移；请检查转向绑定并松开其他按键')
        rate = abs(delta) / (action['duration'] / 1000)
        if not .1 < rate < 12:
            raise PicoError('转向速率异常，请检查按键绑定后重新校准')
        latency = max(0.0, (action['first_change'] or now) - action['sent'])
        if latency>.25:raise PicoError('实测输入生效延迟超过 250 ms，拒绝保存低估的延迟模型')
        self.calibration_samples.append((left, rate, latency))
        if len(self.calibration_samples) == 4:
            for direction in (True, False):
                rates = [sample[1] for sample in self.calibration_samples if sample[0] == direction]
                if max(rates) / min(rates) > 1.25:
                    raise PicoError('重复校准结果相差超过 25%，请保持鼠标键盘不动后重试')
            left_rate = statistics.mean(v[1] for v in self.calibration_samples if v[0])
            right_rate = statistics.mean(v[1] for v in self.calibration_samples if not v[0])
            calibration = Calibration(left_rate, right_rate,
                                      statistics.median(v[2] for v in self.calibration_samples))
            if isinstance(self.lease,MotionClient):
                # A yaw-only result must never make the UI/runtime look ready.
                # Publish both models together after all translation samples.
                self._calibration_candidate = calibration
                self._status('calibrating','转速测量完成，正在验证平移测试区域')
                return
            self.controller.set_calibration(calibration)
            self.settings.calibration = asdict(calibration)
            self.settings.save()
            self.log.write('calibration', calibration=calibration)
            self._emit('calibrated', calibration=self.settings.calibration)
            self._status('monitoring', f'校准完成：左转 {math.degrees(left_rate):.1f}°/秒，右转 {math.degrees(right_rate):.1f}°/秒')

    def _translation_calibrating_step(self,t,now):
        if t.protocol_version!=4 or t.z is None or not t.ground_speed or t.environment&8:
            raise PicoError('平移校准需要 v4 速度遥测和已确定的地面')
        if self._calibration_candidate is None:
            raise PicoError('转向候选校准已失效，请重新校准')
        profile=MotionProfile.from_calibration(self._calibration_candidate)
        radius=t.ground_speed*(.12+profile.latency+.08)+.2
        if radius>4:raise PicoError('当前速度过高，无法认证平移校准区域，请减速后重试')
        if self.motion_worker is None:
            self.motion_worker=MotionWorker(Route(t.point,t.point,()),profile,self._motion_bindings(),
                self.bundle.asset_root/'navigation/manifest.json',self.generation)
        result=self.motion_worker.poll()
        if result and result[0]=='probe':
            _,source,clearance,valid=result
            if not valid:raise PicoError('当前位置周围净空不足，请换到空旷地面校准')
            if source.session==t.session and source.point.distance(t.point)<.07 and now-source.captured_at<.15 and clearance>=radius-.01:
                forward,_,_,left,right,_=self._motion_bindings()
                keys=((left,),(right,),(forward,left),(forward,right))[len(self.movement_samples)%4]
                self._status('calibrating', f'正在测量平移与斜向前进 {len(self.movement_samples) + 1}/8；F12 可停止')
                sent=self._lease_send(t,now,keys=keys,duration=120)
                self.pending=dict(keys=keys,duration=120,before=t,sent=sent,lease_seq=self.lease.seq,
                    calibration=True,translation=True,first_change=None)
                return
        self.motion_worker.probe(t,radius)
        self._lease_renew(t,now)

    def _finish_translation_sample(self,t,now):
        action=self.pending;before=action['before'];distance=t.point.distance(before.point)
        calibration = self._calibration_candidate
        if calibration is None:
            raise PicoError('转向候选校准已失效，请重新校准')
        if not .12<distance<before.ground_speed*(.12+calibration.input_latency_s+.08)+.2:
            raise PicoError('平移校准位移异常，请检查按键绑定及地面阻挡')
        if abs(angle_error(t.facing,before.facing))>math.radians(3):
            raise PicoError(f'平移改变了朝向，请将 {self.settings.strafe_left_key}/{self.settings.strafe_right_key} 绑定为平移')
        direction=angle_error(math.atan2(t.y-before.y,t.x-before.x),before.facing)
        expected=(math.pi/2,-math.pi/2,math.pi/4,-math.pi/4)[len(self.movement_samples)%4]
        if abs(angle_error(direction,expected))>math.radians(22):
            raise PicoError('实测平移方向与绑定不符，请检查按键')
        self.movement_samples.append(direction)
        if len(self.movement_samples)==8:
            if any(abs(angle_error(self.movement_samples[i],self.movement_samples[i+4]))>math.radians(12) for i in range(4)):
                raise PicoError('重复平移校准差异过大，请在空旷地面重试')
            names=('strafe_left_angle','strafe_right_angle','diagonal_left_angle','diagonal_right_angle')
            movement={n:statistics.mean((self.movement_samples[i],self.movement_samples[i+4])) for i,n in enumerate(names)}
            movement['strafe_verified']=True
            updated = replace(self.settings, calibration=asdict(calibration), movement_calibration=movement)
            # This acceptance point is ordered with command(stop/configure).
            # A stop already received discards the candidate; a later stop
            # cannot retroactively undo a completed measurement. Disk I/O is
            # outside the dispatch lock, so it cannot block an urgent stop.
            with self.dispatch_lock:
                if (self.stop_requested.is_set() or self.quit.is_set()
                        or self._calibration_epoch != self.control_epoch):
                    self._calibration_candidate = None
                    return
                self._calibration_candidate = None
            updated.save()
            self.settings.calibration = updated.calibration
            self.settings.movement_calibration = updated.movement_calibration
            self.controller.set_calibration(calibration)
            self.log.write('calibration',calibration=self.settings.calibration,movement_calibration=self.settings.movement_calibration)
            self._emit('calibrated',calibration=self.settings.calibration,movement_calibration=self.settings.movement_calibration)
            self._status('monitoring','转速、输入延迟、纯平移和斜向前进校准完成')

    def _pending_step(self, t: Telemetry, now: float):
        action = self.pending
        if action['first_change'] is None and abs(angle_error(t.facing, action['before'].facing)) > math.radians(.2):
            action['first_change'] = now
        if now - self.last_query < .035:
            return
        result = self.gate.observe(self._query(), t, time.monotonic())
        if result == 'complete':
            if action['calibration']:
                self._finish_calibration_sample(t, now)
            elif not self.controller.on_action_complete(t, now):
                raise PicoError('控制器未接受动作后的观察帧')
            self.log.write('observed', seq=t.seq, action_seq=action['before'].seq,
                           displacement=t.point.distance(action['before'].point),
                           angle=angle_error(t.facing, action['before'].facing))
            self.pending = self.gate = None

    def _active_step(self, t: Telemetry | None, now: float):
        if self.state not in ACTIVE:
            return
        if self.state in ('armed', 'calibration_armed'):
            foreground = self.capture.foreground()
            now = time.monotonic()
            if not foreground or not self.tracker.fresh(now):
                return
            reason = unsafe_reason(t)
            if reason:
                self._halt(reason + '，已暂停；恢复后请手动继续')
                return
            if isinstance(self.lease,MotionClient) and t.protocol_version!=4:
                self._halt('请更新 v4 速度遥测插件并重新加载');return
            if (self.state=='calibration_armed' and isinstance(self.lease,MotionClient)
                    and (t.z is None or not t.ground_speed)):
                self._halt('校准需要插件有效地面速度能力和已确定的楼层');return
            if self.state == 'armed':
                reason = self._calibration_required_reason()
                if reason:
                    self._halt(reason)
                else:
                    self._request_plan(t.point)
            else:
                self._status('calibrating', '校准进行中；F12 可停止')
                if self.lease:
                    with self.dispatch_lock:
                        if self.stop_requested.is_set() or self.quit.is_set():
                            return
                        self.lease.begin(now)
                    self._lease_target = ()
                    self._lease_frame = None
                    self._lease_application = None
            return
        foreground = self.capture.foreground()
        now = time.monotonic()
        if not foreground:
            self._halt('游戏失去前台，已暂停')
            return
        if not self.tracker.fresh(now):
            self._halt('超过 250 ms 没有新有效帧，已暂停')
            return
        reason = unsafe_reason(t)
        if reason:
            self._halt(reason + '，已暂停')
            return
        if self.state=='settling':
            if (t.session,t.seq)==self._settling_frame:return
            if t.speed<=.05:
                goal=self.route.goal
                if t.point.distance(goal)<=.6 and t.z is not None and goal.z is not None and abs(t.z-goal.z)<=1:
                    self._halt('已停止并确认到达终点（0.6 码范围内）','arrived')
                else:self._halt('已停止，但终点位置误差超过 0.6 码；请检查延迟模型后继续')
            elif now-self._settling_at>.25+Calibration(**self.settings.calibration).input_latency_s:
                self._halt('释放后仍未确认停止，请检查输入延迟和游戏状态')
            return
        if self.state == 'running' and self.lease:
            self._lease_running_step(t, now)
            return
        if self.state == 'calibrating' and self.lease:
            self._lease_calibrating_step(t, now)
            return
        if self.pending:
            self._pending_step(t, now)
            return
        if self.state == 'planning':
            if self.future is not None and self.future.done():
                route = self.future.result()
                if self.future_generation != self.generation:
                    return
                self.future = None
                # No movement has been issued while planning. A displaced player
                # needs a new plan, never a guessed line back to the old start.
                if t.point.distance(route.start) > 2:
                    self._request_plan(t.point)
                    return
                if t.z is None and any(s.reliability == 'navmesh' for s in route.segments):
                    self._halt(height_status(t), 'fault')
                    return
                self.route = route
                self.controller.set_route(route, new_task=self.task_new)
                self.task_new = False
                self.log.write('route', route=route)
                self._emit('route', route=route)
                if isinstance(self.lease,MotionClient):
                    profile=MotionProfile.from_calibration(Calibration(**self.settings.calibration),self.settings.movement_calibration)
                    profile=replace(profile,walk_toggle_allowed=not self._motion_walk_attempted)
                    self.motion_worker=MotionWorker(route,profile,self._motion_bindings(),
                        self.bundle.asset_root/'navigation/manifest.json',self.generation)
                if self.lease:
                    with self.dispatch_lock:
                        if self.stop_requested.is_set() or self.quit.is_set():
                            return
                        self.lease.begin(now)
                    self._lease_target = ()
                    self._lease_frame = None
                    self._lease_application = None
                    self._status('running', '正在建立 ADRV v4 连续控制会话；固件和轨迹就绪后行走')
                else:
                    self._status('running', '正在沿路线行走；F8 暂停，F12 停止')
            return
        if self.state == 'calibrating':
            # L,R,L,R avoids accumulating a large change in orientation.
            key = key_code(self.settings.left_key if len(self.calibration_samples) % 2 == 0 else self.settings.right_key)
            self._send(key, 100, t, calibration=True)
            return
        if self.state == 'running':
            decision = self.controller.update(t, now)
            if decision.kind == 'press':
                self._send(decision.key, decision.duration_ms, t)
            elif decision.kind == 'replan':
                self._release()
                self.log.write('replan', reason=decision.reason, blocked_edge=decision.blocked_edge)
                self._request_plan(t.point)
            elif decision.kind == 'arrived':
                self._halt('已到达终点（2 码范围内）', 'arrived')
            elif decision.kind == 'stop':
                self._halt(decision.reason, 'fault')

    def _run(self):
        try:
            self._waiter = LoopWaiter()
            self._connect_objects()
            previous_loop = None
            while not self.quit.is_set():
                began = time.monotonic()
                gap_ms = (began - previous_loop) * 1000 if previous_loop is not None else 0.
                previous_loop = began
                phases = {}
                try:
                    self._loop_phase(phases, 'commands_ms', self._commands)
                    sample = self._loop_phase(phases, 'capture_poll_ms', self.capture.poll)
                    now = time.monotonic()
                    self._loop_phase(phases, 'telemetry_ms', self._accept_sample, sample, now)
                    self._loop_phase(phases, 'control_ms', self._active_step, self.tracker.latest, time.monotonic())
                    if self.lease and (self.state not in ACTIVE or self.state=='settling'):
                        # STOP retries are safe and session-scoped. They cannot
                        # renew keys or reopen the halted session.
                        self._loop_phase(phases, 'inactive_lease_ms', self._poll_lease, time.monotonic())
                    self._loop_phase(phases, 'lease_log_ms', self._drain_lease_events)
                    # Separate socket, inactive states only, one scheduled
                    # snapshot. This path never refreshes the control ACK clock.
                    if self.diagnostic and (self.diagnostic.active or self._diagnostic_scheduled):
                        self._loop_phase(phases, 'diagnostic_ms', self._diagnostic_step, time.monotonic())
                except Exception as exc:
                    self._halt(str(exc), 'fault')
                    self.log.write('error', type=type(exc).__name__, message=str(exc),
                                   loop_gap_ms=gap_ms, loop_elapsed_ms=(time.monotonic() - began) * 1000,
                                   phases_ms=phases, capture=self._capture_diagnostics(),
                                   previous_wait=dict(self._waiter.last_wait))
                finished = time.monotonic()
                self.last_loop_timing = {'gap_ms': gap_ms, 'work_ms': (finished - began) * 1000,
                                         'phases_ms': phases, 'capture': self._capture_diagnostics(),
                                         'previous_wait': dict(self._waiter.last_wait)}
                if gap_ms > 80 or finished - began > .04 or finished - self._last_timing_emit >= 1:
                    self.log.write('runtime_timing', **self.last_loop_timing)
                    self._last_timing_emit = finished
                if time.monotonic() - self.last_emit > .15:
                    self.last_emit = time.monotonic()
                    self._emit('status', state=self.state, reason=self.reason,
                               capture=self.capture.status if self.capture else '未初始化',
                               fresh=self.tracker.fresh(self.last_emit))
                if not self.quit.is_set():
                    now = time.monotonic()
                    self._waiter.wait(self._wait_seconds(now, began))
        except Exception as exc:
            if self.state in ACTIVE or self.pending:
                self._halt(f'控制循环异常：{exc}', 'fault')
            else:
                self._emit('message', text=f'监测初始化失败：{exc}')
        finally:
            if self._waiter:
                self._waiter.close()
            if self.state in ACTIVE or self.pending:
                self._release()
            self.plan_cancel.set()
            self.pool.shutdown(wait=False, cancel_futures=True)
            if self.capture:
                self.capture.close()
            if self.pico:
                self.pico.close()
            if self.lease:
                self.lease.close()
                self._drain_lease_events()
            if self.diagnostic:
                self.diagnostic.close()
            self.log.write('closed')
            self.log.close()

    @staticmethod
    def _loop_phase(phases, name, function, *args):
        began = time.monotonic()
        try:
            return function(*args)
        finally:
            phases[name] = (time.monotonic() - began) * 1000

    def _wait_seconds(self, now, began):
        delay = 1 / 60 - (now - began)
        if self.state in ACTIVE and self.lease and self.lease.is_active:
            delay = min(delay, self.lease.last_ack_at + .18 - now)
            t = self.tracker.latest
            if t is not None:
                delay = min(delay, t.captured_at + .25 - now)
                if (self._lease_target or self.pending) and (t.session, t.seq) != self._lease_frame:
                    delay = min(delay, self.lease.last_sent_at + .05 - now)
        return max(.001, delay)

    def _capture_diagnostics(self):
        if self.capture is None:
            return {'source': 'uninitialized'}
        if hasattr(self.capture, 'diagnostics'):
            return self.capture.diagnostics()
        return {'source': type(self.capture).__name__, 'status': self.capture.status}

    def _accept_sample(self, sample, now):
        previous = self.tracker.latest
        if sample is not None and self.tracker.accept(sample, now):
            if previous and self.state in ('running', 'calibrating', 'planning'):
                if sample.session != previous.session:
                    self._halt('插件会话改变，需手动继续')
                elif sample.c != previous.c or sample.point.distance(previous.point) > max(25, sample.speed * (now - previous.captured_at) * 3):
                    self._halt('检测到地图切换或位置突变，需手动继续')
            self.log.write('telemetry', telemetry=sample)
            self._emit('telemetry', telemetry=sample)
            if sample.region_name and (previous is None or (sample.ui_map, sample.zone_token) != (previous.ui_map, previous.zone_token)):
                self.log.write('region_transition', ui_map=sample.ui_map, zone_token=sample.zone_token,
                               region=sample.region_name, floor_candidates=sample.floor_candidates,
                               height_source=sample.height_source)
                self._emit('message', text='当前区域：' + sample.region_name)
            if sample.height_source != 'raw' and (previous is None or previous.height_source != sample.height_source):
                self._emit('message', text=height_status(sample))
