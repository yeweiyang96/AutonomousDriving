"""Pure pulse controller. This module never sends input or reads the screen."""
from __future__ import annotations

from dataclasses import dataclass
import math

from .models import Flags, Point, Route, Telemetry, angle_error
from .height import height_status
from .pico import forward_sequence


@dataclass(frozen=True)
class Calibration:
    left_rate_rad_s: float
    right_rate_rad_s: float
    input_latency_s: float = 0.0

    def __post_init__(self) -> None:
        if not all(math.isfinite(v) for v in (self.left_rate_rad_s, self.right_rate_rad_s, self.input_latency_s)):
            raise ValueError("校准值必须为有限数值")
        if not .05 <= self.left_rate_rad_s <= 20 or not .05 <= self.right_rate_rad_s <= 20:
            raise ValueError("转向速度校准无效")
        if not 0 <= self.input_latency_s <= 1:
            raise ValueError("输入延迟校准无效")


@dataclass(frozen=True)
class Decision:
    kind: str
    key: int | None = None
    duration_ms: int = 0
    reason: str = ""
    blocked_edge: str | None = None


@dataclass(frozen=True)
class PendingAction:
    key: int
    duration_ms: int
    telemetry: Telemetry
    started_at: float
    target: Point


def unsafe_reason(t: Telemetry, now: float) -> str:
    if not all(math.isfinite(v) for v in (t.x, t.y, t.facing, t.speed, t.captured_at, now)):
        return "遥测包含非有限数值"
    if t.z is not None and not math.isfinite(t.z):
        return '遥测高度无效'
    if not 0 <= now - t.captured_at <= .25:
        return "遥测过期或时间无效"
    if not t.flags & Flags.VALID:
        return "位置无效"
    checks = (
        (Flags.COMBAT, "进入战斗"), (Flags.DEAD, "角色死亡"),
        (Flags.SWIMMING, "进入水中"), (Flags.FLYING, "角色正在飞行"),
        (Flags.LOADING, "游戏正在加载"), (Flags.INPUT_FOCUS, "聊天或编辑框获得焦点"),
        (Flags.TAXI, "角色正在乘坐飞行交通"), (Flags.TRANSPORT, "角色正在乘坐运输工具"),
    )
    for flag, reason in checks:
        if t.flags & flag:
            return reason
    if t.c not in (0, 1):
        return "当前大陆不在首版支持范围"
    return ""


class Controller:
    """Follow each straight edge in order, align first, then bounded forward pulses."""

    def __init__(self, calibration: Calibration | None = None, forward_key: int = 26,
                 left_key: int = 4, right_key: int = 7):
        keys = (forward_key, left_key, right_key)
        if any(type(k) is not int or not 4 <= k <= 0xE7 for k in keys) or len(set(keys)) != 3:
            raise ValueError("前进、左转、右转必须是不同的有效 HID 按键")
        self.calibration = calibration
        self.forward_key, self.left_key, self.right_key = keys
        self.route: Route | None = None
        self.state = "idle"
        self.reason = ""
        self.pending: PendingAction | None = None
        self.blocked_edges: set[str] = set()
        self.replan_count = 0
        self._edges: list[tuple[Point, Point, str]] = []
        self._precise = False
        self._index = 0
        self._session: int | None = None
        self._last_frame: tuple[int, int] | None = None
        self._aligning = True
        self._speed = 7.0
        self._forward_ms = 0
        self._progress_origin: Point | None = None
        self._stuck = False
        self._turn_failures = 0

    @property
    def current_edge_id(self) -> str | None:
        return self._edges[self._index][2] if self._index < len(self._edges) else None

    @property
    def waypoint_index(self) -> int:
        return self._index

    def set_calibration(self, calibration: Calibration) -> None:
        self.calibration = calibration

    def set_route(self, route: Route, new_task: bool = True) -> None:
        edges: list[tuple[Point, Point, str]] = []
        precise = any(s.reliability == 'navmesh' for s in route.segments)
        if precise and any(p.z is None or not math.isfinite(p.z) for p in (route.start, route.goal, *route.points)):
            raise ValueError('精细导航路线缺少有效高度')
        previous = route.start
        if not all(math.isfinite(v) for p in (route.start, route.goal) for v in (p.x, p.y)):
            raise ValueError("路线起终点坐标无效")
        if route.start.c != route.goal.c or route.start.c not in (0, 1):
            raise ValueError("路线不能跨大陆")
        for segment in route.segments:
            if not segment.points:
                raise ValueError("路线包含空分段")
            if any(p.c != route.start.c or not math.isfinite(p.x) or not math.isfinite(p.y) for p in segment.points):
                raise ValueError("路线分段坐标或大陆无效")
            if previous.distance(segment.points[0]) > 1e-6:
                raise ValueError("路线分段不连续")
            if precise and abs(previous.z-segment.points[0].z) > .8:
                raise ValueError('精细路线包含不连续的楼层跳转')
            for a, b in zip(segment.points, segment.points[1:]):
                if a.c != route.start.c or b.c != route.start.c:
                    raise ValueError("路线分段大陆不一致")
                if not all(math.isfinite(v) for v in (a.x, a.y, b.x, b.y)):
                    raise ValueError("路线坐标无效")
                if a.distance(b) > 1e-6:
                    edges.append((a, b, segment.edge_id))
            previous = segment.points[-1]
        if previous.distance(route.goal) > 1e-6:
            raise ValueError("路线没有到达原始终点")
        self.route, self._edges, self._index = route, edges, 0
        self._precise = precise
        self.pending = None
        self.state, self.reason = "aligning", ""
        self._aligning = True
        self._last_frame = None
        self._forward_ms = 0
        self._progress_origin = None
        self._stuck = False
        self._turn_failures = 0
        if new_task:
            self.replan_count = 0
            self.blocked_edges.clear()
            self._session = None
            self._speed = 7.0

    def pause(self, reason: str = "用户暂停") -> None:
        self.pending = None
        self.state, self.reason = "paused", reason

    def reset_observation_session(self) -> None:
        """Called only for explicit manual resume, not an automatic replan.

        A plugin reload invalidates an in-flight task. A user may accept the new
        session after stopping; preserve failed-edge and replan budgets while
        waiting for the newly requested route.
        """
        if self.pending is not None or self.state not in ("idle", "paused", "fault", "arrived"):
            raise ValueError("只能在停止或暂停后接受新的遥测会话")
        self._session = None
        self._last_frame = None

    def _stop(self, reason: str) -> Decision:
        self.pending = None
        self.state, self.reason = "fault", reason
        return Decision("stop", reason=reason)

    def _replan(self, reason: str, block: bool = False) -> Decision:
        edge = self.current_edge_id if block else None
        if edge:
            self.blocked_edges.add(edge)
        if self.replan_count >= 2:
            return self._stop("两次重新规划后仍无法继续：" + reason)
        self.replan_count += 1
        self.pending = None
        self.state, self.reason = "replanning", reason
        return Decision("replan", reason=reason, blocked_edge=edge)

    def _press(self, key: int, ms: float, t: Telemetry, now: float, target: Point) -> Decision:
        duration = max(20, min(200, round(ms)))
        self.pending = PendingAction(key, duration, t, now, target)
        self._last_frame = (t.session, t.seq)
        return Decision("press", key, duration)

    def on_action_complete(self, telemetry: Telemetry, now: float) -> bool:
        """Runtime must first observe idle device plus a post-action visual frame."""
        pending = self.pending
        if pending is None:
            return False
        reason = unsafe_reason(telemetry, now)
        if reason:
            self._stop(reason)
            return False
        old = pending.telemetry
        if telemetry.session != old.session:
            self._stop("游戏插件重新加载；必须手动重新开始")
            return False
        if not forward_sequence(telemetry.seq, old.seq):
            return False
        if telemetry.captured_at < pending.started_at + pending.duration_ms / 1000.0 + .005:
            return False
        if telemetry.c != old.c:
            self._stop("执行中大陆改变")
            return False
        self.pending = None
        if pending.key == self.forward_key:
            moved = old.point.distance(telemetry.point)
            measured_speed = moved * 1000.0 / pending.duration_ms
            if .2 <= measured_speed <= 50:
                self._speed = .65 * self._speed + .35 * measured_speed
            if self._progress_origin is None:
                self._progress_origin = old.point
            self._forward_ms += pending.duration_ms
            if self._forward_ms >= 2000:
                self._stuck = self._progress_origin.distance(telemetry.point) < .5
                self._progress_origin = telemetry.point
                self._forward_ms = 0
        else:
            before = abs(angle_error(math.atan2(pending.target.y - old.y, pending.target.x - old.x), old.facing))
            after = abs(angle_error(math.atan2(pending.target.y - telemetry.y, pending.target.x - telemetry.x), telemetry.facing))
            self._turn_failures = self._turn_failures + 1 if after >= before - math.radians(.2) else 0
        return True

    def update(self, telemetry: Telemetry, now: float) -> Decision:
        if self.state in ("idle", "paused", "fault", "arrived", "replanning"):
            return Decision("none", reason=self.reason)
        reason = unsafe_reason(telemetry, now)
        if reason:
            return self._stop(reason)
        if self.calibration is None:
            return self._stop("尚未完成左右转向校准")
        if self.route is None:
            return self._stop("没有可执行路线")
        if telemetry.c != self.route.start.c:
            return self._stop("角色大陆与路线不一致")
        if self._session is None:
            self._session = telemetry.session
        elif telemetry.session != self._session:
            return self._stop("游戏插件重新加载；必须手动重新开始")
        if self.pending is not None:
            return Decision("none", reason="等待上一动作的设备状态和新遥测")
        if self._last_frame == (telemetry.session, telemetry.seq):
            return Decision("none", reason="等待新遥测")
        if self._stuck:
            return self._replan("累计前进两秒但位移不足 0.5 码", block=True)
        if self._turn_failures >= 8:
            return self._stop("连续转向未能改善朝向；请检查按键并重新校准")
        p = telemetry.point
        precise = self._precise
        if precise and telemetry.z is None:
            return self._stop(height_status(telemetry))
        def reached(point, radius):
            return p.distance(point) <= radius and (not precise or point.z is None or abs(p.z-point.z) <= 1.0)
        # Intermediate corners have a tighter radius than the final goal. Never
        # shortcut by looking for the globally nearest point on a looping route.
        while self._index < len(self._edges) - 1 and reached(self._edges[self._index][1], .3 if precise else .6):
            self._index += 1
            self._aligning = True
        if self._index >= len(self._edges) or (
            self._index == len(self._edges) - 1 and reached(self.route.goal, .6 if precise else 2.0)
        ):
            if not reached(self.route.goal, .6 if precise else 2.0):
                return self._stop("路线没有剩余路径，角色却未到终点")
            self.state, self.reason = "arrived", "已到达原始终点" if precise else "已到达原始终点 2 码范围"
            return Decision("arrived", reason=self.reason)
        a, b, _ = self._edges[self._index]
        dx, dy = b.x - a.x, b.y - a.y
        length = math.hypot(dx, dy)
        fraction = ((p.x - a.x) * dx + (p.y - a.y) * dy) / (length * length)
        projection = max(0.0, min(1.0, fraction))
        nearest = Point(a.c, a.x + projection * dx, a.y + projection * dy)
        cross_track = p.distance(nearest)
        if precise:
            expected_z = a.z + projection*(b.z-a.z)
            if abs(p.z-expected_z) > 2.0:
                return self._stop('角色高度与路线地面不符；请重新确认所在楼层')
            if cross_track > .75:
                return self._replan('偏离精细导航走廊，按当前位置重新检查碰撞')
        if cross_track > 15.0:
            return self._replan("偏离当前路线超过 15 码")
        # Rejoin this edge locally instead of cutting directly to a distant turn.
        target = b
        if cross_track > 3.0:
            ahead = min(1.0, projection + 3.0 / length)
            target = Point(a.c, a.x + ahead * dx, a.y + ahead * dy)
            self._aligning = True
        heading = math.atan2(target.y - p.y, target.x - p.x)
        error = angle_error(heading, telemetry.facing)
        if abs(error) > math.radians(5):
            self._aligning = True
        if self._aligning and abs(error) > math.radians(3):
            self.state = "aligning"
            rate = self.calibration.left_rate_rad_s if error > 0 else self.calibration.right_rate_rad_s
            key = self.left_key if error > 0 else self.right_key
            return self._press(key, abs(error) / rate * 1000.0, telemetry, now, target)
        self._aligning = False
        self.state = "moving"
        speed = max(self._speed, telemetry.speed if .2 <= telemetry.speed <= 50 else 0.0)
        remaining = p.distance(target)
        # Finish an intermediate edge precisely; the last endpoint retains the
        # requested point even though arrival allows the specified 2-yard radius.
        stop_margin = (.1 if self._index < len(self._edges) - 1 else .35) if precise else (.25 if self._index < len(self._edges) - 1 else 1.5)
        milliseconds = max(0.0, remaining - stop_margin) / max(speed, .2) * 1000.0
        return self._press(self.forward_key, milliseconds, telemetry, now, target)
