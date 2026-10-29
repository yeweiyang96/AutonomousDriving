"""Bounded, non-retrying adapter for the existing Pico UDP HID firmware.

The firmware has no command sequence or completion ACK. Health counters are
used to detect ambiguity, never as proof that a key reached the game.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import secrets
import socket
import struct
import threading
import time

from .models import Flags, Telemetry

HEALTH_REQUEST = struct.Struct("<BI")
HEALTH_RESPONSE = struct.Struct("<4sBBBBIIII")
PRESS = struct.Struct("<BBH")
UINT32_MASK = 0xFFFFFFFF


class PicoError(RuntimeError):
    pass


class PicoTimeout(PicoError):
    pass


@dataclass(frozen=True)
class Health:
    version: int
    flags: int
    queue_depth: int
    queue_capacity: int
    uptime_ms: int
    accepted: int
    rejected: int
    received_at: float = 0.0

    @property
    def usb_mounted(self) -> bool:
        return bool(self.flags & 1)

    @property
    def keyboard_ready(self) -> bool:
        return bool(self.flags & 2)

    @property
    def mouse_ready(self) -> bool:
        return bool(self.flags & 4)

    @property
    def busy(self) -> bool:
        return bool(self.flags & 8)

    @property
    def wifi_up(self) -> bool:
        return bool(self.flags & 16)

    @property
    def ready(self) -> bool:
        return self.usb_mounted and self.keyboard_ready and self.wifi_up and not self.busy and self.queue_depth == 0


def decode_health(data: bytes, expected_token: int, received_at: float = 0.0) -> Health:
    if len(data) != HEALTH_RESPONSE.size:
        raise PicoError("Pico 健康响应长度不正确")
    magic, version, flags, depth, capacity, token, uptime, accepted, rejected = HEALTH_RESPONSE.unpack(data)
    if magic != b"PICO" or token != expected_token:
        raise PicoError("Pico 健康响应标识或查询 token 不匹配")
    if version not in (1, 2):
        raise PicoError(f"不支持的 Pico 动作协议版本：{version}")
    if capacity != 8 or depth > capacity:
        raise PicoError("Pico 队列状态无效")
    return Health(version, flags, depth, capacity, uptime, accepted, rejected, received_at)


class PicoClient:
    """One socket, one outstanding read query; movement datagrams never retry."""

    def __init__(self, host: str = "192.168.11.7", port: int = 8888, timeout: float = .15):
        if not host or not 1 <= int(port) <= 65535:
            raise ValueError("Pico 地址或端口无效")
        if not math.isfinite(timeout) or not 0 < timeout <= 10:
            raise ValueError("Pico 查询超时应在 0 到 10 秒之间")
        self.host, self.port, self.timeout = host, int(port), timeout
        self._lock = threading.Lock()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # UDP connect filters packets from other peers. It sends no packet.
            self._socket.connect((host, self.port))
            self._socket.settimeout(timeout)
        except OSError:
            self._socket.close()
            raise
        self._closed = False

    def query(self) -> Health:
        with self._lock:
            if self._closed:
                raise PicoError("Pico 连接已经关闭")
            token = secrets.randbits(32)
            deadline = time.monotonic() + self.timeout
            try:
                self._socket.send(HEALTH_REQUEST.pack(0, token))
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise PicoTimeout("Pico 查询超时；没有重发输入动作")
                    self._socket.settimeout(remaining)
                    data = self._socket.recv(512)
                    # An old query can arrive late. Ignore it until this query's
                    # deadline; do not confuse arrival with current device state.
                    if len(data) != HEALTH_RESPONSE.size or data[:4] != b"PICO":
                        continue
                    if struct.unpack_from("<I", data, 8)[0] != token:
                        continue
                    return decode_health(data, token, time.monotonic())
            except socket.timeout as exc:
                raise PicoTimeout("Pico 查询超时；没有重发输入动作") from exc
            except OSError as exc:
                raise PicoError(f"Pico 网络错误：{exc}") from exc
            finally:
                if not self._closed:
                    self._socket.settimeout(self.timeout)

    def _send(self, data: bytes) -> None:
        with self._lock:
            if self._closed:
                raise PicoError("Pico 连接已经关闭")
            try:
                if self._socket.send(data) != len(data):
                    raise PicoError("Pico 数据报未完整发送；不会重试")
            except OSError as exc:
                raise PicoError(f"Pico 输入发送失败：{exc}；不会重试") from exc

    def press(self, key: int, duration_ms: int) -> None:
        if type(key) is not int or not 4 <= key <= 0xE7:
            raise ValueError("按键必须为 USB HID usage code 4..231")
        if type(duration_ms) is not int or not 20 <= duration_ms <= 200:
            raise ValueError("导航按键时长必须为 20..200 ms")
        self._send(PRESS.pack(1, key, duration_ms))

    def release_all(self) -> None:
        # This command is queued by the existing firmware, not a preemption.
        self._send(b"\x0a")

    def close(self) -> None:
        with self._lock:
            if not self._closed:
                self._closed = True
                self._socket.close()

    def __enter__(self) -> PicoClient:
        return self

    def __exit__(self, *args: object) -> None:
        self.close()


def forward_sequence(new: int, old: int) -> bool:
    delta = (new - old) & UINT32_MASK
    return 0 < delta < 0x80000000


class ActionGate:
    """Detect ambiguous input and wait for a fresh post-action observation.

    'complete' means observed device quiescence plus a new visual sample. It is
    deliberately NOT a firmware ACK or proof of motion. Controller evaluates
    the actual position/heading effect; an ambiguous action always stops.
    """

    def __init__(self) -> None:
        self.clear()

    def clear(self) -> None:
        self.active = False
        self.baseline: Health | None = None
        self.sample: Telemetry | None = None
        self.started_at = 0.0
        self.earliest = 0.0
        self.deadline = 0.0
        self._accepted_seen = False
        self._last_uptime = 0

    def begin(self, before_health: Health, telemetry: Telemetry, now: float, duration_ms: int) -> None:
        if self.active:
            raise PicoError("前一个输入动作尚未观察完成")
        if not before_health.ready:
            raise PicoError("Pico 当前不能接收新动作：检查 USB、Wi-Fi 和队列")
        if not 0 <= now - before_health.received_at <= .25:
            raise PicoError("发令前 Pico 健康状态已过期")
        if not 20 <= duration_ms <= 200:
            raise ValueError("动作时长必须为 20..200 ms")
        if not telemetry.flags & Flags.VALID or not 0 <= now - telemetry.captured_at <= .25:
            raise PicoError("发令前遥测无效或过期")
        self.baseline, self.sample, self.started_at = before_health, telemetry, now
        self.earliest = now + duration_ms / 1000.0 + .005
        self.deadline = now + duration_ms / 1000.0 + 1.0
        self._accepted_seen = False
        self._last_uptime = before_health.uptime_ms
        self.active = True

    def _fail(self, reason: str) -> None:
        self.active = False
        raise PicoError(reason)

    def observe(self, health: Health, telemetry: Telemetry | None, now: float) -> str:
        if not self.active or self.baseline is None or self.sample is None:
            raise PicoError("没有待观察的输入动作")
        if now > self.deadline:
            self._fail("输入结果观察超时；不会重发动作")
        if health.version != self.baseline.version:
            self._fail("Pico 协议版本在执行中改变")
        if not health.usb_mounted or not health.wifi_up:
            self._fail("执行中 Pico USB 或 Wi-Fi 断开")
        uptime_delta = (health.uptime_ms - self._last_uptime) & UINT32_MASK
        if uptime_delta >= 0x80000000:
            self._fail("执行中 Pico 重启或健康响应倒退")
        self._last_uptime = health.uptime_ms
        rejected = (health.rejected - self.baseline.rejected) & UINT32_MASK
        accepted = (health.accepted - self.baseline.accepted) & UINT32_MASK
        if rejected:
            self._fail("Pico 拒绝或中止了动作")
        if accepted > 1 or health.queue_depth > 1:
            self._fail("检测到重复数据报或其他客户端输入；动作不明确")
        if self._accepted_seen and accepted != 1:
            self._fail("Pico 接收计数倒退")
        self._accepted_seen = accepted == 1
        if telemetry is not None:
            if telemetry.session != self.sample.session:
                self._fail("游戏插件会话改变；必须手动重新开始")
            delta = (telemetry.seq - self.sample.seq) & UINT32_MASK
            if delta >= 0x80000000:
                self._fail("输入后的遥测序号倒退")
        fresh_after = telemetry is not None and forward_sequence(telemetry.seq, self.sample.seq)
        fresh_after = fresh_after and self.earliest <= telemetry.captured_at <= now and now - telemetry.captured_at <= .25
        if accepted == 1 and health.ready and now >= self.earliest and fresh_after:
            if not telemetry.flags & Flags.VALID:
                self._fail("输入后的遥测无效")
            self.active = False
            return "complete"
        return "waiting"
