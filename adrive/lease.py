"""Nonblocking, session-scoped keyboard leases. No pulse/action retries.

SET replaces a complete desired state. RENEW never restarts a timed effect.
ACKs describe firmware/USB state, not game movement. See docs/pico-v3.md.
"""
from __future__ import annotations

from dataclasses import dataclass
import secrets
import socket
import struct
import time

from .pico import PicoError, forward_sequence

REQUEST = struct.Struct('<4sBBHHHIIIIIB6sB')
RESPONSE = struct.Struct('<4sBBBB7IHHB6sB')
QUERY, OPEN, SET, STOP, RENEW = range(5)
LEASE_MS = 250
ACK_TIMEOUT = .180


@dataclass(frozen=True)
class LeaseStatus:
    result: int
    flags: int
    token: int
    boot: int
    epoch: int
    session: int
    accepted_seq: int
    applied_seq: int
    uptime_ms: int
    lease_remaining_ms: int
    hold_remaining_ms: int
    modifier: int
    keys: tuple[int, ...]
    received_at: float

    @property
    def active(self):
        return bool(self.flags & 4)

    @property
    def pending(self):
        return bool(self.flags & 8)

    @property
    def hold_expired(self):
        return bool(self.flags & 16)

    @property
    def can_open(self):
        return bool(self.flags & 32)

    @property
    def available(self):
        return self.flags & 3 == 3


def decode_status(data: bytes, now: float) -> LeaseStatus:
    if len(data) != RESPONSE.size:
        raise PicoError('Pico 连续控制响应长度错误')
    (magic, version, result, flags, reserved, token, boot, epoch, session,
     accepted, applied, uptime, lease, hold, modifier, raw_keys, tail) = RESPONSE.unpack(data)
    keys = tuple(k for k in raw_keys if k)
    if magic != b'ADRS' or version != 3 or reserved or tail or flags & ~63 or result > 5:
        raise PicoError('Pico 连续控制响应格式错误')
    if not boot or lease > 300 or hold > 200 or len(set(keys)) != len(keys) or any(k < 4 or k > 223 for k in keys):
        raise PicoError('Pico 连续控制状态字段无效')
    if forward_sequence(applied, accepted):
        raise PicoError('Pico USB 序号超过接收序号')
    return LeaseStatus(result, flags, token, boot, epoch, session, accepted, applied,
                       uptime, lease, hold, modifier, keys, now)


class LeaseClient:
    """Driven by the capture loop. All socket operations are nonblocking.

    Call poll frequently, and renew only while the game observation is safe.
    Stop/close revoke locally before attempting a best-effort stop packet.
    """
    def __init__(self, host='192.168.11.7', port=8888):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.connect((host, int(port)))
            self._socket.setblocking(False)
        except OSError:
            self._socket.close()
            raise
        self.state = 'idle'
        self.supported: bool | None = None
        self.status: LeaseStatus | None = None
        self.last_sent_at = self.last_ack_at = 0.0
        self.seq = 0
        self._boot = self._epoch = self._session = 0
        self._started = 0.0
        self._last_acked_seq = 0
        self._last_uptime = None
        self._last_poll_at = None
        self._pending = {}
        self._open_packet = None
        self._stop_packet = None
        self._stop_until = 0.0
        self._events = []
        self._closed = False

    @property
    def is_active(self):
        return self.state == 'active'

    @property
    def wait_socket(self):
        return None if self._closed else self._socket

    def drain_events(self):
        events, self._events = self._events, []
        return events

    def _event(self, kind, **values):
        self._events.append({'kind': kind, **values})

    def _packet(self, op, *, lease=0, hold=0, keys=(), modifier=0, epoch=None):
        token = secrets.randbits(32)
        if op == QUERY:
            return REQUEST.pack(b'ADRV', 3, QUERY, 0, 0, 0, token, 0, 0, 0, 0, 0, b'\0' * 6, 0)
        return REQUEST.pack(b'ADRV', 3, op, lease, hold, 0, token, self._boot,
                            self._epoch if epoch is None else epoch, self._session,
                            self.seq, modifier, bytes(keys).ljust(6, b'\0'), 0)

    def _send(self, packet, now):
        if self._closed:
            raise PicoError('Pico 连续控制连接已关闭')
        try:
            if self._socket.send(packet) != len(packet):
                raise PicoError('Pico 状态报文未完整发送')
        except OSError as exc:
            raise PicoError(f'Pico 状态发送失败：{exc}') from exc
        fields = REQUEST.unpack(packet)
        token, op, seq = fields[6], fields[2], fields[10]
        self._pending[token] = (op, seq, now)
        # Bounded bookkeeping; expired/foreign responses cannot revive control.
        self._pending = {k: v for k, v in self._pending.items() if now - v[2] < 1.0}
        while len(self._pending) > 64:
            del self._pending[next(iter(self._pending))]
        self.last_sent_at = now
        self._event('lease_send', op=op, seq=seq, token=token, hold_ms=fields[4])

    def begin(self, now):
        if self.state in ('active', 'probing', 'opening'):
            raise PicoError('连续控制会话已经启动')
        self.state = 'probing'
        self.status = None
        self.seq = self._boot = self._epoch = 0
        self._session = secrets.randbits(32) or 1
        self._started = now
        self._last_uptime = None
        self._last_poll_at = now
        self._last_acked_seq = 0
        self._pending.clear()
        self._open_packet = self._stop_packet = None
        self._send(self._packet(QUERY), now)

    def query_only(self, now):
        """Probe support/status without opening, renewing or stopping a session."""
        if self.state not in ('idle', 'stopped', 'fault'):
            raise PicoError('已有进行中的 Pico 请求或控制会话')
        self.state = 'diagnosing'
        self.status = None
        self._started = now
        self._pending.clear()
        self._stop_packet = None
        self._send(self._packet(QUERY), now)

    def _fail(self, message, now):
        self.stop(now)
        self.state = 'fault'
        raise PicoError(message)

    def _consume(self, status, request, now):
        op, request_seq, sent = request
        self.supported = True
        self._event('lease_status', op=op, seq=status.accepted_seq, applied_seq=status.applied_seq,
                    token=status.token, rtt_ms=round((now - sent) * 1000, 3), flags=status.flags,
                    result=status.result, boot=status.boot, epoch=status.epoch,
                    session=status.session, device_uptime_ms=status.uptime_ms,
                    processed_at=now, sent_at=sent,
                    keys=list(status.keys), lease_ms=status.lease_remaining_ms)
        if self.state == 'diagnosing':
            self.status = status
            self.state = 'idle'
            return
        if self.state in ('stopped', 'fault', 'idle'):
            return
        if status.result and not (status.result == 5 and request_seq != self.seq):
            self._fail(f'Pico 拒绝连续控制请求（状态 {status.result}）；请手动重新开始', now)
        if self.state == 'probing':
            if op != QUERY:
                return
            self.status = status
            self._boot, self._epoch = status.boot, status.epoch
            if not status.available or not status.can_open or status.active:
                self._fail('Pico 尚未空闲或有其他控制会话，请停止其他客户端后重试', now)
            self._open_packet = self._packet(OPEN, lease=LEASE_MS)
            self._pending.clear()  # Other pre-OPEN queries describe an old epoch.
            self._epoch = (self._epoch + 1) & 0xFFFFFFFF
            self.state = 'opening'
            self._send(self._open_packet, now)
            return
        if status.boot != self._boot or status.epoch != self._epoch or status.session != self._session or not status.active:
            self._fail('Pico 会话已关闭、过期或设备已重启；请手动重新开始', now)
        if not status.available:
            self._fail('Pico USB 或网络不可用', now)
        if self._last_uptime is not None and forward_sequence(self._last_uptime, status.uptime_ms):
            return  # Reordered status cannot refresh the lease acknowledgement.
        if forward_sequence(status.accepted_seq, self.seq):
            self._fail('Pico 出现非本客户端的状态序号', now)
        if self.state == 'opening':
            self.status = status
            self._last_uptime = status.uptime_ms
            if not status.pending:
                self.state = 'active'
                self.last_ack_at = now
            return
        if forward_sequence(self._last_acked_seq, status.accepted_seq):
            return
        if forward_sequence(status.accepted_seq, self._last_acked_seq):
            self.last_ack_at = now
            self._last_acked_seq = status.accepted_seq
        self._last_uptime = status.uptime_ms
        self.status = status

    def poll(self, now):
        if self._closed:
            return None
        poll_gap = 0.0 if self._last_poll_at is None else max(0.0, now - self._last_poll_at)
        self._last_poll_at = now
        # An ACK already queued in the socket has no trustworthy arrival time.
        # Once the observation deadline passed, processing that old response
        # must not reauthorize movement or renew an expired control session.
        if self.state == 'active':
            self._check_ack_deadline(now, poll_gap)
        for _ in range(64):
            try:
                data = self._socket.recv(512)
            except BlockingIOError:
                break
            except OSError as exc:
                self._fail(f'Pico 状态接收失败：{exc}', now)
            if len(data) != RESPONSE.size or data[:4] != b'ADRS':
                continue
            token = struct.unpack_from('<I', data, 8)[0]
            request = self._pending.pop(token, None)
            if request is None:
                continue
            try:
                status = decode_status(data, now)
            except PicoError as exc:
                self._fail(str(exc), now)
            self._consume(status, request, now)
        if self.state in ('probing', 'opening', 'diagnosing'):
            if now - self._started >= .6:
                if self.state in ('probing', 'diagnosing'):
                    self.supported = False
                    if self.state == 'diagnosing':
                        self.state = 'idle'
                        raise PicoError('未收到 ADR v3 回应；设备可能使用旧固件或网络不可达')
                    self._fail('Pico 未回应连续控制协议；请刷入支持 ADR v3 的固件并检查网络', now)
                self._fail('Pico 连续控制会话开启超时', now)
            if now - self.last_sent_at >= .08:
                packet = self._open_packet if self.state == 'opening' and self.status is None else None
                # Replaying this OPEN is idempotent and never refreshes a lease.
                if self.state == 'opening' and (self.status is None or not self.status.active):
                    packet = self._open_packet
                self._send(packet or self._packet(QUERY), now)
        elif self.state == 'active':
            self._check_ack_deadline(now, poll_gap)
        if self.state in ('stopped', 'fault') and self._stop_packet is not None and now <= self._stop_until and now - self.last_sent_at >= .05:
            try:
                self._send(self._stop_packet, now)
            except PicoError:
                pass  # Firmware lease expiry is independent of this retry.
        return self.status

    def _check_ack_deadline(self, now, poll_gap, *, phase='poll'):
        if now - self.last_ack_at <= ACK_TIMEOUT:
            return
        # This deadline includes client scheduling/capture gaps. Do not
        # describe silence as a missing device reply when all sent states
        # have already been acknowledged and no renewal was produced.
        all_acknowledged = self.seq == self._last_acked_seq
        if poll_gap > ACK_TIMEOUT:
            cause = 'client_poll_delayed'
            message = f'客户端控制循环间隔 {poll_gap * 1000:.0f} ms，超过 180 ms 保护门限；已暂停'
        elif all_acknowledged:
            cause = 'renewal_not_sent'
            message = '客户端超过 180 ms 未及时续期；先前指令均已确认，已暂停'
        else:
            cause = 'ack_wait_timeout'
            message = ('距上次 Pico 状态确认超过 180 ms，已停止续期；'
                       f'最新状态发出 {(now - self.last_sent_at) * 1000:.0f} ms，需手动继续')
        self._event('lease_timeout', cause=cause, phase=phase, poll_gap_ms=round(poll_gap * 1000, 3),
                    ack_age_ms=round((now - self.last_ack_at) * 1000, 3),
                    send_age_ms=round((now - self.last_sent_at) * 1000, 3),
                    sent_seq=self.seq, acknowledged_seq=self._last_acked_seq,
                    all_acknowledged=all_acknowledged,
                    reported_lease_ms=self.status.lease_remaining_ms if self.status else None,
                    status_age_ms=round((now - self.status.received_at) * 1000, 3) if self.status else None)
        self._fail(message, now)

    def _require_active(self, now):
        if not self.is_active:
            raise PicoError('Pico 连续控制会话未就绪')
        # A foreground check or logging between poll and dispatch can be slow.
        # A previously active state must not authorize a late SET or RENEW.
        gap = max(0., now - self._last_poll_at) if self._last_poll_at is not None else 0.
        self._check_ack_deadline(now, gap, phase='dispatch')

    def set_keys(self, keys, now, hold_ms=0, modifier=0):
        self._require_active(now)
        keys = tuple(keys)
        if len(keys) > 6 or len(set(keys)) != len(keys) or any(type(k) is not int or not 4 <= k <= 223 for k in keys):
            raise ValueError('键盘状态必须为最多六个不同的 HID 按键')
        if type(modifier) is not int or not 0 <= modifier <= 255 or type(hold_ms) is not int or not (hold_ms == 0 or 20 <= hold_ms <= 200):
            raise ValueError('持续键盘状态或短动作时长无效')
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        self._send(self._packet(SET, lease=LEASE_MS, hold=hold_ms, keys=keys, modifier=modifier), now)
        return self.seq

    def renew(self, now):
        self._require_active(now)
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        self._send(self._packet(RENEW, lease=LEASE_MS), now)
        return self.seq

    def stop(self, now=None):
        now = time.monotonic() if now is None else now
        if self.state in ('stopped', 'fault', 'idle'):
            return
        if self.state == 'diagnosing':
            self.state = 'idle'
            return
        self.state = 'stopped'
        self._stop_until = now + .3
        if self._boot:
            self._stop_packet = self._packet(STOP)
            try:
                self._send(self._stop_packet, now)
            except PicoError as exc:
                self._event('lease_stop_error', message=str(exc))

    def close(self):
        if not self._closed:
            self.stop()
            self._closed = True
            self._socket.close()
