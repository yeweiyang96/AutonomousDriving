"""One-shot, read-only firmware diagnostics on a socket separate from control.

Nothing in this module opens a lease, renews input, or acknowledges control.
Diagnostic timestamps use the device clock and have 1 ms resolution.
"""
from __future__ import annotations

from dataclasses import dataclass
import secrets
import socket
import struct

from .lease import LEASE_MS, REQUEST, LeaseStatus, decode_status
from .pico import HEALTH_REQUEST, Health, PicoError, decode_health

DIAGNOSTIC_OP = 5
DIAGNOSTIC_SIZE = 160
HEADER = struct.Struct('<4sBBH8I')
IDENTITY_EVENT = struct.Struct('<4I')
STOP_EVENT = struct.Struct('<5I')
STOP_NAMES = {
    0: '未记录停止', 1: '客户端请求停止', 2: '输入租约到期',
    3: 'Wi-Fi 不可用', 4: 'USB 不可用', 5: 'USB 传输失败',
    6: 'USB 提交停滞', 7: '旧协议全部释放', 8: '固件内部取消',
    9: '运动窗口耗尽', 10: 'USB 时间序列延误',
}
CAPABILITY_NAMES = {
    0: '连续按键', 1: '定时短动作', 2: '抢占停止',
    3: '扩展诊断', 4: 'USB 完成时间', 5: '故障计数',
    6: '原子替换运动窗口',
}


@dataclass(frozen=True)
class UsbEvent:
    epoch: int
    session: int
    seq: int
    time_ms: int
    valid: bool
    release: bool = False


@dataclass(frozen=True)
class StopEvent:
    reason: int
    time_ms: int
    epoch: int
    session: int
    seq: int
    valid: bool


def elapsed_ms(end, start):
    """Directed uint32-clock difference; ambiguous/reversed values stay unknown."""
    delta = (end - start) & 0xFFFFFFFF
    return delta if delta < 0x80000000 else None


@dataclass(frozen=True)
class FirmwareDiagnostic:
    token: int
    boot: int
    uptime_ms: int
    firmware_word: int
    capabilities: int
    lease_min_ms: int
    lease_max_ms: int
    hold_min_ms: int
    hold_max_ms: int
    protocol_min: int
    protocol_max: int
    validity: int
    stop: StopEvent
    submitted: UsbEvent
    completed: UsbEvent
    release_submitted: UsbEvent
    release_completed: UsbEvent
    net_send_errors: int
    usb_errors: int
    core0_max_gap_us: int
    core1_max_gap_us: int
    usb_unmatched_completions: int
    usb_tracking_overruns: int
    received_at: float

    @property
    def version(self):
        parts = [(self.firmware_word >> shift) & 255 for shift in (24, 16, 8, 0)]
        return '.'.join(map(str, parts[:3] if parts[-1] == 0 else parts))

    @property
    def capability_names(self):
        return tuple(name for bit, name in CAPABILITY_NAMES.items() if self.capabilities & (1 << bit))

    def compatibility_issues(self,protocol=3):
        issues = []
        for bit in (0, 1, 2):
            if not self.capabilities & (1 << bit):
                issues.append('缺少' + CAPABILITY_NAMES[bit])
        if not self.protocol_min <= protocol <= self.protocol_max:
            issues.append(f'不支持客户端 ADRV v{protocol} 协议')
        if protocol>=4 and not self.capabilities&64:issues.append('缺少原子替换运动窗口')
        if not self.lease_min_ms <= LEASE_MS <= self.lease_max_ms:
            issues.append(f'不支持 {LEASE_MS} ms 租约')
        if not self.hold_min_ms <= 20 <= 200 <= self.hold_max_ms:
            issues.append('不支持完整 20–200 ms 定时动作范围')
        return tuple(issues)

    def submit_to_complete_ms(self):
        a, b = self.submitted, self.completed
        if not a.valid or not b.valid or (a.epoch, a.session, a.seq, a.release) != (b.epoch, b.session, b.seq, b.release):
            return None
        return elapsed_ms(b.time_ms, a.time_ms)

    def stop_to_release_complete_ms(self):
        a, b = self.stop, self.release_completed
        # STOP may identify a later RENEW than the SET being released.
        if not a.valid or not b.valid or (a.epoch, a.session) != (b.epoch, b.session):
            return None
        return elapsed_ms(b.time_ms, a.time_ms)


def decode_diagnostic(data: bytes, expected_token: int, now: float):
    if len(data) != DIAGNOSTIC_SIZE:
        raise PicoError('Pico 扩展诊断长度错误')
    (magic, version, result, size, token, boot, uptime, firmware, capabilities,
     lease_bounds, hold_bounds, validity) = HEADER.unpack_from(data)
    if magic != b'ADRD' or version != 1 or result or size != DIAGNOSTIC_SIZE or token != expected_token:
        raise PicoError('Pico 扩展诊断格式、版本或查询标识不匹配')
    if not boot or validity & ~0x31F or any(data[152:]):
        raise PicoError('Pico 扩展诊断有效位或保留字段错误')
    protocol_bounds, = struct.unpack_from('<I', data, 148)
    lease_min, lease_max = lease_bounds & 0xFFFF, lease_bounds >> 16
    hold_min, hold_max = hold_bounds & 0xFFFF, hold_bounds >> 16
    protocol_min, protocol_max = protocol_bounds & 0xFFFF, protocol_bounds >> 16
    if not (0 < lease_min <= lease_max and 0 < hold_min <= hold_max and 0 < protocol_min <= protocol_max):
        raise PicoError('Pico 扩展诊断能力范围无效')
    stop_values = STOP_EVENT.unpack_from(data, 40)
    if stop_values[0] not in STOP_NAMES:
        raise PicoError('Pico 停止原因编码不支持')
    stop = StopEvent(*stop_values, bool(validity & 1))
    events = []
    for index, offset in enumerate((60, 76, 92, 108)):
        valid = bool(validity & (1 << (index + 1)))
        release = index >= 2 or bool(validity & (1 << (8 + index)))
        events.append(UsbEvent(*IDENTITY_EVENT.unpack_from(data, offset), valid, release))
    counters = struct.unpack_from('<6I', data, 124)
    return FirmwareDiagnostic(token, boot, uptime, firmware, capabilities, lease_min, lease_max,
                              hold_min, hold_max, protocol_min, protocol_max, validity, stop,
                              *events, *counters, now)


@dataclass(frozen=True)
class DiagnosticResult:
    kind: str
    message: str
    diagnostic: FirmwareDiagnostic | None = None
    lease_status: LeaseStatus | None = None
    health: Health | None = None
    context: dict | None = None
    rtt_ms: float | None = None


def describe_diagnostic(result: DiagnosticResult):
    """Chinese summary and details, without treating missing timestamps as zero."""
    d = result.diagnostic
    if d is None:
        if result.lease_status:
            s = result.lease_status
            return ('ADR v3 可达；扩展诊断未提供',
                    result.message + f'；设备可用={s.available}，会话活动={s.active}。固件版本、能力范围和 USB 时间未提供。')
        if result.health:
            return ('Pico 基本健康查询正常；扩展诊断未提供',
                    result.message + f'；USB={result.health.usb_mounted}，Wi-Fi={result.health.wifi_up}。'
                    '版本号与能力范围未提供，不能据此认定支持连续导航。')
        return 'Pico 诊断未完成', result.message
    issues = d.compatibility_issues(protocol=4)
    compatibility = '连续导航协议兼容；角色校准状态见界面校准提示' if not issues else '不兼容：' + '；'.join(issues)
    summary = (f'固件 {d.version} · ADR {d.protocol_min}–{d.protocol_max}\n'
               f'租约 {d.lease_min_ms}–{d.lease_max_ms} ms · {compatibility}')
    stop_text = (f'{STOP_NAMES[d.stop.reason]}（设备启动后 {d.stop.time_ms} ms）'
                 if d.stop.valid else '尚无有效停止记录')
    completion = d.submit_to_complete_ms()
    release = d.stop_to_release_complete_ms()
    def measured(value):
        return f'{value} ms' if value is not None else '无可关联记录'
    def event_text(event):
        return (f'序号 {event.seq}，设备时间 {event.time_ms} ms'
                if event.valid else '无有效记录')
    details = (f'能力：{"、".join(d.capability_names) or "无"}。最近停止：{stop_text}。'
               f'定时动作范围 {d.hold_min_ms}–{d.hold_max_ms} ms。'
               f'最近提交：{event_text(d.submitted)}；最近 USB 完成：{event_text(d.completed)}。'
               f'零报告提交：{event_text(d.release_submitted)}；零报告 USB 完成：{event_text(d.release_completed)}。'
               f'最近键盘报告提交→USB 完成：{measured(completion)}；停止→零报告完成：{measured(release)}。'
               f'网络发送错误={d.net_send_errors}，USB 错误={d.usb_errors}，'
               f'未匹配完成={d.usb_unmatched_completions}，跟踪溢出={d.usb_tracking_overruns}。'
               f'主循环最大间隔：Core0 {d.core0_max_gap_us / 1000:.3f} ms，'
               f'Core1 {d.core1_max_gap_us / 1000:.3f} ms。'
               'USB 时间精度为 1 ms，设备时间不能直接与电脑时钟相减；USB 完成不代表游戏已处理。')
    return summary, details


class FirmwareDiagnosticsClient:
    """Exactly one diagnostic request; only old/no replies use one health fallback."""
    def __init__(self, host='192.168.11.7', port=8888, *, timeout=.35):
        if not .05 <= timeout <= 3:
            raise ValueError('诊断超时应为 0.05–3 秒')
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.connect((host, int(port)))
            self._socket.setblocking(False)
        except OSError:
            self._socket.close()
            raise
        self.timeout = timeout
        self.active = False
        self._closed = False
        self._stage = ''
        self._token = 0
        self._sent = 0.
        self._context = None
        self._fallback_message = ''
        self._result = None

    def begin(self, now, *, context=None):
        if self._closed or self.active:
            raise PicoError('已有诊断正在进行，或诊断连接已关闭')
        self._context = dict(context or {})
        self._result = None
        self._stage = 'extended'
        self._token = secrets.randbits(32) or 1
        self._sent = now
        self.active = True
        packet = REQUEST.pack(b'ADRV', 3, DIAGNOSTIC_OP, 0, 0, 0, self._token,
                              0, 0, 0, 0, 0, b'\0' * 6, 0)
        self._send(packet)

    def _send(self, packet):
        try:
            if self._socket.send(packet) != len(packet):
                raise OSError('数据报未完整发送')
        except OSError as exc:
            self._finish(DiagnosticResult('error', f'只读诊断发送失败：{exc}', context=self._context))

    def _finish(self, result):
        self.active = False
        self._result = result

    def _fallback(self, now):
        self._stage = 'health'
        self._fallback_message = '扩展诊断未回应，可能未支持或报文丢失'
        self._token = secrets.randbits(32) or 1
        self._sent = now
        self._send(HEALTH_REQUEST.pack(0, self._token))

    def poll(self, now):
        if self._closed:
            return None
        if self.active:
            for _ in range(8):
                try:
                    data = self._socket.recv(512)
                except BlockingIOError:
                    break
                except OSError as exc:
                    self._finish(DiagnosticResult('error', f'只读诊断接收失败：{exc}', context=self._context))
                    break
                if len(data) < 12 or struct.unpack_from('<I', data, 8)[0] != self._token:
                    continue
                try:
                    elapsed = max(0., (now - self._sent) * 1000)
                    if self._stage == 'extended' and data[:4] == b'ADRD':
                        value = decode_diagnostic(data, self._token, now)
                        self._finish(DiagnosticResult('extended', '扩展诊断已读取', diagnostic=value,
                                                      context=self._context, rtt_ms=elapsed))
                    elif self._stage == 'extended' and data[:4] == b'ADRS':
                        value = decode_status(data, now)
                        if value.result != 1:
                            raise PicoError(f'扩展诊断收到意外状态 {value.result}')
                        # Old ADR v3 firmware already identifies itself; do not
                        # add another query merely to repeat that information.
                        self._finish(DiagnosticResult('legacy_v3', '该 ADR v3 固件不支持扩展诊断',
                                                      lease_status=value, context=self._context, rtt_ms=elapsed))
                    elif self._stage == 'health' and data[:4] == b'PICO':
                        health = decode_health(data, self._token, now)
                        self._finish(DiagnosticResult('legacy', self._fallback_message, health=health,
                                                      context=self._context, rtt_ms=elapsed))
                except PicoError as exc:
                    self._finish(DiagnosticResult('error', str(exc), context=self._context))
                if not self.active:
                    break
            if self.active and now - self._sent >= self.timeout:
                if self._stage == 'extended':
                    self._fallback(now)
                else:
                    self._finish(DiagnosticResult('error', 'Pico 只读诊断超时；未收到扩展或基本健康回应',
                                                  context=self._context))
        result, self._result = self._result, None
        return result

    def cancel(self):
        self.active = False
        self._result = None
        self._token = 0

    def close(self):
        if not self._closed:
            self.cancel()
            self._closed = True
            self._socket.close()
