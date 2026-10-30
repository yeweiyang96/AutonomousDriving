"""ADRV v4: bounded, atomically replaced keyboard timelines over the v3 lease.

No retries for movement or mode-toggle effects. A firmware timestamp obtained
from the matched status bounds delayed datagrams independently of host clocks.
"""
from dataclasses import dataclass, asdict
import struct

from .lease import LeaseClient, LeaseStatus, LEASE_MS, decode_status
from .pico import PicoError, forward_sequence

WINDOW = 6
WINDOW_HEADER = struct.Struct('<IIHHI')
KEYFRAME = struct.Struct('<HB6sB')
EXTENSION = struct.Struct('<6I4BI')


@dataclass(frozen=True)
class TimedKeyframe:
    duration_ms: int
    keys: tuple[int, ...]
    modifier: int = 0

    def encode(self):
        if type(self.duration_ms) is not int or not 20 <= self.duration_ms <= 200:
            raise ValueError('序列分段必须为 20–200 ms')
        if len(self.keys)>6 or len(set(self.keys))!=len(self.keys) or any(type(k) is not int or not 4<=k<=223 for k in self.keys):
            raise ValueError('无效序列按键')
        if type(self.modifier) is not int or not 0<=self.modifier<=255:
            raise ValueError('无效修饰键')
        return KEYFRAME.pack(self.duration_ms,self.modifier,bytes(self.keys).ljust(6,b'\0'),0)


@dataclass(frozen=True)
class MotionStatus(LeaseStatus):
    revision: int
    submitted_revision: int
    completed_revision: int
    start_ms: int
    window_remaining_ms: int
    applied_event: int
    frame_index: int
    frame_count: int
    submitted_frame: int
    completed_frame: int
    capabilities: int


def decode_motion_status(data, now):
    if len(data)!=80 or data[4]!=4:
        raise PicoError('Pico 不支持 v4 时间序列')
    prefix=bytearray(data[:48]); prefix[4]=3
    base=decode_status(bytes(prefix),now)
    values=EXTENSION.unpack_from(data,48)
    revision,submitted,completed,start,remaining,event,index,count,si,ci,caps=values
    if (caps!=1 or remaining>200 or count>16 or
            (count and index>count) or si>=16 or ci>=16 or
            forward_sequence(submitted,revision) or forward_sequence(completed,submitted)):
        raise PicoError('Pico 时间序列确认无效')
    return MotionStatus(**asdict(base),**dict(zip(
        ('revision','submitted_revision','completed_revision','start_ms','window_remaining_ms',
         'applied_event','frame_index','frame_count','submitted_frame','completed_frame','capabilities'),values)))


class MotionClient(LeaseClient):
    protocol_version=4
    response_size=80
    decode_status=staticmethod(decode_motion_status)

    def begin(self,now):
        self.revision=0
        super().begin(now)

    def set_window(self,frames,now,*,event_id=0,remaining_ms=200):
        self._require_active(now)
        frames=tuple(frames)
        if not 1<=len(frames)<=16 or sum(f.duration_ms for f in frames)>200:
            raise ValueError('序列最多 16 段、总长最多 200 ms')
        encoded=b''.join(f.encode() for f in frames)
        status=self.status
        if not isinstance(status,MotionStatus) or status.capabilities!=1:
            raise PicoError('未确认 v4 时间序列能力')
        # A response timestamp is an earlier device time; do not add transport
        # delay to it. This can only shorten authorization, never extend it.
        if not 20<=remaining_ms<=200 or now-status.received_at>.15:
            raise PicoError('窗口授权或设备时钟已过期')
        if type(event_id) is not int or not 0<=event_id<=0xffffffff:
            raise ValueError('一次性动作标识无效')
        self.seq=(self.seq+1)&0xffffffff
        self.revision=((self.revision+1)&0xffffffff) or 1
        deadline=(status.uptime_ms+remaining_ms)&0xffffffff
        packet=self._packet(WINDOW,lease=LEASE_MS)+WINDOW_HEADER.pack(
            self.revision,event_id,len(frames),0,deadline)+encoded
        self._send(packet,now)
        return self.revision
