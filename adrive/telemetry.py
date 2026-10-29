"""AD v1/v2 pixel telemetry; v2 adds world height without resizing the panel.

All game coordinates use the shared north/west convention. The panel carries
bytes as grayscale nibbles calibrated from its own sixteen reference cells.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import binascii
import math
import struct
import time

from .models import Flags, Telemetry

VERSION = 1
BODY = struct.Struct("<2sBBIIHIiiHHH")
FRAME_BYTES = BODY.size + 2
BODY_V2 = struct.Struct("<2sBBIIHIiiHHHi")
FRAME_BYTES_V2 = BODY_V2.size + 2
COLS, ROWS, DEFAULT_CELL = 16, 8, 8
MAGENTA = (255, 0, 255)
MARKERS = (
    MAGENTA, (0, 255, 255), (255, 255, 0), (0, 0, 0),
    (255, 255, 255), (255, 0, 0), (0, 255, 0), (0, 0, 255),
    (255, 255, 255), (0, 0, 0), (0, 255, 255), MAGENTA,
    (0, 0, 255), (255, 255, 0), (255, 0, 0), (0, 255, 0),
)
KNOWN_FLAGS = sum(int(flag) for flag in Flags)


class DecodeError(ValueError):
    """An image/frame cannot be trusted as fresh, complete telemetry."""


def _round(value: float) -> int:
    # Explicit convention shared with Lua (Python's round uses ties-to-even).
    return math.floor(value + 0.5)


def encode_frame(t: Telemetry) -> bytes:
    if not all(math.isfinite(v) for v in (t.x, t.y, t.facing, t.speed)):
        raise ValueError("遥测含非有限数值")
    if not 0 <= t.speed <= 65535 / 256:
        raise ValueError("速度超出协议范围")
    if int(t.flags) & ~KNOWN_FLAGS:
        raise ValueError("未知状态位")
    if t.z is not None and (not math.isfinite(t.z) or not -134217728 <= t.z < 134217727):
        raise ValueError("高度超出协议范围")
    values = (
        b"AD", 2 if t.z is not None else VERSION, 0, t.session, t.seq, t.c, t.ui_map,
        _round(t.x * 16), _round(t.y * 16),
        _round((t.facing % math.tau) / math.tau * 65536) % 65536,
        _round(t.speed * 256), int(t.flags),
    )
    body = BODY_V2.pack(*values, _round(t.z * 16)) if t.z is not None else BODY.pack(*values)
    return body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF))


def decode_frame(data: bytes, captured_at: float | None = None) -> Telemetry:
    if len(data) not in (FRAME_BYTES, FRAME_BYTES_V2):
        raise DecodeError("色块帧长度错误")
    if binascii.crc_hqx(data[:-2], 0xFFFF) != struct.unpack_from("<H", data, len(data)-2)[0]:
        raise DecodeError("色块帧校验失败")
    codec = BODY_V2 if len(data) == FRAME_BYTES_V2 else BODY
    values = codec.unpack(data[:-2])
    magic, version, reserved, session, seq, c, ui_map, x, y, facing, speed, flags = values[:12]
    if magic != b"AD" or version != (2 if codec is BODY_V2 else VERSION) or reserved:
        raise DecodeError("不支持的色块协议")
    if flags & ~KNOWN_FLAGS:
        raise DecodeError("未知色块状态位")
    return Telemetry(session, seq, c, ui_map, x / 16, y / 16,
                     facing / 65536 * math.tau, speed / 256, Flags(flags),
                     time.monotonic() if captured_at is None else captured_at,
                     values[12] / 16 if codec is BODY_V2 else None)


@dataclass(frozen=True)
class PanelGeometry:
    x: float
    y: float
    cell: float

    @property
    def box(self) -> tuple[int, int, int, int]:
        return (math.floor(self.x), math.floor(self.y),
                math.ceil(self.x + COLS * self.cell),
                math.ceil(self.y + ROWS * self.cell))


def render_frame(t: Telemetry, cell_size: int = DEFAULT_CELL):
    """Synthetic panel for deterministic codec tests and offline replay."""
    from PIL import Image, ImageDraw
    if cell_size < 3:
        raise ValueError("色块尺寸至少为 3 像素")
    image = Image.new("RGB", (COLS * cell_size, ROWS * cell_size))
    draw = ImageDraw.Draw(image)
    cells = list(MARKERS) + [(i * 17,) * 3 for i in range(16)]
    for byte in encode_frame(t):
        cells.extend([((byte >> 4) * 17,) * 3, ((byte & 15) * 17,) * 3])
    cells.extend([(0, 0, 0)] * (COLS * 7 - len(cells)))
    cells.extend(reversed(MARKERS))
    for index, color in enumerate(cells):
        x, y = index % COLS * cell_size, index // COLS * cell_size
        draw.rectangle((x, y, x + cell_size - 1, y + cell_size - 1), fill=color)
    return image


def _sample(image, geometry: PanelGeometry, col: int, row: int) -> tuple[int, int, int]:
    x = int(geometry.x + (col + 0.5) * geometry.cell)
    y = int(geometry.y + (row + 0.5) * geometry.cell)
    if not (0 <= x < image.width and 0 <= y < image.height):
        raise DecodeError("色块面板不完整")
    value = image.getpixel((x, y))
    return tuple(value[:3])


def _color_distance(a, b) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _markers_match(image, geometry: PanelGeometry) -> bool:
    if geometry.cell < 3 or geometry.x < 0 or geometry.y < 0:
        return False
    if geometry.x + COLS * geometry.cell > image.width + 1 or geometry.y + ROWS * geometry.cell > image.height + 1:
        return False
    try:
        for row, markers in ((0, MARKERS), (7, tuple(reversed(MARKERS)))):
            for col, color in enumerate(markers):
                if _color_distance(_sample(image, geometry, col, row), color) > 110:
                    return False
    except (DecodeError, IndexError):
        return False
    return True


def locate_panel(image) -> PanelGeometry:
    """Find a complete marker pair, including fractional display scaling.

    The colored header is intentionally not present in normal grayscale data.
    Locations are accepted only after both marker rows match in full.
    """
    image = image.convert("RGB")
    if image.width < COLS * 3 or image.height < ROWS * 3:
        raise DecodeError("截图太小，找不到遥测色块")
    pixels = image.load()

    def purple(x: int, y: int) -> bool:
        r, g, b = pixels[x, y]
        return r >= 170 and b >= 170 and g <= 100

    for y in range(0, image.height - ROWS * 3 + 1, 3):
        x = 0
        while x < image.width - COLS * 3 + 1:
            if not purple(x, y):
                x += 2
                continue
            left = x
            while left > 0 and purple(left - 1, y):
                left -= 1
            right = x + 1
            while right < image.width and purple(right, y):
                right += 1
            width = right - left
            if 3 <= width <= 64:
                top = y
                while top > 0 and y - top <= 64 and purple(x, top - 1):
                    top -= 1
                # Fractional UI scale can make adjacent cells differ by one pixel.
                sizes = range(max(24, (width - 1) * 8), min(512, (width + 1) * 8) + 1)
                for eighths in sorted(sizes, key=lambda value: abs(value - width * 8)):
                    geometry = PanelGeometry(left, top, eighths / 8)
                    if _markers_match(image, geometry):
                        try:
                            decode_panel(image, geometry, 0.0)
                        except DecodeError:
                            continue
                        return geometry
            x = max(right + 1, x + 2)
    raise DecodeError("未找到完整遥测面板；检查遮挡、缩放或插件是否启用")


def decode_panel(image, geometry: PanelGeometry, captured_at: float | None = None) -> Telemetry:
    image = image.convert("RGB")
    if not _markers_match(image, geometry):
        raise DecodeError("定位色块被遮挡或尺寸变化")
    palette = [_sample(image, geometry, col, 1) for col in range(16)]
    brightness = [sum(color) / 3 for color in palette]
    if any(b - a < 3 for a, b in zip(brightness, brightness[1:])):
        raise DecodeError("灰度标定不可区分；检查 HDR、滤镜或色块遮挡")
    nibbles = []
    frame_bytes = FRAME_BYTES
    for index in range(COLS * 5):
        if index >= frame_bytes * 2:
            break
        color = _sample(image, geometry, index % COLS, 2 + index // COLS)
        distances = [_color_distance(color, reference) for reference in palette]
        nearest = min(range(16), key=distances.__getitem__)
        gaps = [_color_distance(palette[nearest], palette[neighbor])
                for neighbor in (nearest - 1, nearest + 1) if 0 <= neighbor < 16]
        if distances[nearest] > max(2.0, min(gaps) * 0.45):
            raise DecodeError("数据色块偏离标定色阶")
        nibbles.append(nearest)
        if index == 5:
            version = nibbles[4] * 16 + nibbles[5]
            if version not in (1, 2):
                raise DecodeError('不支持的色块协议')
            frame_bytes = FRAME_BYTES_V2 if version == 2 else FRAME_BYTES
    # Padding is part of the framing guard even though it is outside the CRC.
    for index in range(frame_bytes * 2, COLS * 5):
        if _color_distance(_sample(image, geometry, index % COLS, 2 + index // COLS), palette[0]) > 8:
            raise DecodeError("色块帧填充区域损坏")
    return decode_frame(bytes(a * 16 + b for a, b in zip(nibbles[::2], nibbles[1::2])), captured_at)


def decode_image(image, captured_at: float | None = None) -> Telemetry:
    return decode_panel(image, locate_panel(image), captured_at)


class FreshnessTracker:
    """Only increasing frames refresh time; changing sessions retires old ones."""

    def __init__(self):
        self.latest: Telemetry | None = None
        self._accepted_at: float | None = None
        self._retired_sessions: set[int] = set()

    def accept(self, telemetry: Telemetry, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        captured = telemetry.captured_at if telemetry.captured_at > 0 else now
        if not math.isfinite(captured) or captured > now:
            return False
        if telemetry.session in self._retired_sessions:
            return False
        if self.latest is not None:
            if telemetry.session == self.latest.session:
                delta = (telemetry.seq - self.latest.seq) & 0xFFFFFFFF
                if delta == 0 or delta >= 0x80000000:
                    return False
            else:
                self._retired_sessions.add(self.latest.session)
        # Localization can take longer than one control tick. Preserve the
        # acquisition time so an old screenshot never becomes fresh at decode.
        self.latest = replace(telemetry, captured_at=captured)
        self._accepted_at = captured
        return True

    def fresh(self, now: float | None = None, max_age: float = 0.25) -> bool:
        now = time.monotonic() if now is None else now
        return self._accepted_at is not None and 0 <= now - self._accepted_at <= max_age
