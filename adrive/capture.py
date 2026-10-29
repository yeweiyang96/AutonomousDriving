"""Windows window-bound, DPI-aware pixel telemetry capture (no input APIs)."""
from __future__ import annotations

import ctypes
from ctypes import wintypes
from dataclasses import dataclass
import os
import threading
import time

from .models import Telemetry
from .height import TerrainHeightResolver, height_status
from .telemetry import DecodeError, PanelGeometry, decode_panel, locate_panel
from .scheduling import LoopWaiter


class CaptureError(RuntimeError):
    pass


def enable_dpi_awareness() -> None:
    """Call before constructing Tk windows. Does not alter Windows settings."""
    if os.name != "nt":
        return
    user = ctypes.WinDLL("user32", use_last_error=True)
    try:
        fn = user.SetProcessDpiAwarenessContext
        fn.argtypes = [ctypes.c_void_p]
        fn.restype = wintypes.BOOL
        # Already-established process awareness returns false; leave it intact.
        fn(ctypes.c_void_p(-4))  # PER_MONITOR_AWARE_V2
    except AttributeError:
        user.SetProcessDPIAware()


class _BitmapInfoHeader(ctypes.Structure):
    _fields_ = [("biSize", wintypes.DWORD), ("biWidth", wintypes.LONG),
                ("biHeight", wintypes.LONG), ("biPlanes", wintypes.WORD),
                ("biBitCount", wintypes.WORD), ("biCompression", wintypes.DWORD),
                ("biSizeImage", wintypes.DWORD), ("biXPelsPerMeter", wintypes.LONG),
                ("biYPelsPerMeter", wintypes.LONG), ("biClrUsed", wintypes.DWORD),
                ("biClrImportant", wintypes.DWORD)]


class Win32Backend:
    """Native rectangular BitBlt capture; allocations are released on every call."""

    def __init__(self):
        if os.name != "nt":
            raise CaptureError("实时截图仅支持 Windows；其他系统可运行离线测试和回放")
        enable_dpi_awareness()
        self.user = ctypes.WinDLL("user32", use_last_error=True)
        self.gdi = ctypes.WinDLL("gdi32", use_last_error=True)
        self._enum_proc = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
        signatures = {
            "IsWindow": ([wintypes.HWND], wintypes.BOOL),
            "IsWindowVisible": ([wintypes.HWND], wintypes.BOOL),
            "IsIconic": ([wintypes.HWND], wintypes.BOOL),
            "GetWindowTextLengthW": ([wintypes.HWND], ctypes.c_int),
            "GetWindowTextW": ([wintypes.HWND, wintypes.LPWSTR, ctypes.c_int], ctypes.c_int),
            "GetWindowThreadProcessId": ([wintypes.HWND, ctypes.POINTER(wintypes.DWORD)], wintypes.DWORD),
            "GetForegroundWindow": ([], wintypes.HWND),
            "GetClientRect": ([wintypes.HWND, ctypes.POINTER(wintypes.RECT)], wintypes.BOOL),
            "ClientToScreen": ([wintypes.HWND, ctypes.POINTER(wintypes.POINT)], wintypes.BOOL),
            "GetDC": ([wintypes.HWND], wintypes.HDC),
            "ReleaseDC": ([wintypes.HWND, wintypes.HDC], ctypes.c_int),
            "EnumWindows": ([self._enum_proc, wintypes.LPARAM], wintypes.BOOL),
        }
        for name, (args, returns) in signatures.items():
            fn = getattr(self.user, name)
            fn.argtypes, fn.restype = args, returns
        signatures = {
            "CreateCompatibleDC": ([wintypes.HDC], wintypes.HDC),
            "DeleteDC": ([wintypes.HDC], wintypes.BOOL),
            "SelectObject": ([wintypes.HDC, wintypes.HANDLE], wintypes.HANDLE),
            "DeleteObject": ([wintypes.HANDLE], wintypes.BOOL),
            "CreateDIBSection": ([wintypes.HDC, ctypes.POINTER(_BitmapInfoHeader), wintypes.UINT,
                                  ctypes.POINTER(ctypes.c_void_p), wintypes.HANDLE, wintypes.DWORD], wintypes.HANDLE),
            "BitBlt": ([wintypes.HDC, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                        wintypes.HDC, ctypes.c_int, ctypes.c_int, wintypes.DWORD], wintypes.BOOL),
        }
        for name, (args, returns) in signatures.items():
            fn = getattr(self.gdi, name)
            fn.argtypes, fn.restype = args, returns

    def windows(self) -> list[tuple[int, str]]:
        result = []

        @self._enum_proc
        def callback(hwnd, _):
            if self.user.IsWindowVisible(hwnd):
                count = self.user.GetWindowTextLengthW(hwnd)
                if count:
                    title = ctypes.create_unicode_buffer(count + 1)
                    self.user.GetWindowTextW(hwnd, title, count + 1)
                    result.append((int(hwnd), title.value))
            return True

        if not self.user.EnumWindows(callback, 0):
            raise CaptureError("无法枚举 Windows 窗口")
        return result

    def valid(self, hwnd: int) -> bool:
        return bool(self.user.IsWindow(hwnd))

    def identity(self, hwnd: int) -> tuple[int, str]:
        process_id = wintypes.DWORD()
        self.user.GetWindowThreadProcessId(hwnd, ctypes.byref(process_id))
        count = self.user.GetWindowTextLengthW(hwnd)
        title = ctypes.create_unicode_buffer(count + 1)
        self.user.GetWindowTextW(hwnd, title, count + 1)
        return process_id.value, title.value

    def foreground(self) -> int:
        return int(self.user.GetForegroundWindow() or 0)

    def bounds(self, hwnd: int) -> tuple[int, int, int, int] | None:
        if not self.valid(hwnd) or self.user.IsIconic(hwnd):
            return None
        rect, point = wintypes.RECT(), wintypes.POINT(0, 0)
        if not self.user.GetClientRect(hwnd, ctypes.byref(rect)) or not self.user.ClientToScreen(hwnd, ctypes.byref(point)):
            raise CaptureError("无法读取游戏窗口客户区")
        width, height = rect.right - rect.left, rect.bottom - rect.top
        if width <= 0 or height <= 0:
            return None
        return point.x, point.y, point.x + width, point.y + height

    def grab(self, box: tuple[int, int, int, int]):
        from PIL import Image
        left, top, right, bottom = box
        width, height = right - left, bottom - top
        if width <= 0 or height <= 0 or width * height > 100_000_000:
            raise CaptureError("截图区域尺寸无效")
        source = self.user.GetDC(None)
        target = self.gdi.CreateCompatibleDC(source) if source else None
        bitmap, old = None, None
        try:
            if not source or not target:
                raise CaptureError("无法创建屏幕截图设备上下文")
            header = _BitmapInfoHeader()
            header.biSize = ctypes.sizeof(_BitmapInfoHeader)
            header.biWidth, header.biHeight = width, -height
            header.biPlanes, header.biBitCount = 1, 32
            bits = ctypes.c_void_p()
            bitmap = self.gdi.CreateDIBSection(source, ctypes.byref(header), 0, ctypes.byref(bits), None, 0)
            if not bitmap or not bits:
                raise CaptureError("无法创建截图位图")
            old = self.gdi.SelectObject(target, bitmap)
            if not old:
                raise CaptureError("无法选入截图位图")
            if not self.gdi.BitBlt(target, 0, 0, width, height, source, left, top, 0x00CC0020 | 0x40000000):
                raise CaptureError("Windows 截图失败；请使用可见的窗口或无边框模式")
            raw = ctypes.string_at(bits, width * height * 4)
            return Image.frombytes("RGB", (width, height), raw, "raw", "BGRX")
        finally:
            if old and target:
                self.gdi.SelectObject(target, old)
            if bitmap:
                self.gdi.DeleteObject(bitmap)
            if target:
                self.gdi.DeleteDC(target)
            if source:
                self.user.ReleaseDC(None, source)


def list_game_windows() -> list[tuple[int, str]]:
    if os.name != "nt":
        return []
    candidates = ("world of warcraft", "魔兽世界", "warcraft forever", "world of warcraft forever")
    return [(hwnd, title) for hwnd, title in Win32Backend().windows()
            if title.casefold() in candidates]


class WindowsCapture:
    """Bind a game window, discover its panel, then capture only that panel.

    poll() returns decoded frames, including duplicates; FreshnessTracker owns
    freshness. Screen capture errors are surfaced in status, never synthesized
    as valid coordinates. Selection by title is refused if ambiguous.
    """

    def __init__(self, window_title: str = "World of Warcraft", hwnd: int | None = None, *, _backend=None):
        self.window_title = window_title
        self.hwnd = hwnd
        self.status = "等待游戏窗口"
        self._backend = _backend if _backend is not None else Win32Backend()
        self._panel: PanelGeometry | None = None
        self._size: tuple[int, int] | None = None
        self._failures = 0
        self._next_locate = 0.0
        self._closed = False
        self._identity = None
        self.phase = "idle"
        self.last_timings: dict[str, float] = {}
        self._timings: dict[str, float] = {}

    @property
    def window_binding(self):
        """Identity of the window that produced the last capture attempt."""
        if self._closed or self.hwnd is None or self._identity is None:
            return None
        return self.hwnd, self._identity

    def _measure(self, phase, function, *args):
        self.phase = phase
        started = time.monotonic()
        try:
            return function(*args)
        finally:
            elapsed = (time.monotonic() - started) * 1000
            self._timings[phase] = self._timings.get(phase, 0.) + elapsed

    def _window_identity(self, hwnd: int):
        if hasattr(self._backend, "identity"):
            return self._backend.identity(hwnd)
        return next((item for item in self._backend.windows() if item[0] == hwnd), None)

    def foreground(self) -> bool:
        return bool(not self._closed and self.hwnd and self._backend.valid(self.hwnd)
                    and self._identity is not None and self._identity == self._window_identity(self.hwnd)
                    and self._backend.foreground() == self.hwnd)

    def _bind(self) -> bool:
        if self.hwnd and self._backend.valid(self.hwnd):
            identity = self._window_identity(self.hwnd)
            if self._identity is None:
                self._identity = identity
            if identity == self._identity:
                return True
        self.hwnd = None
        self._identity = None
        self._panel = None
        if not self.window_title.strip():
            self.status = "请明确选择游戏窗口"
            return False
        windows = [(hwnd, title) for hwnd, title in self._backend.windows()
                   if self.window_title.casefold() == title.casefold()]
        if len(windows) != 1:
            self.status = "未找到游戏窗口" if not windows else "存在多个匹配窗口，请选择具体窗口"
            return False
        self.hwnd = windows[0][0]
        self._identity = self._window_identity(self.hwnd)
        return True

    def poll(self) -> Telemetry | None:
        if self._closed:
            return None
        began = time.monotonic()
        self._timings = {}
        try:
            if not self._measure("bind", self._bind):
                return None
            box = self._measure("bounds", self._backend.bounds, self.hwnd)
            if box is None:
                self.status = "游戏窗口不可见或已最小化"
                self._panel = None
                return None
            left, top, right, bottom = box
            size = right - left, bottom - top
            if size != self._size:
                self._panel = None
                self._size = size
            if self._panel is not None:
                x0, y0, x1, y1 = self._panel.box
                captured_at = time.monotonic()
                image = self._measure("grab", self._backend.grab, (left + x0, top + y0, left + x1, top + y1))
                local = PanelGeometry(self._panel.x - x0, self._panel.y - y0, self._panel.cell)
                try:
                    telemetry = self._measure("decode", decode_panel, image, local, captured_at)
                except DecodeError as error:
                    self._failures += 1
                    self.status = str(error)
                    if self._failures >= 3:
                        self._panel = None
                    return None
                self._failures = 0
                self.status = "遥测色块已连接"
                return telemetry
            now = time.monotonic()
            if now < self._next_locate:
                return None
            self._next_locate = now + 0.5
            # Default panel is near the top left. Full client-area discovery is
            # needed only when the user has moved it with /adgps pos.
            search = (left, top, min(right, left + 1280), min(bottom, top + 360))
            try:
                captured_at = time.monotonic()
                image = self._measure("grab", self._backend.grab, search)
                panel = self._measure("locate", locate_panel, image)
            except DecodeError:
                if search == box:
                    raise
                captured_at = time.monotonic()
                image = self._measure("grab", self._backend.grab, box)
                panel = self._measure("locate", locate_panel, image)
            telemetry = self._measure("decode", decode_panel, image, panel, captured_at)
            self._panel = panel
            self._failures = 0
            self.status = "遥测色块已定位"
            return telemetry
        except (DecodeError, CaptureError, OSError) as error:
            self.status = str(error)
            return None
        finally:
            self.last_timings = {**self._timings, "total": (time.monotonic() - began) * 1000}
            self.phase = "idle"

    def close(self) -> None:
        self._closed = True
        self._panel = None
        self.status = "截图已关闭"


@dataclass(frozen=True)
class _CaptureSnapshot:
    generation: int = 0
    sample: Telemetry | None = None
    status: str = "等待采集线程"
    binding: tuple | None = None
    started_at: float = 0.
    completed_at: float = 0.
    timings: tuple[tuple[str, float], ...] = ()


class BufferedCapture:
    """One owner captures/decodes; the controller consumes a latest-only slot.

    A complete immutable snapshot replaces one reference. Readers never acquire
    a lock held by the capture worker, wait for BitBlt, or wait for panel search.
    ``captured_at`` always remains the source's pre-grab timestamp, not delivery
    time. Foreground authorization is checked live using the captured binding.
    Only the worker closes its WindowsCapture, including during a slow grab.
    """

    def __init__(self, window_title="World of Warcraft", hwnd=None, *,
                 capture_factory=WindowsCapture, _focus_backend=None,
                 interval=1 / 30, close_timeout=.1, asset_root=None,
                 height_resolver_factory=TerrainHeightResolver):
        if not 0 < interval <= 1 or not 0 <= close_timeout <= 1:
            raise ValueError("采集频率或关闭等待时间无效")
        self._factory = capture_factory
        self._arguments = {"window_title": window_title, "hwnd": hwnd}
        self._focus_backend = _focus_backend if _focus_backend is not None else Win32Backend()
        self._interval = interval
        self._close_timeout = close_timeout
        self._stop = threading.Event()
        self._waiter = LoopWaiter()
        self._wakeup = None
        self._worker_closed = threading.Event()
        self._snapshot = _CaptureSnapshot()
        self._delivered = 0
        self._inflight_started: float | None = None
        self._source = None  # Diagnostics may read phase; lifecycle stays on worker.
        self._asset_root, self._height_factory = asset_root, height_resolver_factory
        self._resolving_height = False
        self._thread = threading.Thread(target=self._run, name="telemetry-capture", daemon=True)
        self._thread.start()

    @property
    def status(self):
        return "截图已关闭" if self._stop.is_set() else self._snapshot.status

    def set_wakeup(self, callback):
        self._wakeup = callback

    def poll(self):
        if self._stop.is_set():
            return None
        snapshot = self._snapshot
        if snapshot.generation == self._delivered:
            return None
        self._delivered = snapshot.generation
        return snapshot.sample

    def foreground(self):
        if self._stop.is_set():
            return False
        binding = self._snapshot.binding
        if binding is None:
            return False
        hwnd, identity = binding
        backend = self._focus_backend
        try:
            # Check the current foreground first; no screen-capture or decoder
            # work is called here. PID/title identity guards recycled handles.
            if backend.foreground() != hwnd or not backend.valid(hwnd):
                return False
            if hasattr(backend, "identity"):
                current = backend.identity(hwnd)
            else:
                current = next((item for item in backend.windows() if item[0] == hwnd), None)
            return current == identity and backend.foreground() == hwnd
        except (CaptureError, OSError):
            return False

    def diagnostics(self):
        snapshot = self._snapshot
        began = self._inflight_started
        now = time.monotonic()
        return {
            "source": "buffered_windows_capture", "busy": began is not None,
            "busy_ms": max(0., (now - began) * 1000) if began is not None else 0.,
            "phase": ("height" if self._resolving_height else getattr(self._source, "phase", "initializing"))
                     if began is not None else "idle",
            "last_elapsed_ms": max(0., (snapshot.completed_at - snapshot.started_at) * 1000),
            "last_completed_age_ms": max(0., (now - snapshot.completed_at) * 1000) if snapshot.completed_at else None,
            "sample_age_ms": max(0., (now - snapshot.sample.captured_at) * 1000) if snapshot.sample else None,
            "generation": snapshot.generation, "status": self.status,
            "timings_ms": dict(snapshot.timings), "worker_alive": self._thread.is_alive(),
        }

    def _run(self):
        source = resolver = None
        try:
            source = self._factory(**self._arguments)
            self._source = source
            if self._asset_root is not None:
                resolver = self._height_factory(self._asset_root, cancel=self._stop)
            while not self._stop.is_set():
                began = time.monotonic()
                self._inflight_started = began
                sample = None
                height_ms = None
                try:
                    sample = source.poll()
                    status = source.status
                    if sample is not None and resolver is not None and not self._stop.is_set():
                        self._resolving_height = True
                        height_began = time.monotonic()
                        sample = resolver.resolve(sample)
                        height_ms = (time.monotonic() - height_began) * 1000
                        status += '；' + height_status(sample)
                        if resolver.error:
                            status += '：' + resolver.error
                except Exception as exc:
                    sample = None
                    status = f"采集异常：{exc}"
                finally:
                    self._resolving_height = False
                finished = time.monotonic()
                # Closing never mutates source while it owns a GDI operation.
                # A frame completing after close is not published or delivered.
                if self._stop.is_set():
                    break
                previous = self._snapshot
                timings = dict(getattr(source, "last_timings", {}))
                if height_ms is not None:
                    timings['height'] = height_ms
                self._snapshot = _CaptureSnapshot(
                    generation=previous.generation + 1, sample=sample, status=status,
                    binding=source.window_binding, started_at=began, completed_at=finished,
                    timings=tuple(timings.items()),
                )
                self._inflight_started = None
                if sample is not None and self._wakeup:
                    self._wakeup()
                if not self._stop.is_set():
                    self._waiter.wait(max(.001, self._interval - (time.monotonic() - began)))
        except Exception as exc:
            previous = self._snapshot
            self._snapshot = _CaptureSnapshot(generation=previous.generation + 1,
                                              status=f"采集初始化失败：{exc}")
        finally:
            try:
                try:
                    if resolver is not None:
                        resolver.close()
                finally:
                    if source is not None:
                        source.close()
            finally:
                self._waiter.close()
                self._inflight_started = None
                self._worker_closed.set()

    def close(self):
        self._stop.set()
        self._waiter.wake()
        # A driver call cannot be interrupted safely. Return after this bound;
        # the daemon owner will release its resources when that call returns.
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=self._close_timeout)
