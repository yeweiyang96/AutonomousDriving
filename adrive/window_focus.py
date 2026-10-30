"""One foreground request from an explicit UI click; never synthesize input."""
from ctypes import wintypes
from dataclasses import dataclass
import time

from .capture import CaptureError, Win32Backend


class WindowFocusBackend(Win32Backend):
    def __init__(self):
        super().__init__()
        self.user.ShowWindowAsync.argtypes = [wintypes.HWND, wintypes.INT]
        self.user.ShowWindowAsync.restype = wintypes.BOOL
        self.user.SetForegroundWindow.argtypes = [wintypes.HWND]
        self.user.SetForegroundWindow.restype = wintypes.BOOL

    def restore(self, hwnd):
        # Do not wait on a hung game's message loop or change a normal/maximized
        # window. Runtime still waits for a visible, fresh telemetry frame.
        if self.user.IsIconic(hwnd) and not self.user.ShowWindowAsync(hwnd, 9):  # SW_RESTORE
            raise CaptureError('无法恢复最小化的游戏窗口；本次操作未启动')

    def activate(self, hwnd):
        return bool(self.user.SetForegroundWindow(hwnd))


@dataclass(frozen=True)
class FocusRequest:
    backend: WindowFocusBackend
    hwnd: int
    identity: tuple[int, str]
    deadline: float

    def verify(self):
        if (not self.backend.valid(self.hwnd) or not self.identity[0]
                or self.backend.identity(self.hwnd) != self.identity):
            raise CaptureError('游戏窗口已关闭或发生变化；本次操作未启动')

    def poll(self, now=None) -> bool:
        # Activation/restoration can finish after the API call returns. Only
        # observe here; never repeat the foreground request or synthesize keys.
        now = time.monotonic() if now is None else now
        self.verify()
        if now >= self.deadline:
            raise CaptureError('等待游戏前台确认超时；本次操作未启动，请重试')
        return self.backend.foreground() == self.hwnd


def focus_game_window(window_title: str, *, _backend=None) -> FocusRequest:
    """Resolve exactly one game window and request foreground once.

    SetForegroundWindow is called by the UI thread handling the user's click.
    Windows may deny it. No Alt key, mouse click, input-thread attachment,
    retry loop or global foreground policy change is used to bypass denial.
    """
    deadline = time.monotonic() + 1.0  # Preflight only; no input session yet.
    backend = _backend if _backend is not None else WindowFocusBackend()
    title = window_title.strip()
    if not title:
        raise CaptureError('请先选择游戏窗口')
    matches = [hwnd for hwnd, name in backend.windows() if name.casefold() == title.casefold()]
    if not matches:
        raise CaptureError('未找到配置的游戏窗口；本次操作未启动')
    if len(matches) != 1:
        raise CaptureError('存在多个同名游戏窗口，无法确定切换目标；本次操作未启动')
    hwnd = matches[0]
    identity = backend.identity(hwnd)
    if identity[1].casefold() != title.casefold():
        raise CaptureError('游戏窗口已关闭或发生变化；本次操作未启动')
    request = FocusRequest(backend, hwnd, identity, deadline)
    request.verify()
    backend.restore(hwnd)
    request.verify()
    if backend.foreground() != hwnd:
        backend.activate(hwnd)
    request.verify()
    return request
