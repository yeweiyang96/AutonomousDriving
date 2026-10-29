"""Windows global pause/stop keys. This module never synthesizes keyboard input."""
from __future__ import annotations

import ctypes
from ctypes import wintypes
import os
import threading


class Hotkeys:
    def __init__(self, callback, on_error=lambda message: None):
        self.callback, self.on_error = callback, on_error
        self.thread = None
        self.thread_id = None

    def start(self):
        if os.name != 'nt':
            return
        self.thread = threading.Thread(target=self._run, name='hotkeys', daemon=True)
        self.thread.start()

    def _run(self):
        user32, kernel32 = ctypes.windll.user32, ctypes.windll.kernel32
        self.thread_id = kernel32.GetCurrentThreadId()
        msg = wintypes.MSG()
        user32.PeekMessageW(ctypes.byref(msg), None, 0, 0, 0)
        registered = []
        try:
            for ident, vk in ((1, 0x77), (2, 0x7B)):  # F8, F12; NOREPEAT
                if user32.RegisterHotKey(None, ident, 0x4000, vk):
                    registered.append(ident)
                else:
                    self.on_error(f'快捷键 {"F8" if ident == 1 else "F12"} 被其他程序占用，请使用窗口按钮')
            while user32.GetMessageW(ctypes.byref(msg), None, 0, 0) > 0:
                if msg.message == 0x0312:
                    self.callback('toggle' if msg.wParam == 1 else 'stop')
        finally:
            for ident in registered:
                user32.UnregisterHotKey(None, ident)

    def close(self):
        if self.thread_id:
            ctypes.windll.user32.PostThreadMessageW(self.thread_id, 0x0012, 0, 0)
        if self.thread:
            self.thread.join(timeout=.5)
