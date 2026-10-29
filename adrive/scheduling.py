"""Interruptible high-resolution waits; no timer-resolution/priority changes.

Windows waits on a one-shot timer, command event and optional UDP readability.
Only the control owner receives packets; waking never authorizes an action.
"""
import ctypes
from ctypes import wintypes
import os
import threading
import time


class _NetworkEvents(ctypes.Structure):
    _fields_ = [('events', ctypes.c_long), ('errors', ctypes.c_int * 10)]


class LoopWaiter:
    def __init__(self):
        self._lock = threading.Lock()
        self._closed = False
        self._socket = None
        self.last_wait = {}
        self._fallback = threading.Event() if os.name != 'nt' else None
        self._timer = self._wake = self._network = None
        if self._fallback is not None:
            return
        self._k = ctypes.WinDLL('kernel32', use_last_error=True)
        self._w = ctypes.WinDLL('ws2_32', use_last_error=True)
        for name, args, result in (
            ('CreateWaitableTimerExW', [ctypes.c_void_p, wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD], wintypes.HANDLE),
            ('CreateEventW', [ctypes.c_void_p, wintypes.BOOL, wintypes.BOOL, wintypes.LPCWSTR], wintypes.HANDLE),
            ('SetEvent', [wintypes.HANDLE], wintypes.BOOL),
            ('ResetEvent', [wintypes.HANDLE], wintypes.BOOL),
            ('CloseHandle', [wintypes.HANDLE], wintypes.BOOL),
            ('SetWaitableTimer', [wintypes.HANDLE, ctypes.POINTER(ctypes.c_longlong), wintypes.LONG,
                                  ctypes.c_void_p, ctypes.c_void_p, wintypes.BOOL], wintypes.BOOL),
            ('WaitForMultipleObjects', [wintypes.DWORD, ctypes.POINTER(wintypes.HANDLE), wintypes.BOOL, wintypes.DWORD], wintypes.DWORD),
        ):
            fn = getattr(self._k, name)
            fn.argtypes, fn.restype = args, result
        self._w.WSAEventSelect.argtypes = [ctypes.c_size_t, wintypes.HANDLE, ctypes.c_long]
        self._w.WSAEventSelect.restype = ctypes.c_int
        self._w.WSAEnumNetworkEvents.argtypes = [ctypes.c_size_t, wintypes.HANDLE, ctypes.POINTER(_NetworkEvents)]
        self._w.WSAEnumNetworkEvents.restype = ctypes.c_int
        self._timer = self._k.CreateWaitableTimerExW(None, None, 2, 0x100002)
        self._wake = self._k.CreateEventW(None, False, False, None)
        self._network = self._k.CreateEventW(None, True, False, None)
        if not all((self._timer, self._wake, self._network)):
            error = ctypes.get_last_error()
            self.close()
            raise OSError(error, '无法创建高精度控制等待器')

    def set_socket(self, sock):
        """Owner thread only. Detach before the socket is closed/replaced."""
        if self._socket is sock:
            return
        if self._fallback is not None:
            self._socket = sock
            return
        if self._socket is not None and self._socket.fileno() >= 0:
            self._w.WSAEventSelect(self._socket.fileno(), None, 0)
        self._socket = None
        self._k.ResetEvent(self._network)
        if sock is not None:
            if self._w.WSAEventSelect(sock.fileno(), self._network, 1 | 32):
                raise OSError(self._w.WSAGetLastError(), '无法订阅控制回应事件')
            self._socket = sock

    def wake(self):
        # The lock guards handle lifetime, never the blocking wait itself.
        with self._lock:
            if not self._closed:
                if self._fallback is not None:
                    self._fallback.set()
                else:
                    self._k.SetEvent(self._wake)

    def wait(self, seconds):
        if self._closed:
            return 'closed'
        seconds = max(0., seconds)
        began = time.monotonic()
        if self._fallback is not None:
            signaled = self._fallback.wait(seconds)
            self._fallback.clear()
            reason = 'signal' if signaled else 'timer'
        else:
            due = ctypes.c_longlong(-max(1, round(seconds * 10_000_000)))
            if not self._k.SetWaitableTimer(self._timer, ctypes.byref(due), 0, None, None, False):
                raise ctypes.WinError(ctypes.get_last_error())
            handles = (wintypes.HANDLE * 3)(self._wake, self._network, self._timer)
            result = self._k.WaitForMultipleObjects(3, handles, False, 0xFFFFFFFF)
            if result not in (0, 1, 2):
                raise ctypes.WinError(ctypes.get_last_error())
            reason = ('signal', 'socket', 'timer')[result]
            if result == 1 and self._socket is not None:
                events = _NetworkEvents()
                if self._w.WSAEnumNetworkEvents(self._socket.fileno(), self._network, ctypes.byref(events)):
                    raise OSError(self._w.WSAGetLastError(), '读取控制回应事件失败')
        elapsed = time.monotonic() - began
        self.last_wait = {'reason': reason, 'requested_ms': seconds * 1000,
                          'elapsed_ms': elapsed * 1000,
                          'overrun_ms': max(0., elapsed - seconds) * 1000}
        return reason

    def close(self):
        """Owner calls after wait exits; other threads use wake(), not close()."""
        with self._lock:
            if self._closed:
                return
            self.set_socket(None)
            self._closed = True
            if self._fallback is None:
                for handle in (self._timer, self._wake, self._network):
                    if handle:
                        self._k.CloseHandle(handle)
