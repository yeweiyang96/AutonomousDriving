"""Read-only local CASC access through the pinned, Unicode x64 CascLib build."""
from __future__ import annotations

import ctypes as C
from pathlib import Path


class CascStorage:
    def __init__(self, install: Path, library: Path, product: str = 'wowf'):
        self.lib = C.WinDLL(str(library.resolve()), use_last_error=True)
        handle = C.c_void_p
        dword = C.c_uint32
        specs = {
            'CascOpenStorage': ([C.c_wchar_p, dword, C.POINTER(handle)], C.c_bool),
            'CascOpenFile': ([handle, C.c_void_p, dword, dword, C.POINTER(handle)], C.c_bool),
            'CascGetFileSize64': ([handle, C.POINTER(C.c_uint64)], C.c_bool),
            'CascReadFile': ([handle, C.c_void_p, dword, C.POINTER(dword)], C.c_bool),
            'CascCloseFile': ([handle], C.c_bool),
            'CascCloseStorage': ([handle], C.c_bool),
        }
        for name, (args, result) in specs.items():
            fn = getattr(self.lib, name)
            fn.argtypes, fn.restype = args, result
        self.handle = handle()
        if not self.lib.CascOpenStorage(f'{install.resolve()}*{product}', 0x40, C.byref(self.handle)):
            raise C.WinError(C.get_last_error())

    def read(self, file_id: int | str) -> bytes:
        handle = C.c_void_p()
        # Strict integrity checks; missing encryption keys must fail, never yield zero-filled art.
        if isinstance(file_id, int):
            name, flags = C.c_void_p(file_id), 3 | 0x10
        else:
            encoded = C.create_string_buffer(file_id.encode('utf-8'))
            name, flags = C.cast(encoded, C.c_void_p), 0x10
        if not self.lib.CascOpenFile(self.handle, name, 0x40, flags, C.byref(handle)):
            raise C.WinError(C.get_last_error())
        try:
            size = C.c_uint64()
            if not self.lib.CascGetFileSize64(handle, C.byref(size)):
                raise C.WinError(C.get_last_error())
            if not 0 < size.value <= 64 * 1024 * 1024:
                raise ValueError(f'Unexpected map resource size: {size.value}')
            buffer = C.create_string_buffer(size.value)
            read = C.c_uint32()
            if not self.lib.CascReadFile(handle, buffer, size.value, C.byref(read)):
                raise C.WinError(C.get_last_error())
            if read.value != size.value:
                raise ValueError(f'Short CASC read: {read.value}/{size.value}')
            return buffer.raw
        finally:
            self.lib.CascCloseFile(handle)

    def close(self):
        if self.handle:
            self.lib.CascCloseStorage(self.handle)
            self.handle = C.c_void_p()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
