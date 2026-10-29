"""JSON Lines event log. Only this application's observations and decisions."""
from __future__ import annotations

from dataclasses import asdict, is_dataclass
from datetime import datetime
from enum import IntFlag
import json
from pathlib import Path
import threading
import time


def json_value(value):
    if is_dataclass(value):
        return {k: json_value(v) for k, v in asdict(value).items()}
    if isinstance(value, IntFlag):
        return int(value)
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set, frozenset)):
        return [json_value(v) for v in value]
    return value


class EventLog:
    def __init__(self, folder: Path):
        folder.mkdir(parents=True, exist_ok=True)
        self.path = folder / (datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.jsonl')
        self._file = self.path.open('w', encoding='utf-8', buffering=1)
        self._lock = threading.Lock()
        self.started = time.monotonic()

    def write(self, kind: str, **data) -> None:
        row = {'kind': kind, 't': time.monotonic() - self.started, **json_value(data)}
        with self._lock:
            self._file.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')

    def close(self) -> None:
        with self._lock:
            self._file.close()
