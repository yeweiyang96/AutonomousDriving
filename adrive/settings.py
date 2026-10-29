"""User configuration, kept outside reference addons and generated data."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

KEY_CODES = {chr(ord('A') + i): 4 + i for i in range(26)}
KEY_CODES.update({'UP': 82, 'DOWN': 81, 'LEFT': 80, 'RIGHT': 79})


def key_code(name: str) -> int:
    if not isinstance(name, str):
        raise ValueError('按键名称必须是文字')
    try:
        return KEY_CODES[name.strip().upper()]
    except KeyError:
        raise ValueError('按键请填写 A–Z 或 UP / DOWN / LEFT / RIGHT') from None


@dataclass
class Settings:
    host: str = '192.168.11.7'
    port: int = 8888
    window_title: str = 'World of Warcraft'
    forward_key: str = 'W'
    left_key: str = 'A'
    right_key: str = 'D'
    allow_candidates: bool = False
    calibration: dict | None = None

    def validate(self) -> None:
        if not isinstance(self.host, str) or not self.host.strip() or \
                type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError('请填写有效的 Pico 地址和端口（1–65535）')
        if not isinstance(self.window_title, str) or not self.window_title.strip():
            raise ValueError('请选择或填写游戏窗口标题')
        keys = [key_code(self.forward_key), key_code(self.left_key), key_code(self.right_key)]
        if len(set(keys)) != 3:
            raise ValueError('前进、左转、右转必须使用不同按键')
        if type(self.allow_candidates) is not bool:
            raise ValueError('候选道路开关必须是布尔值')
        if self.calibration is not None:
            names = {'left_rate_rad_s', 'right_rate_rad_s', 'input_latency_s'}
            if not isinstance(self.calibration, dict) or set(self.calibration) != names:
                raise ValueError('校准数据结构无效，请重新校准')
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in self.calibration.values()):
                raise ValueError('校准数据必须是有效数值')
            if not .1 < self.calibration['left_rate_rad_s'] < 12 or not .1 < self.calibration['right_rate_rad_s'] < 12 or \
                    not 0 <= self.calibration['input_latency_s'] <= .25:
                raise ValueError('校准参数超出范围，请重新校准')

    @classmethod
    def load(cls, path: Path | None = None) -> Settings:
        path = path or ROOT / 'runtime' / 'config.json'
        if not path.exists():
            return cls()
        values = json.loads(path.read_text(encoding='utf-8'))
        if not isinstance(values, dict):
            raise ValueError('配置文件必须是 JSON 对象')
        result = cls(**{k: v for k, v in values.items() if k in cls.__dataclass_fields__})
        result.validate()
        return result

    def save(self, path: Path | None = None) -> None:
        self.validate()
        path = path or ROOT / 'runtime' / 'config.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(asdict(self), ensure_ascii=False, indent=2), encoding='utf-8')
        temporary.replace(path)
