"""User configuration, kept outside reference addons and generated data."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Literal

ROOT = Path(__file__).resolve().parent.parent

KEY_CODES = {chr(ord('A') + i): 4 + i for i in range(26)}
KEY_CODES.update({'[': 47, ']': 48, '/': 56,
                  'UP': 82, 'DOWN': 81, 'LEFT': 80, 'RIGHT': 79, 'NUMPAD_DIVIDE': 84})


def key_code(name: str) -> int:
    if not isinstance(name, str):
        raise ValueError('按键名称必须是文字')
    try:
        return KEY_CODES[name.strip().upper()]
    except KeyError:
        raise ValueError('按键请填写 A–Z、[、]、/、UP / DOWN / LEFT / RIGHT 或 NUMPAD_DIVIDE') from None


def key_name(code: int) -> str:
    return next((name for name,value in KEY_CODES.items() if value==code),f'HID {code}')


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
    strafe_left_key: str = 'Q'
    strafe_right_key: str = 'E'
    walk_toggle_key: str = 'NUMPAD_DIVIDE'
    movement_calibration: dict | None = None

    def validate(self) -> None:
        if not isinstance(self.host, str) or not self.host.strip() or \
                type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError('请填写有效的 Pico 地址和端口（1–65535）')
        if not isinstance(self.window_title, str) or not self.window_title.strip():
            raise ValueError('请选择或填写游戏窗口标题')
        keys = [key_code(getattr(self,n)) for n in ('forward_key','left_key','right_key','strafe_left_key','strafe_right_key','walk_toggle_key')]
        if len(set(keys)) != len(keys):
            raise ValueError('前进、转向、平移、走跑切换必须使用不同按键')
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
        if self.movement_calibration is not None:
            names={'strafe_left_angle','strafe_right_angle','diagonal_left_angle','diagonal_right_angle','strafe_verified'}
            if not isinstance(self.movement_calibration,dict) or set(self.movement_calibration)!=names:
                raise ValueError('平移校准结构无效，请重新校准')
            if self.movement_calibration['strafe_verified'] is not True or any(
                type(self.movement_calibration[n]) not in (int,float) or not math.isfinite(self.movement_calibration[n])
                or abs(self.movement_calibration[n])>math.pi for n in names-{'strafe_verified'}):
                raise ValueError('平移校准无效')

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


def calibration_state(settings: Settings) -> Literal['none', 'turn_only', 'complete']:
    """Summarize validated settings for the continuous-navigation preflight."""
    if not settings.calibration:
        return 'none'
    if not settings.movement_calibration or settings.movement_calibration.get('strafe_verified') is not True:
        return 'turn_only'
    return 'complete'


def calibration_status_text(settings: Settings) -> str:
    return {
        'none': '尚未完成转向与平移校准',
        'turn_only': '仅保存转向校准；平移校准尚未完成',
        'complete': '转向与平移校准已保存；修改转向速度或移动按键后需重做',
    }[calibration_state(settings)]


def calibration_required_reason(settings: Settings) -> str | None:
    state = calibration_state(settings)
    if state == 'complete':
        return None
    missing = '仅保存了转向校准，平移校准尚未完成。' if state == 'turn_only' else '尚未完成转向与平移校准。'
    return missing + '请先将角色停在空旷地面，再点击“转向与平移校准”；程序会自动切回游戏，等待校准完成后再开始导航。'
