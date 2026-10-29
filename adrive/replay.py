"""Offline log inspection. Never imports the device/capture/runtime modules."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

from .models import Flags, Point, Route, Segment, Telemetry


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _point(p):
    if not isinstance(p, dict) or type(p.get('c')) is not int or p['c'] not in (0, 1) or \
            not _number(p.get('x')) or not _number(p.get('y')):
        raise ValueError('坐标无效')
    if p.get('z') is not None and not _number(p['z']):
        raise ValueError('高度无效')
    return Point(p['c'], float(p['x']), float(p['y']), p.get('z'))


def route_from_dict(value: dict) -> Route:
    start, goal = _point(value['start']), _point(value['goal'])
    if start.c != goal.c or not isinstance(value['segments'], list):
        raise ValueError('路线大陆或分段无效')
    segments = []
    previous = start
    for s in value['segments']:
        points = tuple(_point(p) for p in s['points'])
        if len(points) < 2 or any(p.c != start.c for p in points) or previous.distance(points[0]) > .001:
            raise ValueError('路线不连续')
        if s['reliability'] not in ('reviewed', 'candidate', 'terrain', 'navmesh') or \
                not isinstance(s['source'], str) or not isinstance(s['edge_id'], str):
            raise ValueError('路线来源无效')
        segments.append(Segment(points, s['source'], s['reliability'], s['edge_id']))
        previous = points[-1]
    if previous.distance(goal) > .001:
        raise ValueError('路线没有到达记录终点')
    expanded = value.get('expanded', 0)
    if type(expanded) is not int or expanded < 0:
        raise ValueError('搜索统计无效')
    return Route(start, goal, tuple(segments), expanded)


def telemetry_from_dict(value: dict) -> Telemetry:
    for name in ('session', 'seq', 'c', 'ui_map', 'flags'):
        if type(value.get(name)) is not int or not 0 <= value[name] <= 0xFFFFFFFF:
            raise ValueError('遥测整数无效')
    for name in ('x', 'y', 'facing', 'speed', 'captured_at'):
        if not _number(value.get(name)):
            raise ValueError('遥测数值无效')
    if not 0 <= value['facing'] < math.tau or value['speed'] < 0 or value['flags'] > 0x1FF:
        raise ValueError('遥测字段超出范围')
    if value.get('z') is not None and not _number(value['z']):
        raise ValueError('遥测高度无效')
    if value.get('raw_z') is not None and not _number(value['raw_z']):
        raise ValueError('遥测原始高度无效')
    if value.get('height_source', 'raw') not in ('raw', 'client', 'navmesh', 'ambiguous', 'unavailable'):
        raise ValueError('遥测高度来源无效')
    return Telemetry(**{**value, 'flags': Flags(value['flags'])})


@dataclass
class Replay:
    rows: list[dict]

    @classmethod
    def load(cls, path: Path | str) -> Replay:
        rows = []
        for line_no, line in enumerate(Path(path).read_text(encoding='utf-8').splitlines(), 1):
            if line.strip():
                try:
                    row = json.loads(line)
                    if not isinstance(row, dict) or 'kind' not in row:
                        raise ValueError('缺少事件类型')
                    if not isinstance(row['kind'], str) or not _number(row.get('t')) or row['t'] < 0:
                        raise ValueError('事件时间无效')
                    if rows and row['t'] < rows[-1]['t']:
                        raise ValueError('事件时间倒退')
                    if row['kind'] == 'telemetry':
                        telemetry_from_dict(row['telemetry'])
                    elif row['kind'] == 'route':
                        route_from_dict(row['route'])
                    rows.append(row)
                except (ValueError, TypeError, KeyError) as exc:
                    raise ValueError(f'日志第 {line_no} 行无效：{exc}') from exc
        return cls(rows)

    def summary(self) -> dict:
        poses = [telemetry_from_dict(r['telemetry']) for r in self.rows if r['kind'] == 'telemetry']
        routes = [route_from_dict(r['route']) for r in self.rows if r['kind'] == 'route']
        actions = [r for r in self.rows if r['kind'] == 'action']
        states = [r for r in self.rows if r['kind'] == 'state']
        distance = sum(a.point.distance(b.point) for a, b in zip(poses, poses[1:])
                       if a.flags & Flags.VALID and b.flags & Flags.VALID and a.session == b.session
                       and 0 < (b.seq - a.seq) % (2 ** 32) < 2 ** 31
                       and a.c == b.c and a.point.distance(b.point) < 100)
        return {'frames': len(poses), 'route_revisions': len(routes), 'actions': len(actions),
                'tracked_yards': round(distance, 2),
                'final_state': states[-1].get('state') if states else None,
                'final_reason': states[-1].get('reason') if states else None,
                'goal_error_yards': round(poses[-1].point.distance(routes[-1].goal), 3)
                    if poses and routes and poses[-1].flags & Flags.VALID
                    and poses[-1].c == routes[-1].goal.c else None}
