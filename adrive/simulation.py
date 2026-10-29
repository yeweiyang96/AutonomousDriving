"""Deterministic offline closed-loop simulation. No network, capture or HID calls."""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path

from .controller import Calibration, Controller
from .eventlog import json_value
from .models import Flags, Point, Route, Segment, Telemetry
from .telemetry import decode_frame, decode_image, encode_frame, render_frame


def sample_route() -> Route:
    # An explicitly synthetic route placed near a recognizable map location.
    # Its geometry is not a claim of passability in Orgrimmar.
    a = Point(1, 1377.398, -4371.786)
    points = (a, Point(1, a.x + 30, a.y), Point(1, a.x + 30, a.y + 35), Point(1, a.x + 65, a.y + 35))
    return Route(points[0], points[-1], tuple(
        Segment((p, q), 'offline-simulation', 'terrain', f'sim:{i}')
        for i, (p, q) in enumerate(zip(points, points[1:]))))


def simulate(route=None, *, speed_change=True, use_pixels=False, limit=5000):
    route = route or sample_route()
    controller = Controller(Calibration(2.1, 2.05))
    controller.set_route(route)
    now, seq, facing, speed = 0.0, 1, math.tau - .4, 7.0
    position = route.start
    rows = [{'kind': 'session', 't': 0.0, 'source': 'deterministic offline simulator; no physical input'},
            {'kind': 'route', 't': 0.0, 'route': json_value(route)}]
    actions = 0
    maximum_cross_track = 0.0

    def observe():
        t = Telemetry(12345, seq, position.c, 1411, position.x, position.y, facing,
                      speed, Flags.VALID, now)
        if use_pixels:
            t = decode_image(render_frame(t), captured_at=now)
        else:
            t = decode_frame(encode_frame(t), captured_at=now)
        rows.append({'kind': 'telemetry', 't': now, 'telemetry': json_value(t)})
        return t

    for iteration in range(limit):
        t = observe()
        decision = controller.update(t, now)
        if decision.kind == 'arrived':
            rows.append({'kind': 'state', 't': now, 'state': 'arrived', 'reason': '离线模拟到达'})
            return rows, {'arrived': True, 'actions': actions, 'seconds': round(now, 3),
                          'goal_error_yards': round(position.distance(route.goal), 4),
                          'maximum_cross_track_yards': round(maximum_cross_track, 4)}
        if decision.kind != 'press':
            raise AssertionError(f'Simulation stopped: {decision.kind}: {decision.reason}')
        rows.append({'kind': 'action', 't': now, 'action': 'press', 'key': decision.key,
                     'duration_ms': decision.duration_ms, 'simulated': True})
        actions += 1
        duration = decision.duration_ms / 1000
        if decision.key == 4:
            facing = (facing + 2.1 * duration) % math.tau
        elif decision.key == 7:
            facing = (facing - 2.05 * duration) % math.tau
        else:
            if speed_change and actions >= 35:
                speed = 11.2
            position = Point(position.c, position.x + speed * duration * math.cos(facing),
                             position.y + speed * duration * math.sin(facing))
        # Render a later observation after device idle; include acquisition delay.
        now += duration + .04
        seq += max(1, round((duration + .04) * 30))
        after = observe()
        if not controller.on_action_complete(after, now):
            raise AssertionError('Post-action observation rejected')
        for segment in route.segments:
            if segment.edge_id == controller.current_edge_id:
                a, b = segment.points[0], segment.points[-1]
                dx, dy = b.x - a.x, b.y - a.y
                length2 = dx * dx + dy * dy
                fraction = min(1, max(0, ((position.x - a.x) * dx + (position.y - a.y) * dy) / length2)) if length2 else 0
                cross = position.distance(Point(a.c, a.x + fraction * dx, a.y + fraction * dy))
                maximum_cross_track = max(maximum_cross_track, cross)
                break
        now += .01
        seq += 1
    raise AssertionError('Simulation iteration budget exceeded')


def main(argv=None):
    parser = argparse.ArgumentParser(description='无硬件闭环模拟，生成可回放记录')
    parser.add_argument('--output', type=Path, default=Path('runtime/simulation.jsonl'))
    parser.add_argument('--pixels', action='store_true', help='每帧实际生成并读取色块图像')
    args = parser.parse_args(argv)
    rows, summary = simulate(use_pixels=args.pixels)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(''.join(json.dumps(r, ensure_ascii=False, allow_nan=False) + '\n' for r in rows), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f'回放记录：{args.output.resolve()}')


if __name__ == '__main__':
    main()
