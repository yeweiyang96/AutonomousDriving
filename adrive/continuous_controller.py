"""Lease-backed movement: continuous straight travel, bounded precise turns.

This is a pure controller. A zero duration denotes a continuous target state;
positive durations are firmware-timed effects, independent of the session lease.
The runtime only renews on new safe telemetry and observes each timed effect.
"""
from __future__ import annotations

from dataclasses import replace

from .controller import Controller, Decision, unsafe_reason
from .models import Telemetry
from .pico import forward_sequence


class ContinuousController(Controller):
    """Reuse route geometry and stop rules without releasing W every 200 ms."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._stream_sample: Telemetry | None = None

    def set_route(self, *args, **kwargs):
        super().set_route(*args, **kwargs)
        self._stream_sample = None

    def pause(self, reason="用户暂停"):
        self._stream_sample = None
        super().pause(reason)

    def _observe_stream(self, t: Telemetry):
        old = self._stream_sample
        if old is None:
            return
        if t.session != old.session or not forward_sequence(t.seq, old.seq):
            return
        elapsed = t.captured_at - old.captured_at
        if not 0 < elapsed <= .25:
            return
        moved = old.point.distance(t.point)
        measured_speed = moved / elapsed
        if .2 <= measured_speed <= 50:
            self._speed = .65 * self._speed + .35 * measured_speed
        if self._progress_origin is None:
            self._progress_origin = old.point
        self._forward_ms += elapsed * 1000
        if self._forward_ms >= 2000:
            self._stuck = self._progress_origin.distance(t.point) < .5
            self._progress_origin = t.point
            self._forward_ms = 0
        self._stream_sample = t

    def _press(self, key, ms, t, now, target):
        # Long straight travel remains held between new observations. Close to
        # a waypoint, use a device-timed effect so a delayed screenshot cannot
        # carry us past the corner. Turns retain their calibrated short timing.
        if key == self.forward_key and ms > 250:
            self._last_frame = (t.session, t.seq)
            self._stream_sample = t
            return Decision("keys", key=key)
        self._stream_sample = None
        return replace(super()._press(key, ms, t, now, target), kind="keys")

    def update(self, telemetry, now):
        if not unsafe_reason(telemetry, now):
            self._observe_stream(telemetry)
        result = super().update(telemetry, now)
        if result.kind in ("stop", "replan", "arrived"):
            self._stream_sample = None
        return result
