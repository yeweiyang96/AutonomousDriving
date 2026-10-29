"""Shared coordinates and immutable interfaces. Distances are world yards.

WoW world X points north, world Y points west. Facing zero is north and
positive angles turn west. Never substitute uiMapID for the continent ID.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntFlag
import math


@dataclass(frozen=True)
class Point:
    c: int
    x: float
    y: float
    z: float | None = None

    def distance(self, other: Point) -> float:
        return math.hypot(self.x - other.x, self.y - other.y) if self.c == other.c else math.inf


class Flags(IntFlag):
    VALID = 1
    COMBAT = 2
    DEAD = 4
    SWIMMING = 8
    FLYING = 16
    LOADING = 32
    INPUT_FOCUS = 64
    TAXI = 128
    TRANSPORT = 256


@dataclass(frozen=True)
class Telemetry:
    session: int
    seq: int
    c: int
    ui_map: int
    x: float
    y: float
    facing: float
    speed: float
    flags: Flags
    captured_at: float = 0.0
    z: float | None = None
    # Desktop provenance only; never changes the game's telemetry wire format.
    raw_z: float | None = None
    height_source: str = "raw"  # raw | client | navmesh | ambiguous | unavailable

    @property
    def point(self) -> Point:
        return Point(self.c, self.x, self.y, self.z)


@dataclass(frozen=True)
class Segment:
    points: tuple[Point, ...]
    source: str
    reliability: str  # reviewed | candidate | terrain | navmesh
    edge_id: str

    @property
    def distance(self) -> float:
        return sum(a.distance(b) for a, b in zip(self.points, self.points[1:]))


@dataclass(frozen=True)
class Route:
    start: Point
    goal: Point
    segments: tuple[Segment, ...]
    expanded: int = 0

    @property
    def points(self) -> tuple[Point, ...]:
        result: list[Point] = []
        for segment in self.segments:
            for point in segment.points:
                if not result or point != result[-1]:
                    result.append(point)
        return tuple(result) or (self.start, self.goal)

    @property
    def distance(self) -> float:
        return sum(segment.distance for segment in self.segments)


class PlanningError(Exception):
    """A complete executable route could not be produced."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def angle_error(target: float, current: float) -> float:
    return (target - current + math.pi) % math.tau - math.pi
