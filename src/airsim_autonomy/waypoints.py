"""Waypoint missions: pure geometry and progress tracking (no simulator needed).

Coordinates follow AirSim's NED convention: x north, y east, z DOWN, so a
point 10 m above the origin has ``z = -10``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def norm(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def distance_to(self, other: "Vec3") -> float:
        return (self - other).norm()


@dataclass(frozen=True)
class Waypoint:
    position: Vec3
    speed: float = 3.0  # m/s cruise speed toward this waypoint
    acceptance_radius: float = 1.0  # metres within which it counts as reached

    def __post_init__(self) -> None:
        if self.speed <= 0:
            raise ValueError("speed must be positive")
        if self.acceptance_radius <= 0:
            raise ValueError("acceptance_radius must be positive")


class WaypointMission:
    """Ordered list of waypoints with reached/advance logic."""

    def __init__(self, waypoints: Iterable[Waypoint]) -> None:
        self.waypoints: List[Waypoint] = list(waypoints)
        if not self.waypoints:
            raise ValueError("a mission needs at least one waypoint")
        self._index = 0

    @classmethod
    def from_tuples(cls, points: Iterable[tuple], speed: float = 3.0,
                    acceptance_radius: float = 1.0) -> "WaypointMission":
        """Build a mission from ``(x, y, z)`` tuples sharing speed/radius."""
        return cls(Waypoint(Vec3(*p), speed, acceptance_radius) for p in points)

    @property
    def index(self) -> int:
        return self._index

    @property
    def finished(self) -> bool:
        return self._index >= len(self.waypoints)

    @property
    def current(self) -> Optional[Waypoint]:
        return None if self.finished else self.waypoints[self._index]

    def update(self, position: Vec3) -> Optional[Waypoint]:
        """Advance past every waypoint reached from ``position``.

        Returns the waypoint to fly to next, or ``None`` when complete.
        """
        while not self.finished:
            wp = self.waypoints[self._index]
            if position.distance_to(wp.position) <= wp.acceptance_radius:
                self._index += 1
            else:
                break
        return self.current

    def total_length(self, start: Optional[Vec3] = None) -> float:
        """Path length in metres, optionally including the leg from ``start``."""
        pts = [w.position for w in self.waypoints]
        if start is not None:
            pts.insert(0, start)
        return sum(a.distance_to(b) for a, b in zip(pts, pts[1:]))

    def progress(self) -> float:
        """Fraction of waypoints reached, in ``[0, 1]``."""
        return min(self._index, len(self.waypoints)) / len(self.waypoints)
