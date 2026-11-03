"""Velocity controller: position error -> clamped 3-axis velocity command.

One PID per axis turns the position error into a velocity request. The
resulting vector is then limited to a maximum speed (preserving direction, so
the drone does not drift off the straight line to the target) and, optionally,
scaled down inside an *arrival radius* so the drone decelerates smoothly
instead of overshooting the waypoint.

Coordinates follow AirSim's NED convention (z is down). Everything here is
pure Python and needs no simulator.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .pid import PID, PIDConfig
from .waypoints import Vec3


@dataclass(frozen=True)
class VelocityConfig:
    """Tuning for :class:`VelocityController`.

    ``max_speed`` caps the magnitude of the 3D command (m/s). ``max_vertical_speed``
    additionally caps the z component (climbing/descending is usually slower).
    ``arrival_radius`` > 0 enables linear slow-down within that distance of the target.
    """

    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    max_speed: float = 5.0
    max_vertical_speed: Optional[float] = None
    arrival_radius: float = 0.0

    def __post_init__(self) -> None:
        if self.max_speed <= 0:
            raise ValueError("max_speed must be positive")
        if self.max_vertical_speed is not None and self.max_vertical_speed <= 0:
            raise ValueError("max_vertical_speed must be positive")
        if self.arrival_radius < 0:
            raise ValueError("arrival_radius must be >= 0")


def limit_magnitude(v: Vec3, max_norm: float) -> Vec3:
    """Scale ``v`` down to length ``max_norm`` if longer, keeping its direction."""
    if max_norm < 0:
        raise ValueError("max_norm must be >= 0")
    n = v.norm()
    if n <= max_norm or n == 0.0:
        return v
    s = max_norm / n
    return Vec3(v.x * s, v.y * s, v.z * s)


def arrival_scale(distance: float, arrival_radius: float) -> float:
    """Slow-down factor in ``[0, 1]``: 1 outside the radius, linear to 0 at the target."""
    if arrival_radius <= 0:
        return 1.0
    return max(0.0, min(1.0, distance / arrival_radius))


class VelocityController:
    """Three per-axis PIDs plus speed limiting and arrival slow-down."""

    def __init__(self, config: VelocityConfig = VelocityConfig()) -> None:
        self.config = config
        # No per-axis clamp: clamping each axis separately would bend the command away
        # from the straight line to the target. The vector limit is applied in update().
        pid_cfg = PIDConfig(kp=config.kp, ki=config.ki, kd=config.kd)
        self._pids = (PID(pid_cfg), PID(pid_cfg), PID(pid_cfg))

    def reset(self) -> None:
        for pid in self._pids:
            pid.reset()

    def update(self, target: Vec3, position: Vec3, dt: float,
               speed_limit: Optional[float] = None) -> Vec3:
        """Velocity command (m/s, NED) to move from ``position`` toward ``target``.

        ``speed_limit`` (e.g. a waypoint's cruise speed) further lowers ``max_speed``.
        """
        c = self.config
        vx = self._pids[0].update(target.x, position.x, dt)
        vy = self._pids[1].update(target.y, position.y, dt)
        vz = self._pids[2].update(target.z, position.z, dt)

        if c.max_vertical_speed is not None:
            vz = max(-c.max_vertical_speed, min(c.max_vertical_speed, vz))

        limit = c.max_speed if speed_limit is None else min(c.max_speed, speed_limit)
        if limit <= 0:
            raise ValueError("speed_limit must be positive")
        cmd = limit_magnitude(Vec3(vx, vy, vz), limit)

        k = arrival_scale(position.distance_to(target), c.arrival_radius)
        return Vec3(cmd.x * k, cmd.y * k, cmd.z * k)
