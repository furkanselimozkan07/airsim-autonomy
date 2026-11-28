"""Pure geometry helpers: angles, frame conversions, path utilities.

Everything here is simulator-independent. Angles are in radians unless a
name says ``_deg``. AirSim uses NED (x north, y east, z down); yaw 0 points
north and grows clockwise (towards east) when seen from above.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

from .waypoints import Vec3

EARTH_RADIUS_M = 6_371_000.0


# --------------------------------------------------------------------- angles
def wrap_angle(angle: float) -> float:
    """Wrap ``angle`` to the interval ``[-pi, pi)``."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def angle_diff(target: float, current: float) -> float:
    """Shortest signed rotation (rad) taking ``current`` to ``target``."""
    return wrap_angle(target - current)


def yaw_to_target(origin: Vec3, target: Vec3) -> float:
    """Yaw (rad, NED, 0 = north, clockwise positive) from origin facing target.

    Only the horizontal components matter; identical x/y returns 0.0.
    """
    dx, dy = target.x - origin.x, target.y - origin.y
    if dx == 0 and dy == 0:
        return 0.0
    return math.atan2(dy, dx)


def bearing_deg(origin: Vec3, target: Vec3) -> float:
    """Compass bearing in degrees ``[0, 360)`` from origin to target."""
    return math.degrees(yaw_to_target(origin, target)) % 360.0


def quaternion_to_yaw(w: float, x: float, y: float, z: float) -> float:
    """Extract yaw (rad) from a unit quaternion (AirSim ``orientation``)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Quaternion ``(w, x, y, z)`` for a pure rotation of ``yaw`` about z."""
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


# -------------------------------------------------------------------- frames
def ned_to_enu(p: Vec3) -> Vec3:
    """NED ``(n, e, d)`` -> ENU ``(e, n, u)``."""
    return Vec3(p.y, p.x, -p.z)


def enu_to_ned(p: Vec3) -> Vec3:
    """ENU ``(e, n, u)`` -> NED ``(n, e, d)`` (the transform is an involution up to axes)."""
    return Vec3(p.y, p.x, -p.z)


def geodetic_to_ned(lat: float, lon: float, alt: float,
                    lat0: float, lon0: float, alt0: float = 0.0) -> Vec3:
    """Local-tangent (equirectangular) NED offset of a GPS fix from a home fix.

    Accurate to well under a metre over a few kilometres, which is plenty for
    drone missions. Latitudes/longitudes in degrees, altitudes in metres.
    """
    north = math.radians(lat - lat0) * EARTH_RADIUS_M
    east = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return Vec3(north, east, -(alt - alt0))


def ned_to_geodetic(p: Vec3, lat0: float, lon0: float,
                    alt0: float = 0.0) -> Tuple[float, float, float]:
    """Inverse of :func:`geodetic_to_ned`; returns ``(lat, lon, alt)``."""
    lat = lat0 + math.degrees(p.x / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(p.y / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon, alt0 - p.z


# --------------------------------------------------------------------- paths
def lerp(a: Vec3, b: Vec3, t: float) -> Vec3:
    """Linear interpolation between two points."""
    return Vec3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t)


def cross_track_error(start: Vec3, end: Vec3, position: Vec3) -> float:
    """Perpendicular distance from ``position`` to the segment ``start``-``end``.

    Distances past the ends are measured to the nearest endpoint.
    """
    seg = end - start
    length_sq = seg.x**2 + seg.y**2 + seg.z**2
    if length_sq == 0:
        return position.distance_to(start)
    rel = position - start
    t = (rel.x * seg.x + rel.y * seg.y + rel.z * seg.z) / length_sq
    t = max(0.0, min(1.0, t))
    return position.distance_to(lerp(start, end, t))


def path_length(points: Sequence[Vec3]) -> float:
    """Total polyline length."""
    return sum(a.distance_to(b) for a, b in zip(points, points[1:]))


def chaikin_smooth(points: Sequence[Vec3], iterations: int = 2,
                   keep_ends: bool = True) -> List[Vec3]:
    """Round the corners of a polyline with Chaikin's corner-cutting.

    Each pass replaces every segment by points at 25 % and 75 % along it.
    With ``keep_ends`` the first and last points are preserved so the path
    still starts and finishes exactly at the original waypoints.
    """
    if iterations < 0:
        raise ValueError("iterations must be >= 0")
    pts = list(points)
    for _ in range(iterations):
        if len(pts) < 3:
            break
        out: List[Vec3] = [pts[0]] if keep_ends else []
        for a, b in zip(pts, pts[1:]):
            out.append(lerp(a, b, 0.25))
            out.append(lerp(a, b, 0.75))
        if keep_ends:
            out.append(pts[-1])
        pts = out
    return pts


def resample_path(points: Sequence[Vec3], spacing: float) -> List[Vec3]:
    """Resample a polyline into points ``spacing`` metres apart (ends kept)."""
    if spacing <= 0:
        raise ValueError("spacing must be positive")
    pts = list(points)
    if len(pts) < 2:
        return pts
    out = [pts[0]]
    carry = 0.0  # distance already travelled since the last emitted point
    for a, b in zip(pts, pts[1:]):
        seg = a.distance_to(b)
        if seg == 0:
            continue
        d = spacing - carry
        while d <= seg:
            out.append(lerp(a, b, d / seg))
            d += spacing
        carry = seg - (d - spacing)
    if out[-1].distance_to(pts[-1]) > 1e-9:
        out.append(pts[-1])
    return out
