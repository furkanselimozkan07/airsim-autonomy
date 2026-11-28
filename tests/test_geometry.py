import math

import pytest

from airsim_autonomy import Vec3
from airsim_autonomy.geometry import (angle_diff, bearing_deg, chaikin_smooth,
                                      cross_track_error, enu_to_ned,
                                      geodetic_to_ned, ned_to_enu,
                                      ned_to_geodetic, path_length,
                                      quaternion_to_yaw, resample_path,
                                      wrap_angle, yaw_to_quaternion,
                                      yaw_to_target)


def test_wrap_angle():
    assert wrap_angle(3 * math.pi) == pytest.approx(-math.pi)
    assert wrap_angle(-0.5) == pytest.approx(-0.5)
    assert wrap_angle(2 * math.pi + 0.1) == pytest.approx(0.1)


def test_angle_diff_takes_short_way():
    assert angle_diff(math.radians(350), math.radians(10)) == pytest.approx(math.radians(-20))
    assert angle_diff(math.radians(10), math.radians(350)) == pytest.approx(math.radians(20))


def test_yaw_and_bearing():
    o = Vec3(0, 0, 0)
    assert yaw_to_target(o, Vec3(10, 0, 0)) == pytest.approx(0.0)  # north
    assert yaw_to_target(o, Vec3(0, 10, 0)) == pytest.approx(math.pi / 2)  # east
    assert yaw_to_target(o, Vec3(0, 0, -5)) == 0.0  # straight up: undefined -> 0
    assert bearing_deg(o, Vec3(0, -10, 0)) == pytest.approx(270.0)
    assert bearing_deg(o, Vec3(-10, 0, 3)) == pytest.approx(180.0)


def test_quaternion_roundtrip():
    for yaw in (-2.5, -0.3, 0.0, 1.2, 3.0):
        q = yaw_to_quaternion(yaw)
        assert quaternion_to_yaw(*q) == pytest.approx(yaw)


def test_ned_enu_roundtrip():
    p = Vec3(1, 2, -3)
    assert ned_to_enu(p) == Vec3(2, 1, 3)
    assert enu_to_ned(ned_to_enu(p)) == p


def test_geodetic_roundtrip_and_scale():
    lat0, lon0 = 41.0, 29.0  # Istanbul
    p = geodetic_to_ned(41.001, 29.0, 120.0, lat0, lon0, alt0=100.0)
    assert p.x == pytest.approx(111.19, abs=0.1)  # ~111 m per 0.001 deg lat
    assert p.y == pytest.approx(0.0)
    assert p.z == pytest.approx(-20.0)
    lat, lon, alt = ned_to_geodetic(Vec3(50, -70, -10), lat0, lon0, 0.0)
    back = geodetic_to_ned(lat, lon, alt, lat0, lon0, 0.0)
    assert (back.x, back.y, back.z) == pytest.approx((50, -70, -10))


def test_cross_track_error():
    a, b = Vec3(0, 0, 0), Vec3(10, 0, 0)
    assert cross_track_error(a, b, Vec3(5, 3, 0)) == pytest.approx(3.0)
    assert cross_track_error(a, b, Vec3(-4, 3, 0)) == pytest.approx(5.0)  # before start
    assert cross_track_error(a, a, Vec3(0, 2, 0)) == pytest.approx(2.0)  # degenerate


def test_chaikin_keeps_ends_and_shortens():
    square = [Vec3(0, 0, -5), Vec3(10, 0, -5), Vec3(10, 10, -5)]
    smooth = chaikin_smooth(square, iterations=1)
    assert smooth[0] == square[0] and smooth[-1] == square[-1]
    assert len(smooth) == 6
    assert path_length(smooth) < path_length(square)
    assert chaikin_smooth(square, 0) == square
    assert chaikin_smooth(square[:2], 3) == square[:2]
    with pytest.raises(ValueError):
        chaikin_smooth(square, -1)


def test_resample_even_spacing():
    pts = resample_path([Vec3(0, 0, 0), Vec3(10, 0, 0), Vec3(10, 10, 0)], 4.0)
    assert pts[0] == Vec3(0, 0, 0) and pts[-1] == Vec3(10, 10, 0)
    gaps = [a.distance_to(b) for a, b in zip(pts, pts[1:-1])]
    assert all(g <= 4.0 + 1e-9 for g in gaps)
    assert pts[1].x == pytest.approx(4.0) and pts[2].x == pytest.approx(8.0)
    # the point after the corner continues the 4 m spacing around the bend
    assert pts[3].distance_to(pts[2]) == pytest.approx(4.0, abs=1.5)
    with pytest.raises(ValueError):
        resample_path(pts, 0)
