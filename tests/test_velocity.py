import math

import pytest

from airsim_autonomy import Vec3
from airsim_autonomy.velocity import (
    VelocityConfig, VelocityController, arrival_scale, limit_magnitude,
)


def test_limit_magnitude_keeps_direction():
    v = limit_magnitude(Vec3(3, 4, 0), 2.5)
    assert v.norm() == pytest.approx(2.5)
    assert v.x / v.y == pytest.approx(3 / 4)


def test_limit_magnitude_noop_and_zero():
    assert limit_magnitude(Vec3(1, 0, 0), 5) == Vec3(1, 0, 0)
    assert limit_magnitude(Vec3(0, 0, 0), 0) == Vec3(0, 0, 0)
    with pytest.raises(ValueError):
        limit_magnitude(Vec3(1, 0, 0), -1)


def test_arrival_scale():
    assert arrival_scale(10, 0) == 1.0
    assert arrival_scale(10, 5) == 1.0
    assert arrival_scale(2.5, 5) == pytest.approx(0.5)
    assert arrival_scale(0, 5) == 0.0


def test_config_validation():
    with pytest.raises(ValueError):
        VelocityConfig(max_speed=0)
    with pytest.raises(ValueError):
        VelocityConfig(max_vertical_speed=-1)
    with pytest.raises(ValueError):
        VelocityConfig(arrival_radius=-1)


def test_command_points_toward_target_and_is_clamped():
    ctrl = VelocityController(VelocityConfig(kp=2.0, max_speed=4.0))
    cmd = ctrl.update(Vec3(100, 0, 0), Vec3(0, 0, 0), dt=0.1)
    assert cmd.x == pytest.approx(4.0)
    assert cmd.y == 0 and cmd.z == 0


def test_diagonal_direction_preserved_when_clamped():
    ctrl = VelocityController(VelocityConfig(kp=10.0, max_speed=3.0))
    cmd = ctrl.update(Vec3(30, 40, 0), Vec3(0, 0, 0), dt=0.1)
    assert cmd.norm() == pytest.approx(3.0)
    assert cmd.y / cmd.x == pytest.approx(4 / 3)


def test_speed_limit_overrides_lower_and_ignores_higher():
    ctrl = VelocityController(VelocityConfig(kp=10.0, max_speed=5.0))
    assert ctrl.update(Vec3(50, 0, 0), Vec3(0, 0, 0), 0.1, speed_limit=2.0).norm() == pytest.approx(2.0)
    ctrl.reset()
    assert ctrl.update(Vec3(50, 0, 0), Vec3(0, 0, 0), 0.1, speed_limit=9.0).norm() == pytest.approx(5.0)
    with pytest.raises(ValueError):
        ctrl.update(Vec3(1, 0, 0), Vec3(0, 0, 0), 0.1, speed_limit=0)


def test_vertical_speed_cap_ned():
    ctrl = VelocityController(VelocityConfig(kp=5.0, max_speed=5.0, max_vertical_speed=1.0))
    cmd = ctrl.update(Vec3(0, 0, -20), Vec3(0, 0, 0), 0.1)  # climb: z negative
    assert cmd.z == pytest.approx(-1.0)


def test_arrival_slowdown():
    ctrl = VelocityController(VelocityConfig(kp=1.0, max_speed=5.0, arrival_radius=10.0))
    far = ctrl.update(Vec3(20, 0, 0), Vec3(0, 0, 0), 0.1)
    ctrl.reset()
    near = ctrl.update(Vec3(5, 0, 0), Vec3(0, 0, 0), 0.1)
    assert near.x == pytest.approx(5 * 0.5)
    assert far.x == pytest.approx(5.0)


def test_closed_loop_converges_without_overshoot():
    ctrl = VelocityController(VelocityConfig(kp=1.0, max_speed=3.0, arrival_radius=4.0))
    pos, target, dt = Vec3(0, 0, 0), Vec3(20, 10, -5), 0.05
    for _ in range(4000):
        v = ctrl.update(target, pos, dt)
        pos = Vec3(pos.x + v.x * dt, pos.y + v.y * dt, pos.z + v.z * dt)
    assert pos.distance_to(target) < 0.1
    assert not math.isnan(pos.x)


def test_reset_clears_state():
    ctrl = VelocityController(VelocityConfig(kp=0.0, ki=1.0, max_speed=10))
    a = ctrl.update(Vec3(1, 0, 0), Vec3(0, 0, 0), 1.0)
    ctrl.update(Vec3(1, 0, 0), Vec3(0, 0, 0), 1.0)
    ctrl.reset()
    assert ctrl.update(Vec3(1, 0, 0), Vec3(0, 0, 0), 1.0) == a
