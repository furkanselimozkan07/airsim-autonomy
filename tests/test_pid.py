import pytest

from airsim_autonomy import PID, PIDConfig


def test_proportional_only():
    pid = PID(PIDConfig(kp=2.0))
    assert pid.update(10.0, 4.0, 0.1) == pytest.approx(12.0)


def test_integral_accumulates():
    pid = PID(PIDConfig(kp=0.0, ki=1.0))
    pid.update(1.0, 0.0, 0.5)
    assert pid.update(1.0, 0.0, 0.5) == pytest.approx(1.0)


def test_output_clamped():
    pid = PID(PIDConfig(kp=100.0, out_min=-1.0, out_max=1.0))
    assert pid.update(5.0, 0.0, 0.1) == 1.0
    assert pid.update(-5.0, 0.0, 0.1) == -1.0


def test_anti_windup_limits_integral():
    pid = PID(PIDConfig(kp=0.0, ki=1.0, out_min=-1.0, out_max=1.0))
    for _ in range(1000):
        pid.update(10.0, 0.0, 0.1)
    # After the setpoint flips, output must leave saturation quickly.
    out = None
    for _ in range(20):
        out = pid.update(-10.0, 0.0, 0.1)
    assert out == -1.0


def test_derivative_on_measurement_no_kick():
    pid = PID(PIDConfig(kp=0.0, kd=1.0))
    pid.update(0.0, 0.0, 0.1)
    # Setpoint jump with unchanged measurement must not produce a spike.
    assert pid.update(100.0, 0.0, 0.1) == pytest.approx(0.0)


def test_derivative_damps_motion():
    pid = PID(PIDConfig(kp=0.0, kd=1.0))
    pid.update(0.0, 0.0, 0.1)
    assert pid.update(0.0, 1.0, 0.1) == pytest.approx(-10.0)


def test_reset_clears_state():
    pid = PID(PIDConfig(kp=0.0, ki=1.0))
    pid.update(1.0, 0.0, 1.0)
    pid.reset()
    assert pid.update(0.0, 0.0, 1.0) == 0.0


def test_invalid_args():
    with pytest.raises(ValueError):
        PID(PIDConfig(kp=1.0)).update(0, 0, 0)
    with pytest.raises(ValueError):
        PIDConfig(kp=1.0, out_min=1, out_max=0)


def test_converges_on_simple_plant():
    pid = PID(PIDConfig(kp=1.5, ki=0.2, kd=0.4, out_min=-5, out_max=5))
    pos, dt = 0.0, 0.05
    for _ in range(600):
        pos += pid.update(10.0, pos, dt) * dt
    assert pos == pytest.approx(10.0, abs=0.05)
