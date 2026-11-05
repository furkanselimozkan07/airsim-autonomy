from types import SimpleNamespace as NS
from unittest.mock import MagicMock

import pytest

from airsim_autonomy import (AirSimDrone, DroneClient, FakeDrone, Vec3,
                             VelocityConfig, VelocityController, WaypointMission)


def _mock_client():
    c = MagicMock()
    vec = lambda x, y, z: NS(x_val=x, y_val=y, z_val=z)
    c.getMultirotorState.return_value = NS(kinematics_estimated=NS(
        position=vec(1, 2, -3), linear_velocity=vec(0.5, 0, -0.1)))
    c.simGetCollisionInfo.return_value = NS(has_collided=True)
    return c


def test_protocol_satisfied_by_both():
    assert isinstance(FakeDrone(), DroneClient)
    assert isinstance(AirSimDrone(client=_mock_client()), DroneClient)


def test_airsim_connect_enables_control_and_arms():
    c = _mock_client()
    AirSimDrone("Drone1", client=c).connect()
    c.confirmConnection.assert_called_once()
    c.enableApiControl.assert_called_once_with(True, "Drone1")
    c.armDisarm.assert_called_once_with(True, "Drone1")


def test_airsim_state_conversion():
    s = AirSimDrone(client=_mock_client()).get_state()
    assert s.position == Vec3(1, 2, -3)
    assert s.velocity == Vec3(0.5, 0, -0.1)
    assert s.collided is True


def test_airsim_move_and_takeoff_commands():
    c = _mock_client()
    d = AirSimDrone("V", client=c)
    d.takeoff(5.0)
    c.takeoffAsync.assert_called_once_with(vehicle_name="V")
    c.moveToZAsync.assert_called_once_with(-5.0, 2.0, vehicle_name="V")
    d.move_by_velocity(Vec3(1, 2, 3), 0.5)
    c.moveByVelocityAsync.assert_called_once_with(1, 2, 3, 0.5, vehicle_name="V")
    d.land()
    c.landAsync.assert_called_once_with(vehicle_name="V")


def test_airsim_disconnect_disarms():
    c = _mock_client()
    AirSimDrone(client=c).disconnect()
    c.armDisarm.assert_called_once_with(False, "")
    c.enableApiControl.assert_called_once_with(False, "")


@pytest.mark.parametrize("call", [lambda d: d.takeoff(0), lambda d: d.move_by_velocity(Vec3(0, 0, 0), 0)])
def test_airsim_validates_arguments(call):
    with pytest.raises(ValueError):
        call(AirSimDrone(client=_mock_client()))


def test_fake_requires_connection_and_takeoff():
    d = FakeDrone()
    with pytest.raises(RuntimeError):
        d.takeoff()
    d.connect()
    with pytest.raises(RuntimeError):
        d.move_by_velocity(Vec3(1, 0, 0), 1.0)


def test_fake_kinematics_and_land():
    d = FakeDrone()
    d.connect()
    d.takeoff(4.0)
    assert d.get_state().position.z == -4.0
    d.move_by_velocity(Vec3(2, 0, 0), 0.5)
    assert d.get_state().position == Vec3(1.0, 0, -4.0)
    assert d.get_state().velocity == Vec3(2, 0, 0)
    d.land()
    assert d.get_state().position.z == 0.0 and d.get_state().velocity.norm() == 0


def test_closed_loop_reaches_waypoints_with_fake():
    """Velocity controller + mission + fake drone converge on every waypoint."""
    mission = WaypointMission.from_tuples([(10, 0, -5), (10, 10, -5)], speed=4.0)
    ctrl = VelocityController(VelocityConfig(kp=1.0, max_speed=5.0, arrival_radius=3.0))
    d = FakeDrone()
    d.connect()
    d.takeoff(5.0)
    dt = 0.1
    for _ in range(2000):
        wp = mission.update(d.get_state().position)
        if wp is None:
            break
        d.move_by_velocity(ctrl.update(wp.position, d.get_state().position, dt, wp.speed), dt)
    assert mission.finished
