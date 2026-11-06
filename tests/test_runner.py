import pytest

from airsim_autonomy import (FakeDrone, MissionRunner, MissionStatus, RunnerConfig,
                             Vec3, VelocityConfig, VelocityController, WaypointMission)


def _ctrl():
    return VelocityController(VelocityConfig(kp=1.0, max_speed=5.0, arrival_radius=2.0))


def test_completes_square_mission_and_lands():
    drone = FakeDrone()
    m = WaypointMission.from_tuples([(10, 0, -3), (10, 10, -3), (0, 10, -3)], speed=4.0)
    res = MissionRunner(drone, m, _ctrl(), RunnerConfig(dt=0.5)).run()
    assert res.success and res.waypoints_reached == 3
    assert drone.log[0] == "connect" and drone.log[-1] == "disconnect"
    assert "land" in drone.log and drone.position.z == 0.0
    assert res.path_length >= 30 - 3  # at least the leg lengths minus acceptance radii
    assert res.history[0].z == -3.0


def test_no_land_when_disabled():
    drone = FakeDrone()
    m = WaypointMission.from_tuples([(5, 0, -3)])
    MissionRunner(drone, m, _ctrl(), RunnerConfig(dt=0.5, land_on_finish=False)).run()
    assert "land" not in drone.log


def test_timeout_still_lands():
    drone = FakeDrone()
    m = WaypointMission.from_tuples([(1000, 0, -3)], speed=1.0)
    res = MissionRunner(drone, m, _ctrl(), RunnerConfig(dt=0.5, timeout=5, land_on_finish=False)).run()
    assert res.status is MissionStatus.TIMEOUT and not res.success
    assert res.ticks == 10 and "land" in drone.log


def test_collision_aborts():
    class Crashy(FakeDrone):
        def get_state(self):
            s = super().get_state()
            return type(s)(s.position, s.velocity, collided=self.time >= 1.0)

    drone = Crashy()
    m = WaypointMission.from_tuples([(100, 0, -3)])
    res = MissionRunner(drone, m, _ctrl(), RunnerConfig(dt=0.5)).run()
    assert res.status is MissionStatus.COLLISION and res.ticks == 2


def test_on_tick_callback_and_speed_limit():
    ticks = []
    drone = FakeDrone()
    m = WaypointMission.from_tuples([(20, 0, -3)], speed=2.0)
    MissionRunner(drone, m, _ctrl(), RunnerConfig(dt=0.5), on_tick=ticks.append).run()
    assert ticks and ticks[0].tick == 1 and ticks[-1].time == pytest.approx(ticks[-1].tick * 0.5)
    assert all(t.command.norm() <= 2.0 + 1e-9 for t in ticks)


def test_disconnects_on_error():
    class Broken(FakeDrone):
        def takeoff(self, altitude=3.0):
            raise RuntimeError("motor fault")

    drone = Broken()
    with pytest.raises(RuntimeError):
        MissionRunner(drone, WaypointMission.from_tuples([(1, 0, -3)])).run()
    assert drone.log[-1] == "disconnect"


def test_config_validation():
    for kw in ({"dt": 0}, {"timeout": -1}, {"takeoff_altitude": 0}):
        with pytest.raises(ValueError):
            RunnerConfig(**kw)
