import pytest

from airsim_autonomy import Vec3, Waypoint, WaypointMission


def test_distance():
    assert Vec3(0, 0, 0).distance_to(Vec3(3, 4, 0)) == pytest.approx(5.0)


def test_waypoint_validation():
    with pytest.raises(ValueError):
        Waypoint(Vec3(0, 0, 0), speed=0)
    with pytest.raises(ValueError):
        Waypoint(Vec3(0, 0, 0), acceptance_radius=-1)


def test_empty_mission_rejected():
    with pytest.raises(ValueError):
        WaypointMission([])


def test_advance_through_waypoints():
    m = WaypointMission.from_tuples([(10, 0, -5), (10, 10, -5)], acceptance_radius=1.0)
    assert m.update(Vec3(0, 0, 0)).position == Vec3(10, 0, -5)
    assert m.index == 0
    assert m.update(Vec3(9.5, 0, -5)).position == Vec3(10, 10, -5)
    assert m.index == 1
    assert m.update(Vec3(10, 10, -5)) is None
    assert m.finished and m.current is None
    assert m.progress() == 1.0


def test_skips_multiple_when_overlapping():
    m = WaypointMission.from_tuples([(0, 0, 0), (0.5, 0, 0), (50, 0, 0)])
    assert m.update(Vec3(0, 0, 0)).position == Vec3(50, 0, 0)


def test_total_length():
    m = WaypointMission.from_tuples([(3, 4, 0), (3, 4, 12)])
    assert m.total_length() == pytest.approx(12.0)
    assert m.total_length(start=Vec3(0, 0, 0)) == pytest.approx(17.0)
