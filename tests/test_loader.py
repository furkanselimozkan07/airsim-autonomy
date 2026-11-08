import json
from pathlib import Path

import pytest

from airsim_autonomy import MissionFileError, load_mission, parse_mission


def doc(**kw):
    d = {"waypoints": [[1, 2, -3]]}
    d.update(kw)
    return d


def test_minimal_uses_defaults():
    spec = parse_mission(doc())
    wp = spec.mission.waypoints[0]
    assert (wp.position.x, wp.position.z, wp.speed, wp.acceptance_radius) == (1, -3, 3.0, 1.0)
    assert spec.name == "mission"


def test_defaults_and_overrides():
    spec = parse_mission(doc(
        defaults={"speed": 5, "acceptance_radius": 2},
        waypoints=[[0, 0, -1], {"position": [1, 1, -1], "speed": 1.5}]))
    a, b = spec.mission.waypoints
    assert (a.speed, a.acceptance_radius) == (5, 2)
    assert (b.speed, b.acceptance_radius) == (1.5, 2)


def test_runner_and_controller_config():
    spec = parse_mission(doc(runner={"dt": 0.1, "timeout": 50},
                             controller={"kp": 2.0, "max_speed": 7}))
    assert spec.runner.dt == 0.1 and spec.runner.timeout == 50
    assert spec.controller.kp == 2.0 and spec.controller.max_speed == 7


@pytest.mark.parametrize("bad, fragment", [
    ([], "top level"),
    ({}, "non-empty"),
    ({"waypoints": []}, "non-empty"),
    ({"waypoints": [[1, 2]]}, r"waypoints\[0\]"),
    ({"waypoints": [[1, 2, "x"]]}, r"waypoints\[0\]\[2\]"),
    ({"waypoints": [[1, True, 3]]}, "expected a number"),
    ({"waypoints": [{"speed": 2}]}, "missing 'position'"),
    ({"waypoints": [{"position": [0, 0, 0], "spd": 2}]}, "unknown key"),
    ({"waypoints": [{"position": [0, 0, 0], "speed": -1}]}, "speed"),
    ({"waypoints": [[0, 0, 0]], "runner": {"dt": 0}}, "runner"),
    ({"waypoints": [[0, 0, 0]], "runner": {"bogus": 1}}, "unknown key"),
    ({"waypoints": [[0, 0, 0]], "controller": []}, "controller"),
    ({"waypoints": [[0, 0, 0]], "defaults": {"x": 1}}, "defaults"),
    ({"waypoints": [[0, 0, 0]], "name": 5}, "name"),
])
def test_invalid_documents(bad, fragment):
    with pytest.raises(MissionFileError, match=fragment):
        parse_mission(bad)


def test_load_from_file(tmp_path):
    f = tmp_path / "m.json"
    f.write_text(json.dumps(doc(name="t")))
    assert load_mission(f).name == "t"


def test_load_errors(tmp_path):
    with pytest.raises(MissionFileError, match="cannot read"):
        load_mission(tmp_path / "missing.json")
    f = tmp_path / "bad.json"
    f.write_text("{not json")
    with pytest.raises(MissionFileError, match="invalid JSON"):
        load_mission(f)


def test_example_mission_is_valid():
    spec = load_mission(Path(__file__).parent.parent / "examples" / "square.json")
    assert len(spec.mission.waypoints) == 4
    assert spec.mission.waypoints[2].speed == 2.0
