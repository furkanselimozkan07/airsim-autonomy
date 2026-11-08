import json
from pathlib import Path

from airsim_autonomy.cli import main
from airsim_autonomy.drone import FakeDrone

EXAMPLE = str(Path(__file__).parent.parent / "examples" / "square.json")


def write(tmp_path, data):
    f = tmp_path / "m.json"
    f.write_text(json.dumps(data))
    return str(f)


def test_validate_prints_summary(capsys):
    assert main(["validate", EXAMPLE]) == 0
    out = capsys.readouterr().out
    assert "square-5m" in out and "4 waypoints" in out


def test_invalid_file_exits_2(tmp_path, capsys):
    assert main(["validate", write(tmp_path, {"waypoints": []})]) == 2
    assert "error:" in capsys.readouterr().err


def test_run_fake_completes(capsys):
    assert main(["run", EXAMPLE, "--fake"]) == 0
    assert "completed: reached 4/4" in capsys.readouterr().out


def test_run_injected_drone(tmp_path):
    drone = FakeDrone()
    path = write(tmp_path, {"waypoints": [[5, 0, -3]]})
    assert main(["run", path], drone=drone) == 0
    assert drone.log[0] == "connect" and "land" in drone.log


def test_timeout_override_gives_failure_exit(tmp_path, capsys):
    path = write(tmp_path, {"waypoints": [[100, 0, -3]]})
    assert main(["run", path, "--fake", "--timeout", "1"]) == 1
    assert "timeout" in capsys.readouterr().out


def test_bad_timeout_override_exits_2(tmp_path):
    path = write(tmp_path, {"waypoints": [[1, 0, -3]]})
    assert main(["run", path, "--fake", "--timeout", "0"]) == 2
