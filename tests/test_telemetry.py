import io
import math

import pytest

from airsim_autonomy import (FakeDrone, MissionRunner, TelemetryLogger, WaypointMission,
                             read_telemetry, summarize)
from airsim_autonomy.cli import main
from airsim_autonomy.drone import DroneState
from airsim_autonomy.runner import TickInfo
from airsim_autonomy.telemetry import FIELDS, tick_to_row
from airsim_autonomy.waypoints import Vec3


def info(tick=1, pos=(1, 2, -3), collided=False):
    return TickInfo(tick, tick * 0.2, DroneState(Vec3(*pos), Vec3(3, 4, 0), collided),
                    Vec3(10, 2, -3), Vec3(1, 0, 0), 2)


def test_row_matches_header():
    assert len(tick_to_row(info())) == len(FIELDS)


def test_roundtrip_stream():
    buf = io.StringIO()
    log = TelemetryLogger(buf)
    log.on_tick(info(1))
    log.on_tick(info(2, collided=True))
    log.close()
    assert not buf.closed and log.rows_written == 2
    rows = read_telemetry(buf.getvalue().splitlines())
    assert [r.tick for r in rows] == [1, 2]
    assert rows[0].position == (1, 2, -3) and rows[0].waypoint == 2
    assert rows[0].speed == pytest.approx(5.0)
    assert rows[0].distance_to_target == pytest.approx(9.0)
    assert not rows[0].collided and rows[1].collided


def test_path_creates_parent_dirs(tmp_path):
    p = tmp_path / "a" / "b" / "t.csv"
    with TelemetryLogger(p) as log:
        log(info())
    assert len(read_telemetry(p)) == 1


def test_read_rejects_foreign_csv():
    with pytest.raises(ValueError, match="missing columns"):
        read_telemetry(["a,b", "1,2"])


def test_summarize_empty_is_none():
    assert summarize([]) is None


def test_summarize_values():
    buf = io.StringIO()
    log = TelemetryLogger(buf)
    for i, x in enumerate([0, 3, 6]):
        log.on_tick(info(i + 1, pos=(x, 0, -2 - i), collided=(i == 2)))
    s = summarize(read_telemetry(buf.getvalue().splitlines()))
    assert s.samples == 3 and s.duration == pytest.approx(0.4)
    assert s.path_length == pytest.approx(2 * math.sqrt(10))
    assert s.min_altitude == -4 and s.max_altitude == -2
    assert s.collisions == 1


def test_runner_integration(tmp_path):
    path = tmp_path / "flight.csv"
    mission = WaypointMission.from_tuples([(5, 0, -3)], speed=3)
    with TelemetryLogger(path) as log:
        result = MissionRunner(FakeDrone(), mission, on_tick=log.on_tick).run()
    rows = read_telemetry(path)
    assert result.success and len(rows) == result.ticks
    assert rows[-1].distance_to_target < 1.5


def test_cli_log_and_stats(tmp_path, capsys):
    import json
    m = tmp_path / "m.json"
    m.write_text(json.dumps({"waypoints": [[5, 0, -3]]}))
    csv_path = tmp_path / "out.csv"
    assert main(["run", str(m), "--fake", "--log", str(csv_path)]) == 0
    assert "telemetry:" in capsys.readouterr().out
    assert main(["stats", str(csv_path)]) == 0
    assert "samples" in capsys.readouterr().out


def test_cli_stats_errors(tmp_path, capsys):
    assert main(["stats", str(tmp_path / "nope.csv")]) == 2
    bad = tmp_path / "bad.csv"
    bad.write_text("x,y\n1,2\n")
    assert main(["stats", str(bad)]) == 2
    empty = tmp_path / "empty.csv"
    empty.write_text(",".join(FIELDS) + "\n")
    assert main(["stats", str(empty)]) == 2
