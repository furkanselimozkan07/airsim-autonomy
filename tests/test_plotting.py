import pytest

from airsim_autonomy.cli import main
from airsim_autonomy.plotting import flight_series, plot_flight, waypoint_markers
from airsim_autonomy.telemetry import TelemetryRow


def row(i, pos, target=(10, 0, -5), wp=0, collided=False):
    return TelemetryRow(tick=i, time=5.0 + i * 0.5, position=pos, velocity=(3, 4, 0),
                        target=target, command=(0, 0, 0), waypoint=wp, collided=collided)


ROWS = [row(0, (0, 0, 0)), row(1, (1, 2, -3), collided=True),
        row(2, (5, 6, -5), target=(20, 5, -8), wp=1)]


def test_series_converts_ned_to_enu():
    s = flight_series(ROWS)
    assert s.east == [0, 2, 6] and s.north == [0, 1, 5]
    assert s.altitude == [0, 3, 5]
    assert s.time == [0.0, 0.5, 1.0]
    assert s.speed == [5.0] * 3
    assert s.collisions == [1]
    assert s.error[0] == pytest.approx((100 + 25) ** 0.5)


def test_series_empty_raises():
    with pytest.raises(ValueError):
        flight_series([])


def test_waypoint_markers_one_per_index():
    assert waypoint_markers(ROWS) == [(0, 10, 5), (5, 20, 8)]


def test_plot_writes_image(tmp_path):
    pytest.importorskip("matplotlib")
    out = plot_flight(ROWS, tmp_path / "sub" / "f.png")
    assert out.exists() and out.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


def test_cli_run_then_plot(tmp_path, capsys):
    pytest.importorskip("matplotlib")
    csv_path, png = tmp_path / "f.csv", tmp_path / "f.png"
    assert main(["run", "examples/square.json", "--fake", "--log", str(csv_path)]) == 0
    assert main(["plot", str(csv_path), "--out", str(png), "--title", "T"]) == 0
    assert png.exists() and "plot ->" in capsys.readouterr().out


def test_cli_plot_errors(tmp_path, capsys):
    assert main(["plot", str(tmp_path / "missing.csv")]) == 2
    bad = tmp_path / "bad.csv"
    bad.write_text("a,b\n1,2\n")
    assert main(["plot", str(bad)]) == 2
    assert "error:" in capsys.readouterr().err
