"""Plot logged flights (requires the optional ``plot`` extra: ``pip install .[plot]``).

The data preparation (:func:`flight_series`, :func:`waypoint_markers`) is pure
Python and works without matplotlib; only :func:`plot_flight` imports it, lazily,
using the headless ``Agg`` backend so it runs on servers and in CI.

Telemetry is stored in NED (z down). Plots use the friendlier east/north/up
convention: east = ``y``, north = ``x``, altitude = ``-z``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple, Union

from .telemetry import TelemetryRow


@dataclass(frozen=True)
class FlightSeries:
    """Column-oriented view of a telemetry log, ready for plotting."""
    time: List[float]            # seconds since the first sample
    east: List[float]
    north: List[float]
    altitude: List[float]        # metres above the start (positive up)
    speed: List[float]           # m/s
    error: List[float]           # distance to the active target, m
    collisions: List[int]        # indices of samples where a collision was flagged


def flight_series(rows: Sequence[TelemetryRow]) -> FlightSeries:
    """Convert telemetry rows into plot-ready columns (NED -> east/north/up)."""
    if not rows:
        raise ValueError("no telemetry samples to plot")
    t0 = rows[0].time
    return FlightSeries(
        time=[r.time - t0 for r in rows],
        east=[r.position[1] for r in rows],
        north=[r.position[0] for r in rows],
        altitude=[-r.position[2] for r in rows],
        speed=[r.speed for r in rows],
        error=[r.distance_to_target for r in rows],
        collisions=[i for i, r in enumerate(rows) if r.collided],
    )


def waypoint_markers(rows: Sequence[TelemetryRow]) -> List[Tuple[float, float, float]]:
    """One ``(east, north, altitude)`` marker per distinct waypoint index, in order.

    The target of the first tick that pursued each waypoint index is used.
    """
    seen = set()
    out: List[Tuple[float, float, float]] = []
    for r in rows:
        if r.waypoint in seen:
            continue
        seen.add(r.waypoint)
        out.append((r.target[1], r.target[0], -r.target[2]))
    return out


def plot_flight(rows: Sequence[TelemetryRow], out: Union[str, Path], title: str = "Flight",
                dpi: int = 130) -> Path:
    """Render a 3-panel summary PNG/SVG/PDF (by extension) and return its path.

    Panels: top-down trajectory with waypoints, altitude over time, and speed
    plus distance-to-target over time.
    """
    series = flight_series(rows)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover - exercised only without matplotlib
        raise RuntimeError("plotting needs matplotlib: pip install 'airsim-autonomy[plot]'") from exc

    fig = plt.figure(figsize=(11, 6), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, width_ratios=[1.2, 1])
    ax_map = fig.add_subplot(grid[:, 0])
    ax_alt = fig.add_subplot(grid[0, 1])
    ax_spd = fig.add_subplot(grid[1, 1], sharex=ax_alt)

    ax_map.plot(series.east, series.north, color="tab:blue", lw=1.5, label="path")
    ax_map.plot(series.east[0], series.north[0], "go", label="start")
    ax_map.plot(series.east[-1], series.north[-1], "rs", label="end")
    marks = waypoint_markers(rows)
    if marks:
        ax_map.scatter([m[0] for m in marks], [m[1] for m in marks], marker="*", s=120,
                       color="gold", edgecolor="black", zorder=3, label="waypoints")
        for i, m in enumerate(marks):
            ax_map.annotate(str(i), (m[0], m[1]), textcoords="offset points", xytext=(5, 5))
    if series.collisions:
        ax_map.scatter([series.east[i] for i in series.collisions],
                       [series.north[i] for i in series.collisions],
                       marker="x", color="red", zorder=4, label="collision")
    ax_map.set_xlabel("east (m)")
    ax_map.set_ylabel("north (m)")
    ax_map.set_aspect("equal", adjustable="datalim")
    ax_map.grid(alpha=0.3)
    ax_map.legend(loc="best", fontsize=8)
    ax_map.set_title("Top-down trajectory")

    ax_alt.plot(series.time, series.altitude, color="tab:green")
    ax_alt.set_ylabel("altitude (m)")
    ax_alt.grid(alpha=0.3)
    ax_alt.set_title("Altitude")

    ax_spd.plot(series.time, series.speed, color="tab:orange", label="speed (m/s)")
    ax_spd.plot(series.time, series.error, color="tab:purple", ls="--", label="distance to target (m)")
    ax_spd.set_xlabel("time (s)")
    ax_spd.grid(alpha=0.3)
    ax_spd.legend(fontsize=8)

    fig.suptitle(title)
    path = Path(out)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
    return path
