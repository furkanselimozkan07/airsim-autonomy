"""CSV telemetry logging for missions.

:class:`TelemetryLogger` is a context manager whose :meth:`on_tick` method can
be passed straight to :class:`~airsim_autonomy.runner.MissionRunner`. Each tick
becomes one CSV row (NED coordinates). :func:`read_telemetry` loads a log back
into typed rows and :func:`summarize` computes simple statistics, so plotting
and analysis code never has to re-parse strings by hand.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import IO, Iterable, List, Optional, Union

from .runner import TickInfo

FIELDS = ["tick", "time", "x", "y", "z", "vx", "vy", "vz",
          "target_x", "target_y", "target_z", "cmd_vx", "cmd_vy", "cmd_vz",
          "waypoint", "collided"]


@dataclass(frozen=True)
class TelemetryRow:
    tick: int
    time: float
    position: tuple
    velocity: tuple
    target: tuple
    command: tuple
    waypoint: int
    collided: bool

    @property
    def speed(self) -> float:
        return math.sqrt(sum(v * v for v in self.velocity))

    @property
    def distance_to_target(self) -> float:
        return math.dist(self.position, self.target)


def tick_to_row(info: TickInfo) -> List[object]:
    """Flatten a :class:`TickInfo` into a CSV row matching :data:`FIELDS`."""
    s, t, c = info.state, info.target, info.command
    return [info.tick, f"{info.time:.3f}",
            s.position.x, s.position.y, s.position.z,
            s.velocity.x, s.velocity.y, s.velocity.z,
            t.x, t.y, t.z, c.x, c.y, c.z,
            info.waypoint_index, int(s.collided)]


class TelemetryLogger:
    """Write one CSV row per control tick.

    ``dest`` is a path (file created, parent dirs included) or an open text
    file object (left open on exit). Use as a context manager or call
    :meth:`close` explicitly.
    """

    def __init__(self, dest: Union[str, Path, IO[str]]) -> None:
        self._owns = isinstance(dest, (str, Path))
        if self._owns:
            path = Path(dest)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._fh: IO[str] = open(path, "w", newline="", encoding="utf-8")
        else:
            self._fh = dest  # type: ignore[assignment]
        self._writer = csv.writer(self._fh)
        self._writer.writerow(FIELDS)
        self.rows_written = 0

    def on_tick(self, info: TickInfo) -> None:
        self._writer.writerow(tick_to_row(info))
        self.rows_written += 1

    __call__ = on_tick

    def close(self) -> None:
        if self._owns and not self._fh.closed:
            self._fh.close()
        elif not self._fh.closed:
            self._fh.flush()

    def __enter__(self) -> "TelemetryLogger":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def read_telemetry(source: Union[str, Path, Iterable[str]]) -> List[TelemetryRow]:
    """Load a telemetry CSV (path or iterable of lines) into rows."""
    if isinstance(source, (str, Path)):
        with open(source, newline="", encoding="utf-8") as fh:
            return read_telemetry(fh)
    reader = csv.DictReader(source)
    missing = set(FIELDS) - set(reader.fieldnames or [])
    if missing:
        raise ValueError(f"not a telemetry file, missing columns: {sorted(missing)}")
    rows = []
    for r in reader:
        def v(prefix: str, names: Iterable[str]) -> tuple:
            return tuple(float(r[prefix + n]) for n in names)
        rows.append(TelemetryRow(
            tick=int(r["tick"]), time=float(r["time"]),
            position=(float(r["x"]), float(r["y"]), float(r["z"])),
            velocity=(float(r["vx"]), float(r["vy"]), float(r["vz"])),
            target=v("target_", "xyz"), command=v("cmd_v", "xyz"),
            waypoint=int(r["waypoint"]), collided=r["collided"] == "1"))
    return rows


@dataclass(frozen=True)
class TelemetrySummary:
    samples: int
    duration: float
    path_length: float
    max_speed: float
    mean_speed: float
    min_altitude: float  # most negative z = highest point (NED)
    max_altitude: float
    final_error: float   # distance to the active target at the last tick
    collisions: int


def summarize(rows: List[TelemetryRow]) -> Optional[TelemetrySummary]:
    """Aggregate statistics, or ``None`` for an empty log."""
    if not rows:
        return None
    length = sum(math.dist(a.position, b.position) for a, b in zip(rows, rows[1:]))
    speeds = [r.speed for r in rows]
    zs = [r.position[2] for r in rows]
    return TelemetrySummary(
        samples=len(rows), duration=rows[-1].time - rows[0].time, path_length=length,
        max_speed=max(speeds), mean_speed=sum(speeds) / len(speeds),
        min_altitude=min(zs), max_altitude=max(zs),
        final_error=rows[-1].distance_to_target,
        collisions=sum(r.collided for r in rows))
