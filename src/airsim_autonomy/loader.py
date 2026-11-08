"""Load and validate missions from JSON.

Schema (all keys except ``waypoints`` are optional)::

    {
      "name": "square",
      "defaults": {"speed": 3.0, "acceptance_radius": 1.0},
      "runner": {"dt": 0.2, "takeoff_altitude": 3.0, "timeout": 120},
      "controller": {"kp": 1.0, "max_speed": 5.0, "arrival_radius": 4.0},
      "waypoints": [
        [10, 0, -5],
        {"position": [10, 10, -5], "speed": 2.0, "acceptance_radius": 0.5}
      ]
    }

A waypoint is either ``[x, y, z]`` or an object with ``position`` (NED metres).
Every problem is reported as :class:`MissionFileError` with the JSON path.
"""

from __future__ import annotations

import dataclasses
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Union

from .runner import RunnerConfig
from .velocity import VelocityConfig
from .waypoints import Vec3, Waypoint, WaypointMission


class MissionFileError(ValueError):
    """Raised for malformed or invalid mission files."""


@dataclass
class MissionSpec:
    """A parsed mission file: mission plus optional runner/controller tuning."""

    name: str
    mission: WaypointMission
    runner: RunnerConfig
    controller: VelocityConfig


def _number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionFileError(f"{where}: expected a number, got {value!r}")
    if not math.isfinite(value):
        raise MissionFileError(f"{where}: number must be finite")
    return float(value)


def _position(value: Any, where: str) -> Vec3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise MissionFileError(f"{where}: expected [x, y, z], got {value!r}")
    return Vec3(*(_number(v, f"{where}[{i}]") for i, v in enumerate(value)))


def _dataclass_from(cls: type, data: Any, where: str):
    """Build a config dataclass from a dict, rejecting unknown keys."""
    if data is None:
        return cls()
    if not isinstance(data, dict):
        raise MissionFileError(f"{where}: expected an object")
    allowed = {f.name for f in dataclasses.fields(cls)}
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise MissionFileError(
            f"{where}: unknown key(s) {', '.join(unknown)} (allowed: {', '.join(sorted(allowed))})")
    try:
        return cls(**data)
    except (TypeError, ValueError) as exc:
        raise MissionFileError(f"{where}: {exc}") from exc


def parse_mission(data: Any) -> MissionSpec:
    """Validate an already-decoded JSON document."""
    if not isinstance(data, dict):
        raise MissionFileError("top level must be an object")
    defaults = data.get("defaults") or {}
    if not isinstance(defaults, dict):
        raise MissionFileError("defaults: expected an object")
    bad = sorted(set(defaults) - {"speed", "acceptance_radius"})
    if bad:
        raise MissionFileError(f"defaults: unknown key(s) {', '.join(bad)}")
    def_speed = _number(defaults.get("speed", 3.0), "defaults.speed")
    def_radius = _number(defaults.get("acceptance_radius", 1.0), "defaults.acceptance_radius")

    raw = data.get("waypoints")
    if not isinstance(raw, list) or not raw:
        raise MissionFileError("waypoints: expected a non-empty list")
    waypoints = []
    for i, item in enumerate(raw):
        where = f"waypoints[{i}]"
        if isinstance(item, dict):
            extra = sorted(set(item) - {"position", "speed", "acceptance_radius"})
            if extra:
                raise MissionFileError(f"{where}: unknown key(s) {', '.join(extra)}")
            if "position" not in item:
                raise MissionFileError(f"{where}: missing 'position'")
            pos = _position(item["position"], f"{where}.position")
            speed = _number(item.get("speed", def_speed), f"{where}.speed")
            radius = _number(item.get("acceptance_radius", def_radius),
                             f"{where}.acceptance_radius")
        else:
            pos, speed, radius = _position(item, where), def_speed, def_radius
        try:
            waypoints.append(Waypoint(pos, speed, radius))
        except ValueError as exc:
            raise MissionFileError(f"{where}: {exc}") from exc

    name = data.get("name", "mission")
    if not isinstance(name, str):
        raise MissionFileError("name: expected a string")
    return MissionSpec(
        name=name,
        mission=WaypointMission(waypoints),
        runner=_dataclass_from(RunnerConfig, data.get("runner"), "runner"),
        controller=_dataclass_from(VelocityConfig, data.get("controller"), "controller"),
    )


def load_mission(path: Union[str, Path]) -> MissionSpec:
    """Read and validate a mission JSON file."""
    p = Path(path)
    try:
        text = p.read_text(encoding="utf-8")
    except OSError as exc:
        raise MissionFileError(f"cannot read {p}: {exc.strerror or exc}") from exc
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise MissionFileError(f"{p}: invalid JSON: {exc}") from exc
    return parse_mission(data)
