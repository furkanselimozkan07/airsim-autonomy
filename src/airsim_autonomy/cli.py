"""Command line interface: ``airsim-mission``.

    airsim-mission validate examples/square.json
    airsim-mission run examples/square.json --fake        # no simulator needed
    airsim-mission run examples/square.json --vehicle Drone1
"""

from __future__ import annotations

import argparse
import dataclasses
import sys
from typing import List, Optional

from .drone import AirSimDrone, DroneClient, FakeDrone
from .loader import MissionFileError, MissionSpec, load_mission
from .runner import MissionRunner
from .velocity import VelocityController


def _summary(spec: MissionSpec) -> str:
    m = spec.mission
    return (f"{spec.name}: {len(m.waypoints)} waypoints, "
            f"path {m.total_length():.1f} m, timeout {spec.runner.timeout:g}s")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="airsim-mission",
                                description="Validate and fly AirSim waypoint missions.")
    sub = p.add_subparsers(dest="command", required=True)
    v = sub.add_parser("validate", help="check a mission file and print a summary")
    v.add_argument("mission", help="path to mission JSON")
    r = sub.add_parser("run", help="fly a mission")
    r.add_argument("mission", help="path to mission JSON")
    r.add_argument("--fake", action="store_true", help="use the built-in point-mass simulator")
    r.add_argument("--vehicle", default="", help="AirSim vehicle name (default: first vehicle)")
    r.add_argument("--ip", default="", help="AirSim host IP (default: localhost)")
    r.add_argument("--timeout", type=float, help="override mission timeout (seconds)")
    return p


def main(argv: Optional[List[str]] = None, drone: Optional[DroneClient] = None) -> int:
    """Entry point. ``drone`` can be injected for testing. Returns the exit code."""
    args = build_parser().parse_args(argv)
    try:
        spec = load_mission(args.mission)
    except MissionFileError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(_summary(spec))
    if args.command == "validate":
        return 0

    runner_cfg = spec.runner
    if args.timeout is not None:
        try:
            runner_cfg = dataclasses.replace(runner_cfg, timeout=args.timeout)
        except ValueError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2
    if drone is None:
        drone = FakeDrone() if args.fake else AirSimDrone(args.vehicle, ip=args.ip)
    runner = MissionRunner(drone, spec.mission, VelocityController(spec.controller), runner_cfg)
    result = runner.run()
    print(f"{result.status.value}: reached {result.waypoints_reached}/{len(spec.mission.waypoints)} "
          f"waypoints in {result.elapsed:.1f}s, flew {result.path_length:.1f} m"
          + (f" ({result.message})" if result.message else ""))
    return 0 if result.success else 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
