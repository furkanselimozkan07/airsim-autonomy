"""Mission runner: fly a :class:`WaypointMission` through a :class:`DroneClient`.

The runner owns the control loop (takeoff -> track waypoints -> land) and
nothing else: geometry lives in ``waypoints``, control in ``velocity`` and
vehicle I/O in ``drone``. Time is counted in control ticks (``tick * dt``), so
timeouts are deterministic and tests need no real sleeping.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from .drone import DroneClient, DroneState
from .velocity import VelocityController
from .waypoints import Vec3, WaypointMission


class MissionStatus(enum.Enum):
    COMPLETED = "completed"
    TIMEOUT = "timeout"
    COLLISION = "collision"
    ABORTED = "aborted"


@dataclass(frozen=True)
class RunnerConfig:
    """Loop settings: ``dt`` seconds per tick, ``timeout`` simulated seconds."""

    dt: float = 0.2
    takeoff_altitude: float = 3.0
    timeout: float = 300.0
    land_on_finish: bool = True
    abort_on_collision: bool = True

    def __post_init__(self) -> None:
        if self.dt <= 0:
            raise ValueError("dt must be positive")
        if self.timeout <= 0:
            raise ValueError("timeout must be positive")
        if self.takeoff_altitude <= 0:
            raise ValueError("takeoff_altitude must be positive")


@dataclass(frozen=True)
class TickInfo:
    """Passed to the ``on_tick`` callback (e.g. a telemetry logger)."""

    tick: int
    time: float
    state: DroneState
    target: Vec3
    command: Vec3
    waypoint_index: int


@dataclass
class MissionResult:
    status: MissionStatus
    ticks: int
    elapsed: float
    waypoints_reached: int
    final_position: Vec3
    path_length: float = 0.0
    message: str = ""
    history: List[Vec3] = field(default_factory=list, repr=False)

    @property
    def success(self) -> bool:
        return self.status is MissionStatus.COMPLETED


class MissionRunner:
    """Drive ``drone`` along ``mission`` using ``controller``."""

    def __init__(self, drone: DroneClient, mission: WaypointMission,
                 controller: Optional[VelocityController] = None,
                 config: RunnerConfig = RunnerConfig(),
                 on_tick: Optional[Callable[[TickInfo], None]] = None) -> None:
        self.drone = drone
        self.mission = mission
        self.controller = controller or VelocityController()
        self.config = config
        self.on_tick = on_tick

    def run(self) -> MissionResult:
        """Connect, take off, fly every waypoint, land. Always lands/disconnects."""
        cfg = self.config
        self.controller.reset()
        self.drone.connect()
        try:
            self.drone.takeoff(cfg.takeoff_altitude)
            result = self._fly()
            if cfg.land_on_finish or result.status is not MissionStatus.COMPLETED:
                self.drone.land()
            return result
        except KeyboardInterrupt:
            self.drone.land()
            raise
        finally:
            self.drone.disconnect()

    def _fly(self) -> MissionResult:
        cfg, mission = self.config, self.mission
        state = self.drone.get_state()
        history = [state.position]
        length, tick = 0.0, 0
        max_ticks = int(cfg.timeout / cfg.dt)
        status, message = MissionStatus.COMPLETED, ""

        while True:
            target_wp = mission.update(state.position)
            if target_wp is None:
                break
            if cfg.abort_on_collision and state.collided:
                status, message = MissionStatus.COLLISION, "collision detected"
                break
            if tick >= max_ticks:
                status = MissionStatus.TIMEOUT
                message = f"timeout after {cfg.timeout:g}s"
                break
            cmd = self.controller.update(target_wp.position, state.position,
                                         cfg.dt, speed_limit=target_wp.speed)
            self.drone.move_by_velocity(cmd, cfg.dt)
            tick += 1
            new_state = self.drone.get_state()
            length += state.position.distance_to(new_state.position)
            state = new_state
            history.append(state.position)
            if self.on_tick:
                self.on_tick(TickInfo(tick, tick * cfg.dt, state, target_wp.position,
                                      cmd, mission.index))

        return MissionResult(status, tick, tick * cfg.dt, mission.index,
                             state.position, length, message, history)
