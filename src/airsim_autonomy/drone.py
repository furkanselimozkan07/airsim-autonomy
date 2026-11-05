"""Drone adapter layer: a small protocol, the AirSim implementation and a fake.

Control and mission code depends only on :class:`DroneClient`, so it can be
tested with :class:`FakeDrone` (simple point-mass kinematics) and flown for real
with :class:`AirSimDrone`, which wraps the ``airsim`` package. The ``airsim``
import is lazy, so this module imports fine without the simulator installed.

All vectors use AirSim's NED convention (z is down).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Protocol, runtime_checkable

from .waypoints import Vec3


@dataclass(frozen=True)
class DroneState:
    """Snapshot of the vehicle: position/velocity (NED) and collision flag."""

    position: Vec3
    velocity: Vec3 = Vec3(0.0, 0.0, 0.0)
    collided: bool = False


@runtime_checkable
class DroneClient(Protocol):
    """Minimal vehicle interface used by the mission runner."""

    def connect(self) -> None: ...
    def takeoff(self, altitude: float = 3.0) -> None: ...
    def land(self) -> None: ...
    def get_state(self) -> DroneState: ...
    def move_by_velocity(self, velocity: Vec3, duration: float) -> None: ...
    def disconnect(self) -> None: ...


class AirSimDrone:
    """:class:`DroneClient` backed by ``airsim.MultirotorClient``.

    Pass ``client`` (any object with the same methods) to inject a mock; if
    omitted, the real ``airsim`` package is imported and connected on demand.
    """

    def __init__(self, vehicle_name: str = "", client: Optional[Any] = None,
                 ip: str = "") -> None:
        self.vehicle_name = vehicle_name
        self.ip = ip
        self._client = client
        self._airsim: Any = None

    def _api(self) -> Any:
        if self._airsim is None:
            try:
                import airsim
            except ImportError as exc:  # pragma: no cover - needs missing dep
                raise RuntimeError(
                    "the 'airsim' package is required: pip install airsim") from exc
            self._airsim = airsim
        return self._airsim

    @property
    def client(self) -> Any:
        if self._client is None:
            self._client = self._api().MultirotorClient(ip=self.ip)
        return self._client

    def connect(self) -> None:
        c = self.client
        c.confirmConnection()
        c.enableApiControl(True, self.vehicle_name)
        c.armDisarm(True, self.vehicle_name)

    def takeoff(self, altitude: float = 3.0) -> None:
        if altitude <= 0:
            raise ValueError("altitude must be positive")
        self.client.takeoffAsync(vehicle_name=self.vehicle_name).join()
        # NED: climbing means z becomes more negative.
        self.client.moveToZAsync(-altitude, 2.0, vehicle_name=self.vehicle_name).join()

    def land(self) -> None:
        self.client.landAsync(vehicle_name=self.vehicle_name).join()

    def get_state(self) -> DroneState:
        c = self.client
        k = c.getMultirotorState(self.vehicle_name).kinematics_estimated
        collided = bool(c.simGetCollisionInfo(self.vehicle_name).has_collided)
        return DroneState(
            Vec3(k.position.x_val, k.position.y_val, k.position.z_val),
            Vec3(k.linear_velocity.x_val, k.linear_velocity.y_val, k.linear_velocity.z_val),
            collided)

    def move_by_velocity(self, velocity: Vec3, duration: float) -> None:
        if duration <= 0:
            raise ValueError("duration must be positive")
        self.client.moveByVelocityAsync(
            velocity.x, velocity.y, velocity.z, duration,
            vehicle_name=self.vehicle_name).join()

    def disconnect(self) -> None:
        c = self.client
        c.armDisarm(False, self.vehicle_name)
        c.enableApiControl(False, self.vehicle_name)


class FakeDrone:
    """Point-mass drone for tests: integrates commanded velocity exactly."""

    def __init__(self, position: Vec3 = Vec3(0.0, 0.0, 0.0)) -> None:
        self.position = position
        self.velocity = Vec3(0.0, 0.0, 0.0)
        self.connected = False
        self.flying = False
        self.time = 0.0
        self.log: list = []

    def connect(self) -> None:
        self.connected = True
        self.log.append("connect")

    def _require_connected(self) -> None:
        if not self.connected:
            raise RuntimeError("drone is not connected")

    def takeoff(self, altitude: float = 3.0) -> None:
        self._require_connected()
        if altitude <= 0:
            raise ValueError("altitude must be positive")
        self.flying = True
        self.position = Vec3(self.position.x, self.position.y, -altitude)
        self.log.append("takeoff")

    def land(self) -> None:
        self._require_connected()
        self.flying = False
        self.velocity = Vec3(0.0, 0.0, 0.0)
        self.position = Vec3(self.position.x, self.position.y, 0.0)
        self.log.append("land")

    def get_state(self) -> DroneState:
        self._require_connected()
        return DroneState(self.position, self.velocity, False)

    def move_by_velocity(self, velocity: Vec3, duration: float) -> None:
        self._require_connected()
        if not self.flying:
            raise RuntimeError("drone has not taken off")
        if duration <= 0:
            raise ValueError("duration must be positive")
        p = self.position
        self.position = Vec3(p.x + velocity.x * duration, p.y + velocity.y * duration,
                             p.z + velocity.z * duration)
        self.velocity = velocity
        self.time += duration

    def disconnect(self) -> None:
        self.connected = False
        self.log.append("disconnect")
