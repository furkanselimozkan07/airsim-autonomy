# airsim-autonomy

Belirlenen noktalardan geçen uçuş görevleri, PID kontrolcü, engelden kaçınma ve YOLO ile nesne tespiti —
Microsoft AirSim (Cosys-AirSim uyumlu) için otonom drone araç seti.

Autonomous drone flight utilities for [AirSim](https://github.com/microsoft/AirSim): waypoint missions,
PID control, obstacle avoidance and YOLO object detection. Control and geometry logic is pure Python,
so it is fully testable **without a running simulator**.

## Features

- `PID` controller: output clamping, integral anti-windup, derivative on measurement (no setpoint kick)
- `WaypointMission`: ordered waypoints with per-waypoint speed and acceptance radius, progress tracking,
  path length (AirSim NED coordinates: z is down)
- `VelocityController`: position error → clamped 3-axis velocity command (per-axis PID, direction-preserving
  speed limit, vertical cap, smooth arrival slow-down)
- `VelocityController`: per-axis PID, speed clamp and arrival slow-down
- `DroneClient` protocol with `AirSimDrone` (real) and `FakeDrone` (tests)
- `MissionRunner`: takeoff, track waypoints, land; timeout/collision abort, `on_tick` hook
- `load_mission` / `parse_mission`: validated JSON missions (defaults, per-waypoint overrides, runner and controller tuning) with path-aware errors
- `airsim-mission` CLI: `validate` and `run` (`--fake` runs a built-in point-mass simulator, no AirSim needed)
- `TelemetryLogger`: per-tick CSV (position, velocity, target, command, waypoint, collision); `read_telemetry` / `summarize` for analysis; CLI `run --log flight.csv` and `stats flight.csv`
- Roadmap with upcoming features: see [ROADMAP.md](ROADMAP.md)

## Install

```bash
pip install -e ".[dev]"        # core + pytest
pip install -e ".[sim]"        # adds the airsim client when you want to fly
pytest
```

## Example

```python
from airsim_autonomy import PID, PIDConfig, Vec3, WaypointMission

mission = WaypointMission.from_tuples([(10, 0, -5), (10, 10, -5)], speed=3, acceptance_radius=1)
pid_x = PID(PIDConfig(kp=1.2, ki=0.05, kd=0.3, out_min=-5, out_max=5))

# or one controller for all three axes:
from airsim_autonomy import VelocityConfig, VelocityController
ctrl = VelocityController(VelocityConfig(kp=1.0, max_speed=5, arrival_radius=4))
cmd = ctrl.update(mission.current.position, Vec3(0, 0, 0), dt=0.05, speed_limit=mission.current.speed)

position = Vec3(0, 0, 0)
target = mission.update(position)
vx = pid_x.update(target.position.x, position.x, dt=0.05)
```

### Flying a mission (FakeDrone here; swap in `AirSimDrone()` for the simulator)

```python
from airsim_autonomy import FakeDrone, MissionRunner, RunnerConfig, WaypointMission

mission = WaypointMission.from_tuples([(10, 0, -3), (10, 10, -3)], speed=4)
result = MissionRunner(FakeDrone(), mission, config=RunnerConfig(dt=0.2)).run()
print(result.status, result.waypoints_reached, f"{result.path_length:.1f} m")
```

## Mission files and CLI

```bash
pip install -e .
airsim-mission validate examples/square.json
airsim-mission run examples/square.json --fake          # no simulator needed
airsim-mission run examples/square.json --vehicle Drone1 --timeout 90   # real AirSim
```

See [`examples/square.json`](examples/square.json) and the schema in `src/airsim_autonomy/loader.py`.
```bash
airsim-mission run examples/square.json --fake --log flight.csv   # record telemetry
airsim-mission stats flight.csv                                   # speed, path length, altitude, final error
```

Exit codes: 0 success, 1 mission failed (timeout/collision), 2 invalid mission file.

## Layout

```
src/airsim_autonomy/   pid.py, waypoints.py, ...
tests/                 pytest suite (no simulator required)
```

License: MIT

## Drone adapter

Control code talks to a small `DroneClient` protocol (`airsim_autonomy.drone`), so it runs against
either the real simulator or a fake:

```python
from airsim_autonomy import AirSimDrone, FakeDrone, Vec3

drone = FakeDrone()          # or AirSimDrone("Drone1")  (needs `pip install airsim`)
drone.connect(); drone.takeoff(5.0)
drone.move_by_velocity(Vec3(2, 0, 0), 0.5)
print(drone.get_state().position)
drone.land(); drone.disconnect()
```
