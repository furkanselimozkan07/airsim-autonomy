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

position = Vec3(0, 0, 0)
target = mission.update(position)
vx = pid_x.update(target.position.x, position.x, dt=0.05)
```

## Layout

```
src/airsim_autonomy/   pid.py, waypoints.py, ...
tests/                 pytest suite (no simulator required)
```

License: MIT
