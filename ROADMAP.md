## Vision

A small, well-tested toolkit for autonomous drone flight in Microsoft AirSim (Cosys-AirSim compatible).
All math and control logic lives in pure, simulator-independent functions so it can be unit-tested
without a running simulator; a thin adapter layer talks to the `airsim` client (mockable in tests).
End goal: waypoint missions with PID control, obstacle avoidance from depth/lidar, camera capture,
YOLO object detection and mission logging/plotting.

## Backlog

- [x] Project layout, PID controller (anti-windup) and waypoint mission model with tests
- [ ] Velocity controller: turn position error into clamped 3-axis velocity commands (PID per axis)
- [ ] `DroneClient` adapter protocol + `AirSimDrone` implementation with a fake client for tests
- [ ] Mission runner: fly a `WaypointMission` through the adapter (takeoff, loop, land)
- [ ] CLI entry point `airsim-mission` loading missions from JSON/YAML
- [ ] Mission loader/validator for JSON files + example missions
- [ ] Telemetry logger (CSV) recording position, velocity, target each tick
- [ ] Plotting of logged trajectories (matplotlib, optional extra)
- [ ] Geometry helpers: yaw-to-target, NED/ENU conversions, path smoothing
- [ ] Speed profile: trapezoidal acceleration limiting along legs
- [ ] Obstacle avoidance: depth-image sector analysis (pure numpy-free function)
- [ ] Reactive avoidance steering blended into velocity commands
- [ ] Lidar point-cloud based avoidance
- [ ] Camera capture helper (RGB/depth) saving frames with metadata
- [ ] YOLO detector wrapper (ultralytics, optional extra) with fake detector for tests
- [ ] Detection-driven behaviours: hover/orbit over detected object
- [ ] Geofence and battery/timeout failsafes
- [ ] Search patterns: lawnmower and spiral coverage waypoint generators
- [ ] Return-to-home failsafe
- [ ] GitHub Actions CI running pytest
- [ ] Documentation: architecture overview and tutorial

## Changelog

- 2026-10-27: Initial project layout (`src/airsim_autonomy`, tests, pyproject, MIT license), PID controller with anti-windup and derivative-on-measurement, and waypoint mission model; 15 pytest tests.
