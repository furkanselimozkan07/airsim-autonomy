"""A small, simulator-independent PID controller.

Features: output clamping, integral anti-windup (clamping), and a derivative
computed on the measurement to avoid "derivative kick" on setpoint changes.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class PIDConfig:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    out_min: float = float("-inf")
    out_max: float = float("inf")

    def __post_init__(self) -> None:
        if self.out_min > self.out_max:
            raise ValueError("out_min must be <= out_max")


class PID:
    """Discrete PID controller. Call :meth:`update` once per control tick."""

    def __init__(self, config: PIDConfig) -> None:
        self.config = config
        self._integral = 0.0
        self._prev_measurement: Optional[float] = None

    def reset(self) -> None:
        """Clear the integral term and derivative history."""
        self._integral = 0.0
        self._prev_measurement = None

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        """Compute the control output for one time step of length ``dt`` seconds."""
        if dt <= 0:
            raise ValueError("dt must be positive")
        c = self.config
        error = setpoint - measurement

        derivative = 0.0
        if self._prev_measurement is not None:
            # d(error)/dt == -d(measurement)/dt for a constant setpoint.
            derivative = -(measurement - self._prev_measurement) / dt
        self._prev_measurement = measurement

        candidate_integral = self._integral + error * dt
        unclamped = c.kp * error + c.ki * candidate_integral + c.kd * derivative
        output = min(max(unclamped, c.out_min), c.out_max)

        # Anti-windup: only accumulate when not saturated, or when the error
        # would pull the output back toward the allowed range.
        saturated_high = unclamped > c.out_max and error > 0
        saturated_low = unclamped < c.out_min and error < 0
        if not (saturated_high or saturated_low):
            self._integral = candidate_integral
        return output
