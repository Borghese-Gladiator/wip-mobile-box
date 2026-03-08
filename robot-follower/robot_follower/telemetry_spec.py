"""Telemetry specification — defines what the robot would send over BLE.

This module documents the telemetry schema only. No transport is implemented.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Optional


@dataclass
class RobotTelemetry:
    """Schema for robot BLE telemetry packet."""

    # Core state
    timestamp: float
    position_x: float
    position_y: float
    heading: float
    linear_velocity: float
    angular_velocity: float

    # Motor state
    left_pwm: float
    right_pwm: float
    left_wheel_speed: float
    right_wheel_speed: float

    # Follow/task state
    target_signal_strength: float
    target_bearing: float
    obstacle_front: float
    obstacle_left: float
    obstacle_right: float
    controller_mode: str  # "follow", "avoid", "escape", "reached"

    # Diagnostics
    packet_counter: int = 0
    battery_pct: float = 100.0  # simulated

    def to_dict(self) -> dict:
        return asdict(self)
