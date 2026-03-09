"""Telemetry specification — defines what the robot would send over BLE."""

from __future__ import annotations

from dataclasses import dataclass, asdict


@dataclass
class RobotTelemetry:
    """Schema for robot BLE telemetry packet."""
    timestamp: float = 0.0
    position_x: float = 0.0
    position_y: float = 0.0
    heading: float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    left_wheel_cmd: float = 0.0
    right_wheel_cmd: float = 0.0
    left_wheel_speed: float = 0.0
    right_wheel_speed: float = 0.0
    target_signal_strength: float = 0.0
    target_bearing: float = 0.0
    target_confidence: float = 0.0
    obstacle_front: float = 0.0
    obstacle_left: float = 0.0
    obstacle_right: float = 0.0
    controller_mode: str = "follow"
    battery_pct: float = 100.0
    packet_counter: int = 0

    def to_dict(self) -> dict:
        return asdict(self)
