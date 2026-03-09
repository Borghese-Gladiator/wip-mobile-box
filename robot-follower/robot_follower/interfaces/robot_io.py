"""Abstract robot I/O contract — implemented by sim or hardware backends."""

from __future__ import annotations

from abc import ABC, abstractmethod

from robot_follower.core.types import (
    Pose2D, Twist2D, WheelCommand, WheelState,
    IMUReading, TargetMeasurement, ObstacleMeasurement,
)


class RobotIO(ABC):
    """Unified sensor + actuator interface for the robot."""

    @abstractmethod
    def read_pose(self) -> Pose2D:
        """Read current pose estimate (may come from odometry/fusion)."""

    @abstractmethod
    def read_velocity(self) -> Twist2D:
        """Read current velocity estimate."""

    @abstractmethod
    def read_wheels(self) -> WheelState:
        """Read wheel encoder state."""

    @abstractmethod
    def read_imu(self) -> IMUReading | None:
        """Read IMU. Returns None if unavailable."""

    @abstractmethod
    def read_target(self) -> TargetMeasurement:
        """Read BLE target measurement."""

    @abstractmethod
    def read_obstacles(self) -> ObstacleMeasurement:
        """Read obstacle range sensors."""

    @abstractmethod
    def send_wheel_command(self, cmd: WheelCommand) -> None:
        """Apply wheel velocity command."""

    @abstractmethod
    def get_timestamp(self) -> float:
        """Current time in seconds."""
