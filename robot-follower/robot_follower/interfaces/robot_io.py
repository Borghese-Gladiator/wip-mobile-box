"""Abstract robot I/O contract — implemented by sim or hardware backends."""

from __future__ import annotations

from abc import ABC, abstractmethod

from robot_follower.core.types import (
    Pose2D, Twist2D, WheelCommand, WheelState,
    IMUReading, TargetMeasurement, ObstacleMeasurement,
)


class RobotIO(ABC):
    """Unified sensor + actuator interface for the robot.

    Sensor reads that depend on robot pose (target, obstacles) accept an
    estimated pose so the controller never sees ground truth.
    """

    @abstractmethod
    def read_wheels(self) -> WheelState:
        """Read wheel encoder state."""

    @abstractmethod
    def read_imu(self) -> IMUReading | None:
        """Read IMU. Returns None if unavailable."""

    @abstractmethod
    def read_target(self, estimated_pose: Pose2D) -> TargetMeasurement:
        """Read BLE target measurement relative to the estimated pose."""

    @abstractmethod
    def read_obstacles(self, estimated_pose: Pose2D) -> ObstacleMeasurement:
        """Read obstacle range sensors from the estimated pose."""

    @abstractmethod
    def send_wheel_command(self, cmd: WheelCommand) -> None:
        """Apply wheel velocity command."""

    @abstractmethod
    def get_timestamp(self) -> float:
        """Current time in seconds."""
