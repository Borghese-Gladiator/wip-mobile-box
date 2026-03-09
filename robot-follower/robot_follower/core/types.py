"""Shared data types used across the entire robot stack."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto


@dataclass(frozen=True)
class Pose2D:
    """Planar robot pose."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class Twist2D:
    """Planar velocity."""
    linear: float = 0.0
    angular: float = 0.0


@dataclass(frozen=True)
class WheelCommand:
    """Target angular velocities for left/right wheels (rad/s)."""
    left: float = 0.0
    right: float = 0.0


@dataclass(frozen=True)
class WheelState:
    """Measured wheel state."""
    left_speed: float = 0.0   # rad/s
    right_speed: float = 0.0  # rad/s
    left_ticks: int = 0       # encoder ticks (cumulative)
    right_ticks: int = 0


@dataclass(frozen=True)
class IMUReading:
    """IMU measurement (gyro z-rate + accelerometer)."""
    gyro_z: float = 0.0    # rad/s
    accel_x: float = 0.0   # m/s^2
    accel_y: float = 0.0
    accel_z: float = 0.0
    timestamp: float = 0.0


@dataclass(frozen=True)
class TargetMeasurement:
    """What the robot knows about the target from BLE-like sensing."""
    signal_strength: float = 0.0   # 0..1 (1=close)
    bearing: float = 0.0           # radians, relative to robot heading
    distance_estimate: float = 0.0 # metres (from RSSI model)
    valid: bool = False
    stale: bool = False            # True if last update is old
    timestamp: float = 0.0


@dataclass(frozen=True)
class ObstacleMeasurement:
    """Obstacle proximity from range sensors."""
    front: float = 0.0       # 0=clear, 1=blocked
    front_left: float = 0.0
    front_right: float = 0.0

    @property
    def front_clear(self) -> bool:
        return self.front < 0.4

    @property
    def left_clear(self) -> bool:
        return self.front_left < 0.4

    @property
    def right_clear(self) -> bool:
        return self.front_right < 0.4

    @property
    def any_danger(self) -> bool:
        return self.front >= 0.4 or self.front_left >= 0.4 or self.front_right >= 0.4


class BehaviorState(Enum):
    FOLLOW_TARGET = auto()
    AVOID_OBSTACLE = auto()
    ESCAPE_TRAP = auto()
    SEARCH_TARGET = auto()
    TARGET_REACHED = auto()
    STOPPED = auto()


@dataclass
class RobotStateEstimate:
    """Fused state estimate from odometry + IMU."""
    pose: Pose2D = field(default_factory=Pose2D)
    twist: Twist2D = field(default_factory=Twist2D)
    heading_confidence: float = 1.0  # 0..1
    timestamp: float = 0.0


@dataclass(frozen=True)
class SafetyStatus:
    """Safety flags."""
    e_stop: bool = False
    stalled: bool = False
    sensor_fault: bool = False
    low_battery: bool = False
    watchdog_expired: bool = False
