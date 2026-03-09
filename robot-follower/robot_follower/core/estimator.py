"""Lightweight state estimator: fuses wheel odometry + IMU heading."""

from __future__ import annotations

from .types import Pose2D, Twist2D, RobotStateEstimate, IMUReading
from .odometry import Odometry
from .math_utils import normalize_angle
from .filters import ExponentialMovingAverage


class StateEstimator:
    """Fuses wheel odometry with gyro for heading correction."""

    def __init__(self, wheel_radius: float, wheelbase: float, gyro_weight: float = 0.3):
        self._odom = Odometry(wheel_radius, wheelbase)
        self._gyro_weight = gyro_weight
        self._gyro_yaw = 0.0
        self._fused_yaw = 0.0
        self._linear_filter = ExponentialMovingAverage(alpha=0.5)

    def update(
        self,
        left_speed: float,
        right_speed: float,
        imu: IMUReading | None,
        dt: float,
        timestamp: float,
    ) -> RobotStateEstimate:
        """Produce a fused state estimate."""
        odom_pose = self._odom.update(left_speed, right_speed, dt)

        if imu is not None:
            self._gyro_yaw = normalize_angle(self._gyro_yaw + imu.gyro_z * dt)
            # Complement filter: blend odometry yaw with gyro yaw
            self._fused_yaw = normalize_angle(
                (1.0 - self._gyro_weight) * odom_pose.yaw
                + self._gyro_weight * self._gyro_yaw
            )
            confidence = 0.9
        else:
            self._fused_yaw = odom_pose.yaw
            confidence = 0.6

        wheel_radius = self._odom.wheel_radius
        v = (left_speed + right_speed) / 2.0 * wheel_radius
        omega = (right_speed - left_speed) / self._odom.wheelbase * wheel_radius
        lin = self._linear_filter.update(v)

        pose = Pose2D(x=odom_pose.x, y=odom_pose.y, yaw=self._fused_yaw)
        twist = Twist2D(linear=lin, angular=omega)
        return RobotStateEstimate(
            pose=pose, twist=twist,
            heading_confidence=confidence, timestamp=timestamp,
        )

    def reset(self, pose: Pose2D | None = None) -> None:
        self._odom.reset(pose)
        self._gyro_yaw = pose.yaw if pose else 0.0
        self._fused_yaw = self._gyro_yaw
        self._linear_filter.reset()
