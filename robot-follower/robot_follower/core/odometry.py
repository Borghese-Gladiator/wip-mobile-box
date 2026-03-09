"""Differential-drive odometry from encoder ticks / wheel speeds."""

from __future__ import annotations

import math

from .types import Pose2D
from .math_utils import normalize_angle


class Odometry:
    """Incremental dead-reckoning from left/right wheel speeds."""

    def __init__(self, wheel_radius: float, wheelbase: float):
        self.wheel_radius = wheel_radius
        self.wheelbase = wheelbase
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    def update(self, left_speed: float, right_speed: float, dt: float) -> Pose2D:
        """Integrate one step. Returns updated pose."""
        v_left = left_speed * self.wheel_radius
        v_right = right_speed * self.wheel_radius
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheelbase

        self._x += v * math.cos(self._yaw) * dt
        self._y += v * math.sin(self._yaw) * dt
        self._yaw = normalize_angle(self._yaw + omega * dt)

        return self.pose

    @property
    def pose(self) -> Pose2D:
        return Pose2D(x=self._x, y=self._y, yaw=self._yaw)

    def reset(self, pose: Pose2D | None = None) -> None:
        if pose is None:
            self._x = self._y = self._yaw = 0.0
        else:
            self._x, self._y, self._yaw = pose.x, pose.y, pose.yaw
