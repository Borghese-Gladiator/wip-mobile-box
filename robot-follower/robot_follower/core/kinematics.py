"""Differential-drive kinematics — pure math, no hardware."""

from __future__ import annotations

import math

from .types import Twist2D, WheelCommand
from .math_utils import clamp


def wheel_to_twist(
    left_speed: float,
    right_speed: float,
    wheel_radius: float,
    wheelbase: float,
) -> Twist2D:
    """Convert wheel angular velocities to body twist."""
    v = (left_speed + right_speed) / 2.0 * wheel_radius
    omega = (right_speed - left_speed) / wheelbase * wheel_radius
    return Twist2D(linear=v, angular=omega)


def twist_to_wheels(
    linear: float,
    angular: float,
    wheel_radius: float,
    wheelbase: float,
    max_wheel_speed: float = 10.0,
) -> WheelCommand:
    """Convert body twist to wheel angular velocities."""
    left = (linear - angular * wheelbase / 2.0) / wheel_radius
    right = (linear + angular * wheelbase / 2.0) / wheel_radius
    left = clamp(left, -max_wheel_speed, max_wheel_speed)
    right = clamp(right, -max_wheel_speed, max_wheel_speed)
    return WheelCommand(left=left, right=right)
