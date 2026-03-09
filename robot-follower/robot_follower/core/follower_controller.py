"""Follow controller: bearing + distance error → forward/turn demands."""

from __future__ import annotations

import math

from .types import TargetMeasurement, Twist2D
from .math_utils import clamp


class FollowerController:
    """PD distance-error controller with bearing-proportional steering.

    Maintains a desired follow distance behind the target. Uses derivative
    damping to prevent overshoot and a dead zone to allow the robot to stop
    when at the right distance.
    """

    def __init__(
        self,
        base_speed: float = 3.0,
        steering_gain: float = 6.0,
        follow_distance: float = 1.5,
        dead_zone: float = 0.2,
        kp: float = 2.0,
        kd: float = 0.8,
    ):
        self.base_speed = base_speed
        self.steering_gain = steering_gain
        self.follow_distance = follow_distance
        self.dead_zone = dead_zone
        self.kp = kp
        self.kd = kd
        self._prev_error: float | None = None

    def compute(self, target: TargetMeasurement, dt: float = 0.02) -> Twist2D:
        if not target.valid or target.stale:
            self._prev_error = None
            return Twist2D(linear=0.0, angular=0.0)

        # Steering: proportional to bearing angle (unchanged — works well)
        turn = self.steering_gain * target.bearing

        # Distance error: positive = too far, negative = too close
        error = target.distance_estimate - self.follow_distance

        # Dead zone: if within tolerance, stop approaching
        if abs(error) < self.dead_zone:
            forward = 0.0
            self._prev_error = error
            return Twist2D(linear=forward, angular=turn)

        # PD control on distance error
        d_error = 0.0
        if self._prev_error is not None and dt > 0:
            d_error = (error - self._prev_error) / dt
        self._prev_error = error

        forward = self.kp * error + self.kd * d_error

        # Modulate by alignment — only drive forward when roughly facing target
        alignment = max(0.0, math.cos(target.bearing))
        forward *= alignment

        # Clamp to safe speed range (allow negative for backing up if too close)
        forward = clamp(forward, -self.base_speed * 0.3, self.base_speed)

        return Twist2D(linear=forward, angular=turn)
