"""Follow controller: bearing + signal strength → forward/turn demands."""

from __future__ import annotations

import math

from .types import TargetMeasurement, Twist2D


class FollowerController:
    """Produces a body-frame velocity demand to follow the target."""

    def __init__(self, base_speed: float = 3.0, steering_gain: float = 6.0):
        self.base_speed = base_speed
        self.steering_gain = steering_gain

    def compute(self, target: TargetMeasurement) -> Twist2D:
        if not target.valid or target.stale:
            return Twist2D(linear=0.0, angular=0.0)

        turn = self.steering_gain * target.bearing

        alignment = max(0.0, math.cos(target.bearing))
        proximity = 1.0 - max(0.0, (target.signal_strength - 0.8) / 0.2)
        forward = self.base_speed * alignment * max(0.3, proximity)

        return Twist2D(linear=forward, angular=turn)
