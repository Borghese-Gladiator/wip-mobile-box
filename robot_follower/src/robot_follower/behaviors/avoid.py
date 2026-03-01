"""Avoid behaviour — compute a Twist correction from obstacle data."""

import math
import numpy as np
from ..types.messages import ObstacleSectors, Twist


class AvoidBehavior:
    """
    Generates a velocity correction to steer away from obstacles.

    Reads from ObstacleSectors (sector-minimum ranges from LiDAR) and
    produces a (delta_v, delta_w) correction.

    Parameters
    ----------
    stop_distance : float
        Hard stop if any frontal sector is closer (metres).
    slow_distance : float
        Begin slowing down beyond stop_distance up to here.
    turn_bias : float
        Positive = prefer turning right when heading is blocked.
    """

    def __init__(
        self,
        stop_distance: float = 0.4,
        slow_distance: float = 1.0,
        turn_bias: float = 1.0,
    ):
        self.stop_distance = stop_distance
        self.slow_distance = slow_distance
        self.turn_bias = turn_bias

    def compute(self, sectors: ObstacleSectors) -> Twist:
        """Return a Twist correction (add to follow command)."""
        front = sectors.front
        fl = sectors.front_left
        fr = sectors.front_right

        if front <= self.stop_distance:
            # Hard stop + turn away
            turn = self.turn_bias * (1.0 if fr > fl else -1.0)
            return Twist(v=-0.2, w=turn)

        delta_v = 0.0
        delta_w = 0.0

        if front < self.slow_distance:
            scale = (front - self.stop_distance) / (self.slow_distance - self.stop_distance)
            delta_v = -(1.0 - scale) * 0.5
            if fr > fl:
                delta_w = self.turn_bias * (1.0 - scale)
            else:
                delta_w = -self.turn_bias * (1.0 - scale)

        return Twist(v=delta_v, w=delta_w)
