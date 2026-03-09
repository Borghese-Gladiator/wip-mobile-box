"""Obstacle avoidance controller: danger readings → avoidance twist."""

from __future__ import annotations

from .types import ObstacleMeasurement, Twist2D


class ObstacleAvoidance:
    """Computes avoidance velocity correction from obstacle proximity."""

    def __init__(self, base_speed: float = 3.0):
        self.base_speed = base_speed

    def compute(self, obs: ObstacleMeasurement) -> Twist2D:
        speed_suppress = -self.base_speed * obs.front

        turn = 0.0
        if obs.front_left > 0.2 or obs.front_right > 0.2:
            turn = 5.0 * (obs.front_left - obs.front_right)

        if obs.front > 0.5:
            if obs.front_left < obs.front_right:
                turn += 4.0
            else:
                turn -= 4.0

        return Twist2D(linear=speed_suppress, angular=turn)
