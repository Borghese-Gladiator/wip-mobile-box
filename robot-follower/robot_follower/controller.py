"""Core motion controller: converts follow/avoid demands into wheel velocities."""

from __future__ import annotations

import math

from .utils import clamp


class Controller:
    """Differential-drive controller producing left/right wheel velocities (rad/s)."""

    def __init__(self, config: dict):
        self.cfg = config["robot"]
        self.wheel_radius = self.cfg["wheel_radius"]
        self.wheelbase = self.cfg["wheelbase"]
        self.max_wheel_speed = self.cfg["max_wheel_speed"]
        # Base forward speed in rad/s (moderate — ~0.12 m/s linear)
        self.base_speed = 3.0

    def compute_follow(self, bearing: float, signal_strength: float) -> tuple[float, float]:
        """Compute (forward_speed, turn_rate) for target following.

        bearing: relative angle to target in [-pi, pi]
        signal_strength: 0..1 (1=close)
        Returns (forward_vel, turn_correction) in rad/s for wheel mixing.
        """
        # Steering proportional to bearing error
        turn_rate = 6.0 * bearing

        # Forward speed: fast when aligned, slow when turning
        alignment = max(0.0, math.cos(bearing))
        # Reduce speed when very close (signal_strength > 0.8)
        proximity_factor = 1.0 - max(0.0, (signal_strength - 0.8) / 0.2)
        forward_speed = self.base_speed * alignment * max(0.3, proximity_factor)

        return (forward_speed, turn_rate)

    def compute_avoid(self, front: float, front_left: float, front_right: float) -> tuple[float, float]:
        """Compute (forward_speed, turn_rate) for obstacle avoidance.

        Danger values in [0, 1], 1 = very close obstacle.
        Returns correction terms to blend with follow output.
        """
        # Slow down proportional to front danger
        speed_suppress = -self.base_speed * front

        # Steer away from more dangerous side
        turn_rate = 0.0
        if front_left > 0.2 or front_right > 0.2:
            turn_rate = 5.0 * (front_left - front_right)

        # If front is very dangerous, add strong turn
        if front > 0.5:
            if front_left < front_right:
                turn_rate += 4.0
            else:
                turn_rate -= 4.0

        return (speed_suppress, turn_rate)

    def mix_to_wheels(self, forward: float, turn: float) -> tuple[float, float]:
        """Convert (forward_vel, turn_rate) to (left_vel, right_vel) in rad/s.

        Differential drive mixing: left = forward - turn, right = forward + turn.
        """
        left = forward - turn * 0.5
        right = forward + turn * 0.5

        # Clamp to actuator limits
        left = clamp(left, -self.max_wheel_speed, self.max_wheel_speed)
        right = clamp(right, -self.max_wheel_speed, self.max_wheel_speed)

        return (left, right)
