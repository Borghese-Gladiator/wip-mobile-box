"""Person-like target movement patterns — pure math."""

from __future__ import annotations

import math

from .math_utils import distance_2d, normalize_angle


class WaypointProfile:
    """Waypoint-based walking profile with smooth heading transitions."""

    def __init__(self, waypoints: list[tuple[float, float]], speed: float = 0.6,
                 speed_variation: float = 0.15, turn_smoothing: float = 0.3):
        self.waypoints = waypoints
        self.base_speed = speed
        self.speed_var = speed_variation
        self.turn_smoothing = turn_smoothing
        self.current_wp_idx = 0
        self.heading = 0.0

    def update(self, pos: tuple[float, float], dt: float) -> tuple[float, float, float]:
        """Compute (dx, dy, heading) displacement for this step."""
        goal = self.waypoints[self.current_wp_idx]
        dist = distance_2d(pos, goal)

        if dist < 0.3:
            self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
            goal = self.waypoints[self.current_wp_idx]

        desired_heading = math.atan2(goal[1] - pos[1], goal[0] - pos[0])
        heading_error = normalize_angle(desired_heading - self.heading)
        self.heading = normalize_angle(self.heading + heading_error * self.turn_smoothing)

        alignment = max(0.0, math.cos(heading_error))
        speed = self.base_speed * (1.0 - self.speed_var + self.speed_var * alignment)

        dx = speed * math.cos(self.heading) * dt
        dy = speed * math.sin(self.heading) * dt
        return (dx, dy, self.heading)
