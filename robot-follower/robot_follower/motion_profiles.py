"""Person-like target movement patterns."""

from __future__ import annotations

import math

from .utils import distance_2d, lerp, normalize_angle


class WaypointProfile:
    """Waypoint-based walking profile with smooth heading transitions."""

    def __init__(self, config: dict):
        mp_cfg = config["motion_profiles"]["waypoint"]
        self.waypoints: list[tuple[float, float]] = [tuple(w) for w in mp_cfg["waypoints"]]
        self.base_speed: float = mp_cfg["speed"]
        self.speed_var: float = mp_cfg["speed_variation"]
        self.turn_smoothing: float = mp_cfg["turn_smoothing"]
        self.current_wp_idx: int = 0
        self.heading: float = 0.0

    def get_current_target(self) -> tuple[float, float]:
        return self.waypoints[self.current_wp_idx]

    def update(self, pos: tuple[float, float], dt: float) -> tuple[float, float, float]:
        """Compute desired (vx, vy, heading) for the target.

        Returns (dx, dy, new_heading) — displacement this step.
        """
        goal = self.get_current_target()
        dist = distance_2d(pos, goal)

        # Advance waypoint if close enough
        if dist < 0.3:
            self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
            goal = self.get_current_target()
            dist = distance_2d(pos, goal)

        # Compute desired heading toward waypoint
        desired_heading = math.atan2(goal[1] - pos[1], goal[0] - pos[0])

        # Smooth heading transition
        heading_error = normalize_angle(desired_heading - self.heading)
        self.heading = normalize_angle(
            self.heading + heading_error * self.turn_smoothing
        )

        # Speed with small variation based on heading alignment
        alignment = max(0.0, math.cos(heading_error))
        speed = self.base_speed * (1.0 - self.speed_var + self.speed_var * alignment)

        dx = speed * math.cos(self.heading) * dt
        dy = speed * math.sin(self.heading) * dt

        return (dx, dy, self.heading)
