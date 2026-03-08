"""Obstacle proximity sensing using MuJoCo raycasting."""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np

from .utils import normalize_angle


@dataclass
class ObstacleReading:
    """Obstacle proximity readings for three zones."""
    front: float      # 0 = clear, 1 = blocked
    front_left: float
    front_right: float

    @property
    def front_clear(self) -> bool:
        return self.front < 0.4

    @property
    def left_clear(self) -> bool:
        return self.front_left < 0.4

    @property
    def right_clear(self) -> bool:
        return self.front_right < 0.4

    @property
    def any_danger(self) -> bool:
        return self.front >= 0.4 or self.front_left >= 0.4 or self.front_right >= 0.4


class ObstacleSensors:
    """Raycast-based obstacle detection around the robot."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, config: dict):
        self.model = model
        self.data = data
        self.cfg = config["sensing"]["obstacle"]
        self._robot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot")
        self._robot_geom_ids = self._get_robot_geom_ids()

        self.ray_length = self.cfg["ray_length"]
        # Sensor zones: (center_angle_offset, spread)
        # Angles are relative to robot heading
        self._zones = {
            "front": (0.0, 0.2),          # +-0.2 rad around forward
            "front_left": (0.6, 0.2),     # ~35 deg left
            "front_right": (-0.6, 0.2),   # ~35 deg right
        }

    def _get_robot_geom_ids(self) -> set[int]:
        """Get all geom IDs belonging to the robot to exclude from raycasts."""
        ids = set()
        for i in range(self.model.ngeom):
            body = self.model.geom_bodyid[i]
            # Walk up parent chain to see if this belongs to robot
            b = body
            while b > 0:
                if b == self._robot_body_id:
                    ids.add(i)
                    break
                b = self.model.body_parentid[b]
        return ids

    def _cast_ray(self, origin: np.ndarray, direction: np.ndarray) -> float:
        """Cast a ray and return normalized danger (0=clear, 1=contact). Excludes robot geoms."""
        geom_id = np.array([-1], dtype=np.int32)
        dist = mujoco.mj_ray(
            self.model, self.data,
            origin, direction,
            None,  # geomgroup
            1,     # flg_static
            self._robot_body_id,  # bodyexclude
            geom_id,
        )
        if dist < 0:
            return 0.0  # no hit
        # Normalize: closer = more danger
        return max(0.0, 1.0 - dist / self.ray_length)

    def read(self, robot_pos: tuple[float, float], robot_heading: float) -> ObstacleReading:
        """Read obstacle proximity for all zones."""
        origin = np.array([robot_pos[0], robot_pos[1], 0.065], dtype=np.float64)
        results = {}

        for zone_name, (angle_offset, spread) in self._zones.items():
            num_rays = self.cfg.get("num_rays", 3)
            max_danger = 0.0

            for i in range(num_rays):
                if num_rays > 1:
                    frac = i / (num_rays - 1) - 0.5  # [-0.5, 0.5]
                else:
                    frac = 0.0
                angle = robot_heading + angle_offset + frac * spread * 2

                direction = np.array([math.cos(angle), math.sin(angle), 0.0], dtype=np.float64)
                danger = self._cast_ray(origin, direction)
                max_danger = max(max_danger, danger)

            results[zone_name] = max_danger

        return ObstacleReading(
            front=results["front"],
            front_left=results["front_left"],
            front_right=results["front_right"],
        )
