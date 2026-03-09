"""MuJoCo sensor implementations: raycasting, BLE-from-ground-truth."""

from __future__ import annotations

import math
import random

import mujoco
import numpy as np

from robot_follower.core.types import ObstacleMeasurement, TargetMeasurement
from robot_follower.core.math_utils import distance_2d, bearing_to
from .mujoco_noise import BLENoise


class MuJoCoRangeSensors:
    """Raycast-based obstacle detection."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, config: dict):
        self._model = model
        self._data = data
        cfg = config["sensing"]["obstacle"]
        self._ray_length = cfg["ray_length"]
        self._num_rays = cfg.get("num_rays", 5)
        self._robot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot")
        self._zones = {
            "front": (0.0, 0.2),
            "front_left": (0.6, 0.2),
            "front_right": (-0.6, 0.2),
        }

    def read(self, robot_x: float, robot_y: float, robot_yaw: float) -> ObstacleMeasurement:
        origin = np.array([robot_x, robot_y, 0.065], dtype=np.float64)
        geom_id = np.array([-1], dtype=np.int32)
        results = {}

        for zone_name, (angle_offset, spread) in self._zones.items():
            max_danger = 0.0
            for i in range(self._num_rays):
                if self._num_rays > 1:
                    frac = i / (self._num_rays - 1) - 0.5
                else:
                    frac = 0.0
                angle = robot_yaw + angle_offset + frac * spread * 2
                direction = np.array([math.cos(angle), math.sin(angle), 0.0], dtype=np.float64)

                dist = mujoco.mj_ray(
                    self._model, self._data, origin, direction,
                    None, 1, self._robot_body_id, geom_id,
                )
                if dist < 0:
                    danger = 0.0
                else:
                    danger = max(0.0, 1.0 - dist / self._ray_length)
                max_danger = max(max_danger, danger)
            results[zone_name] = max_danger

        return ObstacleMeasurement(
            front=results["front"],
            front_left=results["front_left"],
            front_right=results["front_right"],
        )


class MuJoCoBLETracker:
    """Simulated BLE target sensing from ground-truth positions + noise."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, config: dict):
        self._model = model
        self._data = data
        rssi_cfg = config["sensing"]["rssi"]
        self._ref_power = rssi_cfg["ref_power"]
        self._path_loss_exp = rssi_cfg["path_loss_exp"]

        self._target_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
        self._robot_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot")

        self._noise = BLENoise(
            rssi_noise_std=rssi_cfg["noise_std"],
            bearing_noise_std=config["sensing"]["bearing"]["noise_std"],
        )

    def read(self, robot_x: float, robot_y: float, robot_yaw: float, timestamp: float) -> TargetMeasurement:
        tx = float(self._data.xpos[self._target_body][0])
        ty = float(self._data.xpos[self._target_body][1])
        true_dist = distance_2d((robot_x, robot_y), (tx, ty))

        true_rssi = self._ref_power - 10.0 * self._path_loss_exp * math.log10(max(true_dist, 0.01))
        true_bearing = bearing_to((robot_x, robot_y), robot_yaw, (tx, ty))

        noisy_rssi, noisy_bearing, noisy_dist, valid = self._noise.apply(true_rssi, true_bearing, true_dist)

        if not valid:
            return TargetMeasurement(valid=False, stale=True, timestamp=timestamp)

        strength = max(0.0, min(1.0, (noisy_rssi + 90.0) / 60.0))
        return TargetMeasurement(
            signal_strength=strength,
            bearing=noisy_bearing,
            distance_estimate=noisy_dist,
            valid=True,
            stale=False,
            timestamp=timestamp,
        )
