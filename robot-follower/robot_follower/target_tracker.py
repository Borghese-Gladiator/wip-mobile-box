"""Simulated BLE-like target sensing: RSSI + relative bearing."""

from __future__ import annotations

import math
import random

import mujoco

from .utils import bearing_to, distance_2d


class TargetTracker:
    """Estimates target distance (via RSSI) and bearing from robot."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, config: dict):
        self.model = model
        self.data = data
        self.cfg = config["sensing"]["rssi"]
        self.bearing_noise = config["sensing"]["bearing"]["noise_std"]
        self._target_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
        self._robot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot")

    def get_target_position(self) -> tuple[float, float]:
        """Ground-truth target position."""
        pos = self.data.xpos[self._target_body_id]
        return (float(pos[0]), float(pos[1]))

    def get_robot_position(self) -> tuple[float, float]:
        pos = self.data.xpos[self._robot_body_id]
        return (float(pos[0]), float(pos[1]))

    def get_distance(self) -> float:
        """Euclidean distance from robot to target."""
        return distance_2d(self.get_robot_position(), self.get_target_position())

    def get_rssi(self) -> float:
        """Simulated RSSI value (dBm). Stronger (less negative) when closer."""
        dist = max(self.get_distance(), 0.01)
        ref_power = self.cfg["ref_power"]
        n = self.cfg["path_loss_exp"]
        noise_std = self.cfg["noise_std"]
        rssi = ref_power - 10.0 * n * math.log10(dist)
        rssi += random.gauss(0, noise_std)
        return rssi

    def get_signal_strength(self) -> float:
        """Normalized signal strength in [0, 1]. 1 = very close, 0 = far."""
        rssi = self.get_rssi()
        # Map RSSI range roughly [-90, -30] to [0, 1]
        strength = (rssi + 90.0) / 60.0
        return max(0.0, min(1.0, strength))

    def get_bearing(self, robot_heading: float) -> float:
        """Relative bearing to target from robot heading, with noise."""
        b = bearing_to(self.get_robot_position(), robot_heading, self.get_target_position())
        b += random.gauss(0, self.bearing_noise)
        return b

    def is_valid(self) -> bool:
        """Whether the target signal is detectable."""
        return self.get_distance() < 15.0
