"""Environment state manager: target motion, episode checks."""

from __future__ import annotations

import math

import mujoco
import numpy as np

from .motion_profiles import WaypointProfile
from .utils import distance_2d


class SimEnvironment:
    """Manages target motion and episode lifecycle."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, config: dict):
        self.model = model
        self.data = data
        self.config = config
        self.success_radius = config["target"]["success_radius"]

        self._target_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
        self._robot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot")

        # Target joint (freejoint) qpos index
        self._target_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "target_free")
        self._target_qpos_adr = model.jnt_qposadr[self._target_jnt_id]

        self.motion_profile = WaypointProfile(config)

    def get_target_pos(self) -> tuple[float, float]:
        pos = self.data.xpos[self._target_body_id]
        return (float(pos[0]), float(pos[1]))

    def get_robot_pos(self) -> tuple[float, float]:
        pos = self.data.xpos[self._robot_body_id]
        return (float(pos[0]), float(pos[1]))

    def step_target_motion(self, dt: float) -> None:
        """Move the target along its motion profile by directly setting qpos."""
        current_pos = self.get_target_pos()
        dx, dy, heading = self.motion_profile.update(current_pos, dt)

        adr = self._target_qpos_adr
        # Update position (freejoint qpos: x, y, z, qw, qx, qy, qz)
        self.data.qpos[adr + 0] += dx
        self.data.qpos[adr + 1] += dy
        # Keep z at ground level for the capsule
        self.data.qpos[adr + 2] = 0.5

        # Update heading quaternion (rotation around z)
        half = heading / 2.0
        self.data.qpos[adr + 3] = math.cos(half)  # qw
        self.data.qpos[adr + 4] = 0.0              # qx
        self.data.qpos[adr + 5] = 0.0              # qy
        self.data.qpos[adr + 6] = math.sin(half)   # qz

        # Zero target velocities so it doesn't get pushed around
        vel_adr = self.model.jnt_dofadr[self._target_jnt_id]
        self.data.qvel[vel_adr:vel_adr + 6] = 0.0

    def is_target_reached(self) -> bool:
        return distance_2d(self.get_robot_pos(), self.get_target_pos()) < self.success_radius
