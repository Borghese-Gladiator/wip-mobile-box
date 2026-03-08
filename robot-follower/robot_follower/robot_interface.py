"""Low-level robot control and state access via MuJoCo."""

from __future__ import annotations

import math

import mujoco
import numpy as np

from .utils import clamp, normalize_angle


def quat_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
    """Extract yaw angle from quaternion (w, x, y, z)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotInterface:
    """Interface to the differential-drive robot in MuJoCo."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, config: dict):
        self.model = model
        self.data = data
        self.cfg = config["robot"]
        self._max_wheel_speed = self.cfg["max_wheel_speed"]

        # Cache actuator IDs
        self._left_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_motor")
        self._right_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_motor")

        # Cache body ID and freejoint qpos address
        self._body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot")
        free_jnt = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "robot_free")
        self._qpos_adr = model.jnt_qposadr[free_jnt]

    def get_pose(self) -> tuple[float, float, float]:
        """Return (x, y, yaw) of the robot chassis from qpos."""
        adr = self._qpos_adr
        x = float(self.data.qpos[adr])
        y = float(self.data.qpos[adr + 1])
        qw, qx, qy, qz = self.data.qpos[adr + 3:adr + 7]
        yaw = quat_to_yaw(qw, qx, qy, qz)
        return (x, y, yaw)

    def get_position(self) -> tuple[float, float]:
        """Return (x, y) position of the robot chassis."""
        adr = self._qpos_adr
        return (float(self.data.qpos[adr]), float(self.data.qpos[adr + 1]))

    def get_heading(self) -> float:
        """Return robot yaw angle in [-pi, pi]."""
        _, _, yaw = self.get_pose()
        return yaw

    def get_velocity(self) -> tuple[float, float]:
        """Return (linear_vel, angular_vel) of robot."""
        linvel = self.data.subtree_linvel[self._body_id]
        lin = math.hypot(float(linvel[0]), float(linvel[1]))
        # Angular velocity from cvel (spatial velocity, first 3 are angular)
        ang = float(self.data.cvel[self._body_id][2])
        return (lin, ang)

    def set_wheel_velocities(self, left: float, right: float) -> None:
        """Set target angular velocities for left and right wheels (rad/s).

        Clamped to [-max_wheel_speed, max_wheel_speed].
        """
        left = clamp(left, -self._max_wheel_speed, self._max_wheel_speed)
        right = clamp(right, -self._max_wheel_speed, self._max_wheel_speed)
        self.data.ctrl[self._left_act] = left
        self.data.ctrl[self._right_act] = right

    def stop(self) -> None:
        """Stop both motors."""
        self.set_wheel_velocities(0.0, 0.0)
