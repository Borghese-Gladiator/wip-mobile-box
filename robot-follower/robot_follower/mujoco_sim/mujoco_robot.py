"""MuJoCo robot: reads state from sim, writes actuator commands."""

from __future__ import annotations

import math

import mujoco

from robot_follower.core.types import Pose2D, Twist2D, WheelCommand, WheelState
from robot_follower.core.math_utils import quat_to_yaw, clamp


class MuJoCoRobot:
    """Thin wrapper over MuJoCo model/data for the differential-drive robot."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, max_wheel_speed: float = 10.0):
        self._model = model
        self._data = data
        self._max = max_wheel_speed

        self._left_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_motor")
        self._right_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_motor")
        self._body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot")

        free_jnt = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "robot_free")
        self._qpos_adr = model.jnt_qposadr[free_jnt]

        self._left_jnt = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_joint")
        self._right_jnt = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_joint")

    def get_pose(self) -> Pose2D:
        adr = self._qpos_adr
        x = float(self._data.qpos[adr])
        y = float(self._data.qpos[adr + 1])
        qw, qx, qy, qz = self._data.qpos[adr + 3:adr + 7]
        yaw = quat_to_yaw(qw, qx, qy, qz)
        return Pose2D(x=x, y=y, yaw=yaw)

    def get_velocity(self) -> Twist2D:
        linvel = self._data.subtree_linvel[self._body_id]
        lin = math.hypot(float(linvel[0]), float(linvel[1]))
        ang = float(self._data.cvel[self._body_id][2])
        return Twist2D(linear=lin, angular=ang)

    def get_wheel_speeds(self) -> tuple[float, float]:
        """True wheel angular velocities from MuJoCo joints."""
        left_dof = self._model.jnt_dofadr[self._left_jnt]
        right_dof = self._model.jnt_dofadr[self._right_jnt]
        return (float(self._data.qvel[left_dof]), float(self._data.qvel[right_dof]))

    def set_ctrl(self, left: float, right: float) -> None:
        left = clamp(left, -self._max, self._max)
        right = clamp(right, -self._max, self._max)
        self._data.ctrl[self._left_act] = left
        self._data.ctrl[self._right_act] = right
