"""MuJoCo target mover: drives the person-like target along waypoints."""

from __future__ import annotations

import math

import mujoco

from robot_follower.core.motion_profiles import WaypointProfile


class MuJoCoTarget:
    """Moves the target body along a waypoint profile by setting qpos."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, config: dict):
        self._model = model
        self._data = data
        self._body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")

        jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "target_free")
        self._qpos_adr = model.jnt_qposadr[jnt_id]
        self._dof_adr = model.jnt_dofadr[jnt_id]

        mp = config["motion_profiles"]["waypoint"]
        waypoints = [tuple(w) for w in mp["waypoints"]]
        self._profile = WaypointProfile(
            waypoints=waypoints,
            speed=mp["speed"],
            speed_variation=mp["speed_variation"],
            turn_smoothing=mp["turn_smoothing"],
        )

    def get_position(self) -> tuple[float, float]:
        pos = self._data.xpos[self._body_id]
        return (float(pos[0]), float(pos[1]))

    def step(self, dt: float) -> None:
        current = self.get_position()
        dx, dy, heading = self._profile.update(current, dt)

        adr = self._qpos_adr
        self._data.qpos[adr + 0] += dx
        self._data.qpos[adr + 1] += dy
        self._data.qpos[adr + 2] = 0.5

        half = heading / 2.0
        self._data.qpos[adr + 3] = math.cos(half)
        self._data.qpos[adr + 4] = 0.0
        self._data.qpos[adr + 5] = 0.0
        self._data.qpos[adr + 6] = math.sin(half)

        self._data.qvel[self._dof_adr:self._dof_adr + 6] = 0.0
