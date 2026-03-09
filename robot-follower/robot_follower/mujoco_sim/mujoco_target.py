"""MuJoCo target mover: drives the person-like target along waypoints."""

from __future__ import annotations

import math

import mujoco
import numpy as np

from robot_follower.core.motion_profiles import WaypointProfile


class MuJoCoTarget:
    """Moves the target body along a waypoint profile by setting qpos.

    Uses raycasting to prevent the target from walking into walls —
    displacement is clamped when a wall is detected ahead.
    """

    _WALL_MARGIN = 0.15  # stop this far from the wall surface
    _RAY_HEIGHT = 0.5    # raycast origin height (torso centre)

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

    def _clamp_displacement(self, x: float, y: float, dx: float, dy: float) -> tuple[float, float]:
        """Raycast in the movement direction; shorten step if a wall is hit."""
        step_len = math.hypot(dx, dy)
        if step_len < 1e-6:
            return dx, dy

        direction = np.array([dx / step_len, dy / step_len, 0.0], dtype=np.float64)
        origin = np.array([x, y, self._RAY_HEIGHT], dtype=np.float64)
        geom_id = np.array([-1], dtype=np.int32)

        hit_dist = mujoco.mj_ray(
            self._model, self._data, origin, direction,
            None, 1, self._body_id, geom_id,
        )

        if hit_dist < 0:
            # No hit — movement is clear
            return dx, dy

        max_travel = max(0.0, hit_dist - self._WALL_MARGIN)
        if step_len <= max_travel:
            return dx, dy

        # Shorten the displacement to stop before the wall
        scale = max_travel / step_len
        clamped_dx = dx * scale
        clamped_dy = dy * scale

        # Slide along the wall: project remainder onto the wall tangent
        # Get wall normal from the hit geometry
        remainder = step_len - max_travel
        tangent = np.array([-direction[1], direction[0], 0.0])
        # Project original movement direction's perpendicular component
        slide_dot = dx * tangent[0] + dy * tangent[1]
        slide_dx = tangent[0] * slide_dot * (remainder / step_len)
        slide_dy = tangent[1] * slide_dot * (remainder / step_len)

        # Verify slide direction is also clear
        if abs(slide_dx) > 1e-6 or abs(slide_dy) > 1e-6:
            slide_len = math.hypot(slide_dx, slide_dy)
            slide_dir = np.array([slide_dx / slide_len, slide_dy / slide_len, 0.0], dtype=np.float64)
            slide_origin = np.array([x + clamped_dx, y + clamped_dy, self._RAY_HEIGHT], dtype=np.float64)
            slide_hit = mujoco.mj_ray(
                self._model, self._data, slide_origin, slide_dir,
                None, 1, self._body_id, geom_id,
            )
            if 0 <= slide_hit < slide_len + self._WALL_MARGIN:
                slide_dx = 0.0
                slide_dy = 0.0

        return clamped_dx + slide_dx, clamped_dy + slide_dy

    def step(self, dt: float) -> None:
        current = self.get_position()
        dx, dy, heading = self._profile.update(current, dt)

        # Clamp displacement to prevent walking into walls
        dx, dy = self._clamp_displacement(current[0], current[1], dx, dy)

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
