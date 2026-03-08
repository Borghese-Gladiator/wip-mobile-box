"""Behavior arbitration: blends follow and avoid, handles escape."""

from __future__ import annotations

from enum import Enum, auto

from .controller import Controller
from .obstacle_sensors import ObstacleReading
from .utils import clamp


class BehaviorState(Enum):
    FOLLOW_TARGET = auto()
    AVOID_OBSTACLE = auto()
    ESCAPE_TRAP = auto()
    TARGET_REACHED = auto()


class BehaviorManager:
    """Arbitrates between follow, avoid, and escape behaviors."""

    def __init__(self, config: dict, controller: Controller):
        self.controller = controller
        self.cfg = config["behavior"]
        self.state = BehaviorState.FOLLOW_TARGET

        self._stall_start: float | None = None
        self._escape_stall_time = self.cfg["escape_stall_time"]
        self._escape_turn_speed = self.cfg["escape_turn_speed"]
        self._avoid_weight = self.cfg["avoid_weight"]

    def update(
        self,
        bearing: float,
        signal_strength: float,
        obstacles: ObstacleReading,
        robot_speed: float,
        target_reached: bool,
        sim_time: float,
    ) -> tuple[float, float]:
        """Compute final (left_vel, right_vel) in rad/s."""

        if target_reached:
            self.state = BehaviorState.TARGET_REACHED
            return (0.0, 0.0)

        # Check for stall → escape
        if robot_speed < 0.05 and obstacles.any_danger:
            if self._stall_start is None:
                self._stall_start = sim_time
            elif sim_time - self._stall_start > self._escape_stall_time:
                self.state = BehaviorState.ESCAPE_TRAP
        else:
            self._stall_start = None

        # Escape behavior: spin in place
        if self.state == BehaviorState.ESCAPE_TRAP:
            if obstacles.front_clear:
                self.state = BehaviorState.FOLLOW_TARGET
                self._stall_start = None
            else:
                # Rotate toward the clearer side
                if obstacles.front_left <= obstacles.front_right:
                    return (self._escape_turn_speed, -self._escape_turn_speed)
                else:
                    return (-self._escape_turn_speed, self._escape_turn_speed)

        # Normal operation: blend follow + avoid
        follow_fwd, follow_turn = self.controller.compute_follow(bearing, signal_strength)
        avoid_fwd, avoid_turn = self.controller.compute_avoid(
            obstacles.front, obstacles.front_left, obstacles.front_right
        )

        # Danger level determines blend
        danger = max(obstacles.front, obstacles.front_left, obstacles.front_right)
        if danger > 0.6:
            self.state = BehaviorState.AVOID_OBSTACLE
        elif danger < 0.2:
            self.state = BehaviorState.FOLLOW_TARGET

        # Weighted blend
        w_avoid = clamp(danger * self._avoid_weight, 0.0, 1.0)
        w_follow = 1.0 - w_avoid

        fwd = w_follow * follow_fwd + w_avoid * avoid_fwd
        turn = w_follow * follow_turn + w_avoid * avoid_turn

        return self.controller.mix_to_wheels(fwd, turn)
