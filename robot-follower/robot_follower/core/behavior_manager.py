"""Behavior arbitration: blends follow + avoid, handles escape and search."""

from __future__ import annotations

from .types import (
    BehaviorState, TargetMeasurement, ObstacleMeasurement, Twist2D, WheelCommand,
)
from .follower_controller import FollowerController
from .obstacle_avoidance import ObstacleAvoidance
from .wheel_controller import WheelController
from .math_utils import clamp


class BehaviorManager:
    """Top-level behavior arbitration."""

    def __init__(
        self,
        follower: FollowerController,
        avoidance: ObstacleAvoidance,
        wheel_ctrl: WheelController,
        avoid_weight: float = 1.5,
        escape_stall_time: float = 2.0,
        escape_turn_speed: float = 4.0,
    ):
        self.follower = follower
        self.avoidance = avoidance
        self.wheel_ctrl = wheel_ctrl
        self.state = BehaviorState.FOLLOW_TARGET

        self._avoid_weight = avoid_weight
        self._escape_stall_time = escape_stall_time
        self._escape_turn_speed = escape_turn_speed
        self._stall_start: float | None = None

    def update(
        self,
        target: TargetMeasurement,
        obstacles: ObstacleMeasurement,
        robot_speed: float,
        target_reached: bool,
        sim_time: float,
    ) -> WheelCommand:
        if target_reached:
            self.state = BehaviorState.TARGET_REACHED
            return WheelCommand(0.0, 0.0)

        # Search mode if target signal lost
        if not target.valid or target.stale:
            self.state = BehaviorState.SEARCH_TARGET
            return self.wheel_ctrl.compute(Twist2D(linear=0.0, angular=2.0))

        # Stall detection → escape
        if robot_speed < 0.05 and obstacles.any_danger:
            if self._stall_start is None:
                self._stall_start = sim_time
            elif sim_time - self._stall_start > self._escape_stall_time:
                self.state = BehaviorState.ESCAPE_TRAP
        else:
            self._stall_start = None

        # Escape: spin in place
        if self.state == BehaviorState.ESCAPE_TRAP:
            if obstacles.front_clear:
                self.state = BehaviorState.FOLLOW_TARGET
                self._stall_start = None
            else:
                if obstacles.front_left <= obstacles.front_right:
                    return WheelCommand(self._escape_turn_speed, -self._escape_turn_speed)
                else:
                    return WheelCommand(-self._escape_turn_speed, self._escape_turn_speed)

        # Normal: blend follow + avoid
        follow = self.follower.compute(target)
        avoid = self.avoidance.compute(obstacles)

        danger = max(obstacles.front, obstacles.front_left, obstacles.front_right)
        if danger > 0.6:
            self.state = BehaviorState.AVOID_OBSTACLE
        elif danger < 0.2:
            self.state = BehaviorState.FOLLOW_TARGET

        w_avoid = clamp(danger * self._avoid_weight, 0.0, 1.0)
        w_follow = 1.0 - w_avoid

        blended = Twist2D(
            linear=w_follow * follow.linear + w_avoid * avoid.linear,
            angular=w_follow * follow.angular + w_avoid * avoid.angular,
        )
        return self.wheel_ctrl.compute(blended)
