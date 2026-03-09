"""Tests for behavior manager state transitions."""

import pytest
from robot_follower.core.types import (
    BehaviorState, TargetMeasurement, ObstacleMeasurement,
)
from robot_follower.core.follower_controller import FollowerController
from robot_follower.core.obstacle_avoidance import ObstacleAvoidance
from robot_follower.core.wheel_controller import WheelController
from robot_follower.core.behavior_manager import BehaviorManager


@pytest.fixture
def behavior():
    follower = FollowerController(base_speed=3.0, steering_gain=6.0)
    avoidance = ObstacleAvoidance(base_speed=3.0)
    wheel_ctrl = WheelController(wheel_radius=0.04, wheelbase=0.22, max_wheel_speed=10.0)
    return BehaviorManager(
        follower=follower, avoidance=avoidance, wheel_ctrl=wheel_ctrl,
        avoid_weight=1.5, escape_stall_time=2.0, escape_turn_speed=4.0,
    )


def _target(strength=0.5, bearing=0.0, valid=True, stale=False):
    return TargetMeasurement(
        signal_strength=strength, bearing=bearing,
        valid=valid, stale=stale,
    )


class TestBehaviorStates:
    def test_target_reached_stops(self, behavior):
        obs = ObstacleMeasurement(front=0.0, front_left=0.0, front_right=0.0)
        cmd = behavior.update(
            target=_target(), obstacles=obs,
            robot_speed=0.5, target_reached=True, sim_time=1.0,
        )
        assert cmd.left == 0.0
        assert cmd.right == 0.0
        assert behavior.state == BehaviorState.TARGET_REACHED

    def test_clear_path_follows(self, behavior):
        obs = ObstacleMeasurement(front=0.0, front_left=0.0, front_right=0.0)
        cmd = behavior.update(
            target=_target(), obstacles=obs,
            robot_speed=0.5, target_reached=False, sim_time=1.0,
        )
        assert cmd.left > 0
        assert cmd.right > 0
        assert behavior.state == BehaviorState.FOLLOW_TARGET

    def test_high_danger_triggers_avoid(self, behavior):
        obs = ObstacleMeasurement(front=0.8, front_left=0.3, front_right=0.1)
        behavior.update(
            target=_target(), obstacles=obs,
            robot_speed=0.5, target_reached=False, sim_time=1.0,
        )
        assert behavior.state == BehaviorState.AVOID_OBSTACLE

    def test_stall_triggers_escape(self, behavior):
        obs = ObstacleMeasurement(front=0.9, front_left=0.8, front_right=0.8)
        for t in range(50):
            behavior.update(
                target=_target(), obstacles=obs,
                robot_speed=0.01, target_reached=False, sim_time=float(t) * 0.1,
            )
        assert behavior.state == BehaviorState.ESCAPE_TRAP

    def test_lost_target_triggers_search(self, behavior):
        obs = ObstacleMeasurement(front=0.0, front_left=0.0, front_right=0.0)
        behavior.update(
            target=_target(valid=False), obstacles=obs,
            robot_speed=0.5, target_reached=False, sim_time=1.0,
        )
        assert behavior.state == BehaviorState.SEARCH_TARGET
