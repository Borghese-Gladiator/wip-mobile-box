"""Tests for behavior manager state transitions."""

import pytest
from robot_follower.controller import Controller
from robot_follower.behavior_manager import BehaviorManager, BehaviorState
from robot_follower.obstacle_sensors import ObstacleReading
from robot_follower.utils import load_config


@pytest.fixture
def behavior():
    config = load_config()
    controller = Controller(config)
    return BehaviorManager(config, controller)


class TestBehaviorStates:
    def test_target_reached_stops(self, behavior):
        obs = ObstacleReading(front=0.0, front_left=0.0, front_right=0.0)
        left, right = behavior.update(
            bearing=0.0, signal_strength=0.5, obstacles=obs,
            robot_speed=0.5, target_reached=True, sim_time=1.0,
        )
        assert left == 0.0
        assert right == 0.0
        assert behavior.state == BehaviorState.TARGET_REACHED

    def test_clear_path_follows(self, behavior):
        obs = ObstacleReading(front=0.0, front_left=0.0, front_right=0.0)
        left, right = behavior.update(
            bearing=0.0, signal_strength=0.5, obstacles=obs,
            robot_speed=0.5, target_reached=False, sim_time=1.0,
        )
        # Should produce forward motion (both positive, roughly equal)
        assert left > 0
        assert right > 0
        assert behavior.state == BehaviorState.FOLLOW_TARGET

    def test_high_danger_triggers_avoid(self, behavior):
        obs = ObstacleReading(front=0.8, front_left=0.3, front_right=0.1)
        behavior.update(
            bearing=0.0, signal_strength=0.5, obstacles=obs,
            robot_speed=0.5, target_reached=False, sim_time=1.0,
        )
        assert behavior.state == BehaviorState.AVOID_OBSTACLE

    def test_stall_triggers_escape(self, behavior):
        obs = ObstacleReading(front=0.9, front_left=0.8, front_right=0.8)
        # Simulate stall for longer than escape_stall_time
        for t in range(50):
            behavior.update(
                bearing=0.0, signal_strength=0.5, obstacles=obs,
                robot_speed=0.01, target_reached=False, sim_time=float(t) * 0.1,
            )
        assert behavior.state == BehaviorState.ESCAPE_TRAP
