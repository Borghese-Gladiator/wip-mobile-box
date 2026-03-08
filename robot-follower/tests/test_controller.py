"""Tests for the motion controller."""

import math
import pytest
from robot_follower.controller import Controller
from robot_follower.utils import load_config


@pytest.fixture
def controller():
    config = load_config()
    return Controller(config)


class TestFollowCompute:
    def test_target_ahead_gives_forward(self, controller):
        fwd, turn = controller.compute_follow(bearing=0.0, signal_strength=0.5)
        assert fwd > 0.3
        assert abs(turn) < 0.1

    def test_target_left_gives_left_turn(self, controller):
        fwd, turn = controller.compute_follow(bearing=0.5, signal_strength=0.5)
        assert turn > 0.5  # positive = turn left

    def test_target_right_gives_right_turn(self, controller):
        fwd, turn = controller.compute_follow(bearing=-0.5, signal_strength=0.5)
        assert turn < -0.5

    def test_close_target_slows_down(self, controller):
        fwd_far, _ = controller.compute_follow(bearing=0.0, signal_strength=0.3)
        fwd_close, _ = controller.compute_follow(bearing=0.0, signal_strength=0.95)
        assert fwd_close < fwd_far


class TestWheelMixing:
    @pytest.mark.parametrize("forward,turn,left_gt_right", [
        (0.5, 0.0, None),   # straight: equal
        (0.5, 0.5, False),  # turn left: right > left
        (0.5, -0.5, True),  # turn right: left > right
    ])
    def test_mixing(self, controller, forward, turn, left_gt_right):
        left, right = controller.mix_to_wheels(forward, turn)
        if left_gt_right is None:
            assert left == pytest.approx(right, abs=0.05)
        elif left_gt_right:
            assert left > right
        else:
            assert right > left

    def test_velocity_in_range(self, controller):
        left, right = controller.mix_to_wheels(100.0, 50.0)
        assert -10.0 <= left <= 10.0
        assert -10.0 <= right <= 10.0
