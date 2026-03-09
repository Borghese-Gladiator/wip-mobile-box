"""Tests for differential-drive kinematics."""

import pytest
from robot_follower.core.kinematics import wheel_to_twist, twist_to_wheels


class TestWheelToTwist:
    def test_both_forward_gives_straight(self):
        twist = wheel_to_twist(5.0, 5.0, wheel_radius=0.04, wheelbase=0.22)
        assert twist.linear == pytest.approx(0.2, abs=0.01)
        assert twist.angular == pytest.approx(0.0, abs=0.01)

    def test_opposite_gives_rotation(self):
        twist = wheel_to_twist(-5.0, 5.0, wheel_radius=0.04, wheelbase=0.22)
        assert abs(twist.linear) < 0.01
        assert twist.angular > 0.0

    def test_zero_gives_zero(self):
        twist = wheel_to_twist(0.0, 0.0, wheel_radius=0.04, wheelbase=0.22)
        assert twist.linear == 0.0
        assert twist.angular == 0.0


class TestTwistToWheels:
    def test_straight_forward(self):
        cmd = twist_to_wheels(0.2, 0.0, wheel_radius=0.04, wheelbase=0.22)
        assert cmd.left == pytest.approx(cmd.right, abs=0.01)
        assert cmd.left > 0

    def test_pure_rotation(self):
        cmd = twist_to_wheels(0.0, 1.0, wheel_radius=0.04, wheelbase=0.22)
        assert cmd.left < 0
        assert cmd.right > 0

    def test_clamped_to_max(self):
        cmd = twist_to_wheels(100.0, 0.0, wheel_radius=0.04, wheelbase=0.22, max_wheel_speed=10.0)
        assert cmd.left == 10.0
        assert cmd.right == 10.0
