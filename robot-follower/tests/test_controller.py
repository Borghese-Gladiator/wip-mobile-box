"""Tests for follower controller and wheel controller."""

import math
import pytest
from robot_follower.core.types import TargetMeasurement, Twist2D
from robot_follower.core.follower_controller import FollowerController
from robot_follower.core.wheel_controller import WheelController


@pytest.fixture
def follower():
    return FollowerController(
        base_speed=3.0, steering_gain=6.0, follow_distance=1.5, dead_zone=0.2,
    )


@pytest.fixture
def wheel_ctrl():
    return WheelController(wheel_radius=0.04, wheelbase=0.22, max_wheel_speed=10.0)


class TestFollowerController:
    def test_target_ahead_far_gives_forward(self, follower):
        t = TargetMeasurement(signal_strength=0.5, bearing=0.0, distance_estimate=4.0, valid=True)
        twist = follower.compute(t)
        assert twist.linear > 1.0
        assert abs(twist.angular) < 0.1

    def test_target_left_gives_left_turn(self, follower):
        t = TargetMeasurement(signal_strength=0.5, bearing=0.5, distance_estimate=3.0, valid=True)
        twist = follower.compute(t)
        assert twist.angular > 1.0

    def test_target_right_gives_right_turn(self, follower):
        t = TargetMeasurement(signal_strength=0.5, bearing=-0.5, distance_estimate=3.0, valid=True)
        twist = follower.compute(t)
        assert twist.angular < -1.0

    def test_invalid_target_returns_zero(self, follower):
        t = TargetMeasurement(valid=False)
        twist = follower.compute(t)
        assert twist.linear == 0.0
        assert twist.angular == 0.0

    def test_stale_target_returns_zero(self, follower):
        t = TargetMeasurement(valid=True, stale=True, bearing=0.3, signal_strength=0.5)
        twist = follower.compute(t)
        assert twist.linear == 0.0

    @pytest.mark.parametrize("distance,expect_positive", [
        (4.0, True),     # far away → approach
        (1.5, False),    # at follow distance → dead zone, no forward
        (0.5, False),    # too close → back up or stop
    ])
    def test_distance_error_response(self, follower, distance, expect_positive):
        t = TargetMeasurement(
            signal_strength=0.5, bearing=0.0, distance_estimate=distance, valid=True,
        )
        twist = follower.compute(t)
        if expect_positive:
            assert twist.linear > 0.5
        else:
            assert twist.linear <= 0.0

    def test_stops_in_dead_zone(self, follower):
        """Robot should have zero forward speed when within dead zone of follow distance."""
        for dist in [1.4, 1.5, 1.6]:
            t = TargetMeasurement(
                signal_strength=0.5, bearing=0.0, distance_estimate=dist, valid=True,
            )
            twist = follower.compute(t)
            assert twist.linear == 0.0, f"Expected zero forward at dist={dist}"

    def test_derivative_dampens_approach(self):
        """Rapidly closing distance should produce less forward than steady error."""
        # Use higher base_speed so PD output doesn't saturate at the clamp
        ctrl = FollowerController(
            base_speed=20.0, steering_gain=6.0, follow_distance=1.5, dead_zone=0.2,
        )
        far = TargetMeasurement(
            signal_strength=0.3, bearing=0.0, distance_estimate=3.0, valid=True,
        )
        closer = TargetMeasurement(
            signal_strength=0.5, bearing=0.0, distance_estimate=2.5, valid=True,
        )
        # First call seeds prev_error
        ctrl.compute(far, dt=0.02)
        # Second call: distance dropped 0.5m in 0.02s → negative derivative
        dampened = ctrl.compute(closer, dt=0.02)

        # Compare with a fresh controller seeing distance=2.5 with no history
        fresh = FollowerController(
            base_speed=20.0, steering_gain=6.0, follow_distance=1.5, dead_zone=0.2,
        )
        undampened = fresh.compute(closer, dt=0.02)

        assert dampened.linear < undampened.linear

    def test_negative_forward_when_too_close(self, follower):
        """Robot should produce negative (reverse) speed when much too close."""
        t = TargetMeasurement(
            signal_strength=0.9, bearing=0.0, distance_estimate=0.3, valid=True,
        )
        twist = follower.compute(t)
        assert twist.linear < 0.0


class TestWheelController:
    @pytest.mark.parametrize("lin,ang,left_gt_right", [
        (0.1, 0.0, None),    # straight: equal
        (0.1, 1.0, False),   # turn left: right > left
        (0.1, -1.0, True),   # turn right: left > right
    ])
    def test_mixing(self, wheel_ctrl, lin, ang, left_gt_right):
        cmd = wheel_ctrl.compute(Twist2D(linear=lin, angular=ang))
        if left_gt_right is None:
            assert cmd.left == pytest.approx(cmd.right, abs=0.1)
        elif left_gt_right:
            assert cmd.left > cmd.right
        else:
            assert cmd.right > cmd.left

    def test_velocity_clamped(self, wheel_ctrl):
        cmd = wheel_ctrl.compute(Twist2D(linear=100.0, angular=50.0))
        assert -10.0 <= cmd.left <= 10.0
        assert -10.0 <= cmd.right <= 10.0
