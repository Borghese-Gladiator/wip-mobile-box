"""Tests for FollowBehavior."""

import math
import pytest
import numpy as np

from robot_follower.behaviors.follow import FollowBehavior
from robot_follower.types.messages import PersonTarget


def _make_target(x: float, y: float, confidence: float = 1.0) -> PersonTarget:
    return PersonTarget(position=np.array([x, y]), confidence=confidence)


@pytest.mark.parametrize("tx, ty, rx, ry, rth, expect_moving", [
    # Far away — should command forward motion
    (5.0,  0.0,  0.0, 0.0, 0.0,  True),
    # Within desired distance — should stop
    (0.5,  0.0,  0.0, 0.0, 0.0,  False),
])
def test_follow_command(tx, ty, rx, ry, rth, expect_moving):
    follow = FollowBehavior(
        desired_distance=1.0,
        ble_latency_seconds=0.0,  # no delay for deterministic test
        ble_packet_drop_rate=0.0,
        dt=0.05,
    )
    target = _make_target(tx, ty)
    pose = np.array([rx, ry, rth])

    cmd = follow.compute(target, pose)

    if expect_moving:
        assert cmd.v > 0, "expected forward velocity"
    else:
        assert cmd.v == pytest.approx(0.0)
        assert cmd.w == pytest.approx(0.0)


def test_angular_correction_for_off_axis_target():
    """Target to the left of the robot should produce positive w."""
    follow = FollowBehavior(
        desired_distance=0.5,
        ble_latency_seconds=0.0,
        ble_packet_drop_rate=0.0,
        dt=0.05,
    )
    target = _make_target(0.0, 3.0)          # directly left (robot faces +x)
    pose = np.array([0.0, 0.0, 0.0])

    cmd = follow.compute(target, pose)

    assert cmd.w > 0, "should turn left (positive w) toward left target"
