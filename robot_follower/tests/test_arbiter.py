"""Tests for Arbiter blending and safety override."""

import pytest

from robot_follower.behaviors.arbiter import Arbiter
from robot_follower.types.messages import Twist, ObstacleSectors


def _clear_sectors() -> ObstacleSectors:
    return ObstacleSectors(front=10.0, front_left=10.0, front_right=10.0, rear=10.0)


def _close_sectors(front: float = 0.2) -> ObstacleSectors:
    return ObstacleSectors(front=front, front_left=10.0, front_right=10.0, rear=10.0)


@pytest.mark.parametrize("follow_v, follow_w, avoid_v, avoid_w, expected_v, expected_w", [
    # Normal blend: outputs are summed and clamped
    (1.0,  0.5, 0.0,  0.0, 1.0,  0.5),
    (1.0,  0.0, -0.3, 0.5, 0.7,  0.5),
    # Clamped to max_linear=1.5
    (1.4,  0.0, 0.3,  0.0, 1.5,  0.0),
])
def test_blend_normal(follow_v, follow_w, avoid_v, avoid_w, expected_v, expected_w):
    arbiter = Arbiter(estop_distance=0.3, max_linear=1.5, max_angular=4.0)
    cmd = arbiter.blend(
        Twist(v=follow_v, w=follow_w),
        Twist(v=avoid_v,  w=avoid_w),
        _clear_sectors(),
    )
    assert cmd.v == pytest.approx(expected_v, abs=1e-9)
    assert cmd.w == pytest.approx(expected_w, abs=1e-9)


def test_estop_overrides_all():
    arbiter = Arbiter(estop_distance=0.3)
    cmd = arbiter.blend(
        Twist(v=1.0, w=0.5),
        Twist(v=0.0, w=0.0),
        _close_sectors(front=0.2),
    )
    assert cmd.v == pytest.approx(0.0)
    assert cmd.w == pytest.approx(0.0)
