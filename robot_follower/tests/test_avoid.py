"""Tests for AvoidBehavior."""

import pytest

from robot_follower.behaviors.avoid import AvoidBehavior
from robot_follower.types.messages import ObstacleSectors


@pytest.mark.parametrize("front, fl, fr, expect_stop, expect_decel", [
    # Clear — no correction
    (5.0, 5.0, 5.0, False, False),
    # Hard stop
    (0.3, 5.0, 5.0, True,  False),
    # Slow zone
    (0.7, 5.0, 5.0, False, True),
])
def test_avoid_command(front, fl, fr, expect_stop, expect_decel):
    avoid = AvoidBehavior(stop_distance=0.4, slow_distance=1.0)
    sectors = ObstacleSectors(front=front, front_left=fl, front_right=fr)

    cmd = avoid.compute(sectors)

    if expect_stop:
        assert cmd.v < 0, "hard stop should produce negative v"
    elif expect_decel:
        assert cmd.v < 0, "slow zone should reduce v"
    else:
        assert cmd.v == pytest.approx(0.0)
        assert cmd.w == pytest.approx(0.0)
