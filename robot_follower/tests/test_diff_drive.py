"""
TDD — DiffDriveKinematics.

Formulas:
  left  = v - w * (track/2)     clamped to ±max_speed
  right = v + w * (track/2)     clamped to ±max_speed
  v     = (left + right) / 2    (inverse)
  w     = (right - left) / track (inverse)
"""

import pytest

from robot_follower.control.diff_drive import DiffDriveKinematics
from robot_follower.types.messages import Twist


def _kin(track: float = 0.5, max_speed: float = 2.0) -> DiffDriveKinematics:
    return DiffDriveKinematics(track_width=track, max_speed=max_speed)


# ---------------------------------------------------------------------------
# twist_to_wheels
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("v, w, track, max_spd, exp_left, exp_right", [
    # Straight forward: left=right=v
    (1.0,  0.0, 0.5, 2.0,  1.0,  1.0),
    # Spin left (w>0): left=-w*half, right=+w*half
    (0.0,  2.0, 0.5, 2.0, -0.5,  0.5),
    # Spin right (w<0)
    (0.0, -2.0, 0.5, 2.0,  0.5, -0.5),
    # Combined: v=1, w=2, half=0.25 → left=0.5, right=1.5
    (1.0,  2.0, 0.5, 2.0,  0.5,  1.5),
    # Left wheel unclamped, right exactly at max: v=1,w=4 → left=0.0, right=2.0
    (1.0,  4.0, 0.5, 2.0,  0.0,  2.0),
    # Right wheel would exceed max: v=2,w=4 → right=3→clamp 2, left=1
    (2.0,  4.0, 0.5, 2.0,  1.0,  2.0),
    # Reverse + spin: v=-1,w=-2 → left=-1+0.5=-0.5, right=-1-0.5=-1.5
    (-1.0, -2.0, 0.5, 2.0, -0.5, -1.5),
    # Straight reverse
    (-1.0,  0.0, 0.5, 2.0, -1.0, -1.0),
])
def test_twist_to_wheels(v, w, track, max_spd, exp_left, exp_right):
    left, right = _kin(track, max_spd).twist_to_wheels(Twist(v=v, w=w))
    assert left  == pytest.approx(exp_left,  abs=1e-6)
    assert right == pytest.approx(exp_right, abs=1e-6)


# ---------------------------------------------------------------------------
# wheels_to_twist (forward kinematics)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("left, right, track, exp_v, exp_w", [
    # Straight
    (1.0,  1.0, 0.5, 1.0,  0.0),
    # Arc: l=0.5, r=1.5 → v=1, w=(1.5-0.5)/0.5=2.0
    (0.5,  1.5, 0.5, 1.0,  2.0),
    # Spin in place: l=-0.5, r=0.5 → v=0, w=2.0
    (-0.5, 0.5, 0.5, 0.0,  2.0),
    # Reverse straight
    (-1.0, -1.0, 0.5, -1.0, 0.0),
])
def test_wheels_to_twist(left, right, track, exp_v, exp_w):
    twist = _kin(track).wheels_to_twist(left, right)
    assert twist.v == pytest.approx(exp_v, abs=1e-6)
    assert twist.w == pytest.approx(exp_w, abs=1e-6)


# ---------------------------------------------------------------------------
# Roundtrip: twist → wheels → twist
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("v, w", [
    (1.0,  0.0),
    (0.5,  1.5),
    (0.0,  2.0),
    (-0.5, -1.0),
])
def test_roundtrip_unclamped(v, w):
    """wheels_to_twist(twist_to_wheels(t)) == t when no clamping occurs."""
    kin = _kin(track=0.5, max_speed=10.0)
    left, right = kin.twist_to_wheels(Twist(v=v, w=w))
    recovered = kin.wheels_to_twist(left, right)
    assert recovered.v == pytest.approx(v, abs=1e-6)
    assert recovered.w == pytest.approx(w, abs=1e-6)
