"""Tests for FollowBehavior — exact computed values, w always checked."""

import math
import pytest
import numpy as np

from robot_follower.behaviors.follow import FollowBehavior
from robot_follower.types.messages import PersonTarget


def _follow(**kwargs) -> FollowBehavior:
    """Factory with BLE disabled for deterministic output."""
    defaults = dict(
        ble_latency_seconds=0.0,
        ble_packet_drop_rate=0.0,
        dt=0.05,
        kp_dist=1.0,
        kp_angle=3.0,
        max_linear_cmd=1.0,
        desired_distance=1.0,
    )
    defaults.update(kwargs)
    return FollowBehavior(**defaults)


def _target(x: float, y: float) -> PersonTarget:
    return PersonTarget(position=np.array([x, y], dtype=float), confidence=1.0)


def _pose(x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> np.ndarray:
    return np.array([x, y, theta], dtype=float)


# ---------------------------------------------------------------------------
# Straight-line cases
# ---------------------------------------------------------------------------

def test_straight_ahead_far_clamped():
    """
    target=(5,0), pose=(0,0,0): dist=5, v=min(1*5,1)=1.0, angle_err=0 → w=0.
    """
    cmd = _follow().compute(_target(5.0, 0.0), _pose())
    assert cmd.v == pytest.approx(1.0, abs=1e-6)
    assert cmd.w == pytest.approx(0.0, abs=1e-6)


def test_within_desired_distance_stops():
    """
    dist=0.5 < desired_distance=1.0 → Twist(0,0).
    """
    cmd = _follow().compute(_target(0.5, 0.0), _pose())
    assert cmd.v == pytest.approx(0.0, abs=1e-6)
    assert cmd.w == pytest.approx(0.0, abs=1e-6)


def test_unclamped_speed():
    """
    target=(0.3,0), desired_distance=0, kp_dist=2, max=10:
    dist=0.3, v=min(2*0.3,10)=0.6, w=0.
    """
    cmd = _follow(desired_distance=0.0, kp_dist=2.0, max_linear_cmd=10.0).compute(
        _target(0.3, 0.0), _pose()
    )
    assert cmd.v == pytest.approx(0.6, abs=1e-6)
    assert cmd.w == pytest.approx(0.0, abs=1e-6)


def test_max_linear_cmd_clamp():
    """
    kp_dist=5, dist=10 → raw v=50, clamped to max_linear_cmd=2.0.
    """
    cmd = _follow(kp_dist=5.0, max_linear_cmd=2.0, desired_distance=0.0).compute(
        _target(10.0, 0.0), _pose()
    )
    assert cmd.v == pytest.approx(2.0, abs=1e-6)


# ---------------------------------------------------------------------------
# Angular cases
# ---------------------------------------------------------------------------

def test_target_left_90_degrees():
    """
    target=(0,3), pose facing +x (theta=0):
    angle_err = wrap(π/2 - 0) = π/2
    w = 3 * π/2 ≈ 4.71239
    v = min(1*3, 1) = 1.0
    """
    cmd = _follow().compute(_target(0.0, 3.0), _pose())
    assert cmd.v == pytest.approx(1.0, abs=1e-6)
    assert cmd.w == pytest.approx(3 * math.pi / 2, abs=1e-6)


def test_target_right_90_degrees():
    """
    target=(0,-3): angle_err = wrap(-π/2) = -π/2
    w = 3 * (-π/2) ≈ -4.71239
    """
    cmd = _follow().compute(_target(0.0, -3.0), _pose())
    assert cmd.v == pytest.approx(1.0, abs=1e-6)
    assert cmd.w == pytest.approx(-3 * math.pi / 2, abs=1e-6)


def test_target_directly_behind():
    """
    target=(-3,0): desired_angle=π, angle_err=wrap(π-0)=±π (float boundary).
    The magnitude must be 3π; sign is implementation-defined at exactly ±π.
    """
    cmd = _follow().compute(_target(-3.0, 0.0), _pose())
    assert cmd.v == pytest.approx(1.0, abs=1e-6)
    assert abs(cmd.w) == pytest.approx(3 * math.pi, abs=1e-6)


def test_robot_rotated_target_ahead():
    """
    Robot at theta=π/4, target at (1,0):
    desired_angle = 0
    angle_err = wrap(0 - π/4) = -π/4
    w = 3 * (-π/4) ≈ -2.35619
    dist = 1, v = min(1*1, 1) = 1.0
    """
    cmd = _follow().compute(_target(1.0, 0.0), _pose(theta=math.pi / 4))
    assert cmd.v == pytest.approx(1.0, abs=1e-6)
    assert cmd.w == pytest.approx(-3 * math.pi / 4, abs=1e-6)
