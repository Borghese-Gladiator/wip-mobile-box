"""Tests for AvoidBehavior — exact v and w, turn direction always checked."""

import pytest

from robot_follower.behaviors.avoid import AvoidBehavior
from robot_follower.types.messages import ObstacleSectors


def _avoid(**kwargs) -> AvoidBehavior:
    defaults = dict(stop_distance=0.4, slow_distance=1.0, turn_bias=1.0)
    defaults.update(kwargs)
    return AvoidBehavior(**defaults)


def _sectors(front=5.0, fl=5.0, fr=5.0) -> ObstacleSectors:
    return ObstacleSectors(front=front, front_left=fl, front_right=fr)


# ---------------------------------------------------------------------------
# Clear — no obstacle in range
# ---------------------------------------------------------------------------

def test_clear_no_correction():
    cmd = _avoid().compute(_sectors())
    assert cmd.v == pytest.approx(0.0, abs=1e-6)
    assert cmd.w == pytest.approx(0.0, abs=1e-6)


# ---------------------------------------------------------------------------
# Hard stop (front <= stop_distance)
# Stop returns v=-0.2; turn toward clearer side.
# turn = turn_bias * (1.0 if fr > fl else -1.0)
# ---------------------------------------------------------------------------

def test_hard_stop_right_clearer():
    """fr=5 > fl=0.5 → turn = +1.0"""
    cmd = _avoid().compute(_sectors(front=0.3, fl=0.5, fr=5.0))
    assert cmd.v == pytest.approx(-0.2, abs=1e-6)
    assert cmd.w == pytest.approx(1.0, abs=1e-6)


def test_hard_stop_left_clearer():
    """fl=5 > fr=0.5 → fr>fl is False → turn = -1.0"""
    cmd = _avoid().compute(_sectors(front=0.3, fl=5.0, fr=0.5))
    assert cmd.v == pytest.approx(-0.2, abs=1e-6)
    assert cmd.w == pytest.approx(-1.0, abs=1e-6)


def test_hard_stop_equal_sides():
    """fl==fr → fr>fl is False → turn = -1.0"""
    cmd = _avoid().compute(_sectors(front=0.3, fl=5.0, fr=5.0))
    assert cmd.v == pytest.approx(-0.2, abs=1e-6)
    assert cmd.w == pytest.approx(-1.0, abs=1e-6)


def test_exactly_at_stop_distance_triggers_stop():
    """front == stop_distance (0.4): condition is <=, so stop fires."""
    cmd = _avoid().compute(_sectors(front=0.4, fl=5.0, fr=5.0))
    assert cmd.v == pytest.approx(-0.2, abs=1e-6)
    assert cmd.w == pytest.approx(-1.0, abs=1e-6)


# ---------------------------------------------------------------------------
# Slow zone (stop_distance < front < slow_distance)
# scale = (front - stop) / (slow - stop)
# delta_v = -(1 - scale) * 0.5
# delta_w = ±turn_bias * (1 - scale)
# ---------------------------------------------------------------------------

def test_slow_zone_midpoint_right_clearer():
    """
    front=0.7, stop=0.4, slow=1.0:
    scale = (0.7-0.4)/(1.0-0.4) = 0.3/0.6 = 0.5
    delta_v = -(1-0.5)*0.5 = -0.25
    fr=5 > fl=0 → delta_w = +1*(1-0.5) = +0.5
    """
    cmd = _avoid().compute(_sectors(front=0.7, fl=0.0, fr=5.0))
    assert cmd.v == pytest.approx(-0.25, abs=1e-6)
    assert cmd.w == pytest.approx(0.5, abs=1e-6)


def test_slow_zone_midpoint_left_clearer():
    """fl=5 > fr=0 → delta_w = -0.5"""
    cmd = _avoid().compute(_sectors(front=0.7, fl=5.0, fr=0.0))
    assert cmd.v == pytest.approx(-0.25, abs=1e-6)
    assert cmd.w == pytest.approx(-0.5, abs=1e-6)


def test_exactly_at_slow_distance_no_correction():
    """
    front == slow_distance: scale=1.0, delta_v=0, delta_w=0.
    """
    cmd = _avoid().compute(_sectors(front=1.0))
    assert cmd.v == pytest.approx(0.0, abs=1e-6)
    assert cmd.w == pytest.approx(0.0, abs=1e-6)


def test_turn_bias_scales_angular_output():
    """turn_bias=2.0: hard stop w = 2.0 * +1 = +2.0"""
    cmd = AvoidBehavior(stop_distance=0.4, slow_distance=1.0, turn_bias=2.0).compute(
        _sectors(front=0.3, fl=0.0, fr=5.0)
    )
    assert cmd.v == pytest.approx(-0.2, abs=1e-6)
    assert cmd.w == pytest.approx(2.0, abs=1e-6)
