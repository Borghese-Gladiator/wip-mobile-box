"""Tests for Arbiter — blend arithmetic, clamping, and estop boundaries."""

import pytest

from robot_follower.behaviors.arbiter import Arbiter
from robot_follower.types.messages import Twist, ObstacleSectors


def _arbiter(**kwargs) -> Arbiter:
    defaults = dict(estop_distance=0.3, max_linear=1.5, max_angular=4.0)
    defaults.update(kwargs)
    return Arbiter(**defaults)


def _clear() -> ObstacleSectors:
    return ObstacleSectors(front=10.0, front_left=10.0, front_right=10.0, rear=10.0)


def _front(dist: float) -> ObstacleSectors:
    return ObstacleSectors(front=dist, front_left=10.0, front_right=10.0, rear=10.0)


# ---------------------------------------------------------------------------
# Normal blend: output = follow + avoid, clamped
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("fv, fw, av, aw, exp_v, exp_w", [
    # Pass-through unchanged
    (1.0,  0.5,  0.0,  0.0,  1.0,  0.5),
    # Additive blend
    (1.0,  0.0, -0.3,  0.5,  0.7,  0.5),
    # Positive linear clamp
    (1.4,  0.0,  0.3,  0.0,  1.5,  0.0),
    # Negative linear clamp
    (-1.4, 0.0, -0.3,  0.0, -1.5,  0.0),
    # Positive angular clamp
    (0.0,  3.5,  0.0,  1.0,  0.0,  4.0),
    # Negative angular clamp
    (0.0, -3.5,  0.0, -1.0,  0.0, -4.0),
])
def test_blend(fv, fw, av, aw, exp_v, exp_w):
    cmd = _arbiter().blend(Twist(v=fv, w=fw), Twist(v=av, w=aw), _clear())
    assert cmd.v == pytest.approx(exp_v, abs=1e-9)
    assert cmd.w == pytest.approx(exp_w, abs=1e-9)


# ---------------------------------------------------------------------------
# Emergency stop
# ---------------------------------------------------------------------------

def test_estop_overrides_all():
    cmd = _arbiter().blend(Twist(v=1.0, w=0.5), Twist(v=0.0, w=0.0), _front(0.2))
    assert cmd.v == pytest.approx(0.0, abs=1e-9)
    assert cmd.w == pytest.approx(0.0, abs=1e-9)


def test_estop_at_exact_boundary_stops():
    """front == estop_distance: condition is <=, must stop."""
    cmd = _arbiter().blend(Twist(v=1.0, w=0.0), Twist(), _front(0.3))
    assert cmd.v == pytest.approx(0.0, abs=1e-9)
    assert cmd.w == pytest.approx(0.0, abs=1e-9)


def test_estop_just_above_boundary_passes_through():
    """front = 0.31 > 0.30: estop does not fire, command passes through."""
    cmd = _arbiter().blend(Twist(v=1.0, w=0.0), Twist(), _front(0.31))
    assert cmd.v == pytest.approx(1.0, abs=1e-9)
    assert cmd.w == pytest.approx(0.0, abs=1e-9)
