"""
TDD — AccelLimiter.

Formula per step:
  max_dv = max_accel * dt
  max_dw = max_alpha * dt
  dv     = clip(desired.v - prev.v, ±max_dv)
  dw     = clip(desired.w - prev.w, ±max_dw)
  out    = Twist(prev.v + dv, prev.w + dw)

With max_accel=1.0, max_alpha=2.0, dt=0.05:
  max_dv = 0.05,  max_dw = 0.10
"""

import pytest

from robot_follower.control.limits import AccelLimiter
from robot_follower.types.messages import Twist


def _limiter(**kwargs) -> AccelLimiter:
    defaults = dict(max_accel=1.0, max_alpha=2.0, dt=0.05)
    defaults.update(kwargs)
    return AccelLimiter(**defaults)


# ---------------------------------------------------------------------------
# Single-step cases (prev = 0,0)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("des_v, des_w, exp_v, exp_w", [
    # Within limits: pass through unchanged
    (0.02,  0.05,  0.02,  0.05),
    # Exactly at limits: pass through
    (0.05,  0.10,  0.05,  0.10),
    # v clamped, w zero
    (1.00,  0.00,  0.05,  0.00),
    # w clamped, v zero
    (0.00,  2.00,  0.00,  0.10),
    # Both clamped
    (1.00,  2.00,  0.05,  0.10),
    # Negative v clamped
    (-1.00, 0.00, -0.05,  0.00),
    # Negative w clamped
    (0.00, -2.00,  0.00, -0.10),
])
def test_single_step_from_zero(des_v, des_w, exp_v, exp_w):
    lim = _limiter()
    out = lim.apply(Twist(v=des_v, w=des_w))
    assert out.v == pytest.approx(exp_v, abs=1e-9)
    assert out.w == pytest.approx(exp_w, abs=1e-9)


# ---------------------------------------------------------------------------
# Multi-step ramp
# ---------------------------------------------------------------------------

def test_ramp_three_steps():
    """
    Each step clamps at max_dv=0.05: v should accumulate 0.05, 0.10, 0.15.
    """
    lim = _limiter()
    results = [lim.apply(Twist(v=1.0, w=0.0)).v for _ in range(3)]
    assert results[0] == pytest.approx(0.05, abs=1e-9)
    assert results[1] == pytest.approx(0.10, abs=1e-9)
    assert results[2] == pytest.approx(0.15, abs=1e-9)


def test_ramp_reaches_target_eventually():
    """After enough steps (1.0/0.05 = 20) v should equal desired."""
    lim = _limiter()
    out = None
    for _ in range(20):
        out = lim.apply(Twist(v=1.0, w=0.0))
    assert out.v == pytest.approx(1.0, abs=1e-9)


# ---------------------------------------------------------------------------
# Deceleration
# ---------------------------------------------------------------------------

def test_decel_from_positive_velocity():
    """
    Set prev to v=0.5 by ramping, then command v=0.
    Each step decrements by max_dv=0.05.
    """
    lim = _limiter()
    for _ in range(10):          # ramp up to 0.50
        lim.apply(Twist(v=1.0))
    out = lim.apply(Twist(v=0.0))
    assert out.v == pytest.approx(0.45, abs=1e-9)


# ---------------------------------------------------------------------------
# Reset
# ---------------------------------------------------------------------------

def test_reset_clears_state():
    """After reset, limiter behaves as if starting from zero."""
    lim = _limiter()
    for _ in range(10):
        lim.apply(Twist(v=1.0))   # prev.v is now 0.5

    lim.reset()
    out = lim.apply(Twist(v=1.0))
    assert out.v == pytest.approx(0.05, abs=1e-9)
    assert out.w == pytest.approx(0.00, abs=1e-9)
