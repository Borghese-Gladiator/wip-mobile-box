"""
TDD — DiffDriveOdometry.

Formula per step:
  d      = (left + right) / 2
  dtheta = (right - left) / track_width
  x     += d * cos(theta + dtheta / 2)
  y     += d * sin(theta + dtheta / 2)
  theta  = wrap(theta + dtheta)
  v      = d / dt
  w      = dtheta / dt
"""

import math
import pytest

from robot_follower.estimation.odom import DiffDriveOdometry


def _odom(track: float = 0.5, **kwargs) -> DiffDriveOdometry:
    return DiffDriveOdometry(track_width=track, **kwargs)


# ---------------------------------------------------------------------------
# Single-step pose integration
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("left, right, init_th, track, exp_x, exp_y, exp_th", [
    # Straight forward: d=0.1, dtheta=0, x+=0.1*cos(0)=0.1, y+=0
    (0.1,  0.1,  0.0, 0.5,  0.1,    0.0,   0.0),
    # Straight backward
    (-0.1, -0.1, 0.0, 0.5, -0.1,    0.0,   0.0),
    # Spin in place left: d=0, dtheta=0.2/0.5=0.4, x=0, y=0, theta=0.4
    (-0.1, 0.1,  0.0, 0.5,  0.0,    0.0,   0.4),
    # Spin in place right
    (0.1,  -0.1, 0.0, 0.5,  0.0,    0.0,  -0.4),
    # Arc left: left=0, right=0.2, d=0.1, dtheta=0.4
    #   x += 0.1*cos(0 + 0.2) = 0.1*cos(0.2)
    #   y += 0.1*sin(0 + 0.2) = 0.1*sin(0.2)
    (0.0,  0.2,  0.0, 0.5,
     0.1 * math.cos(0.2),
     0.1 * math.sin(0.2),
     0.4),
    # Straight forward at 45° heading
    #   d=0.1, dtheta=0
    #   x += 0.1*cos(π/4), y += 0.1*sin(π/4)
    (0.1,  0.1,  math.pi / 4, 0.5,
     0.1 * math.cos(math.pi / 4),
     0.1 * math.sin(math.pi / 4),
     math.pi / 4),
])
def test_single_step(left, right, init_th, track, exp_x, exp_y, exp_th):
    odom = _odom(track=track, initial_theta=init_th)
    result = odom.update(left, right, dt=0.1)
    assert result.x     == pytest.approx(exp_x,  abs=1e-6)
    assert result.y     == pytest.approx(exp_y,  abs=1e-6)
    assert result.theta == pytest.approx(exp_th, abs=1e-6)


# ---------------------------------------------------------------------------
# Velocity output
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("left, right, dt, exp_v, exp_w", [
    # Straight: d=0.1, v=0.1/0.1=1.0, w=0
    (0.1,  0.1,  0.1, 1.0,  0.0),
    # Spin: d=0, dtheta=0.4, v=0, w=0.4/0.1=4.0
    (-0.1, 0.1,  0.1, 0.0,  4.0),
    # Arc: d=0.1, dtheta=0.4, v=1.0, w=4.0
    (0.0,  0.2,  0.1, 1.0,  4.0),
])
def test_velocity_output(left, right, dt, exp_v, exp_w):
    odom = _odom()
    result = odom.update(left, right, dt=dt)
    assert result.v == pytest.approx(exp_v, abs=1e-6)
    assert result.w == pytest.approx(exp_w, abs=1e-6)


# ---------------------------------------------------------------------------
# Angle wrapping
# ---------------------------------------------------------------------------

def test_angle_wraps_past_pi():
    """
    Spin left enough to cross +π: theta should wrap to near -π, not exceed π.
    10 steps of dtheta=0.4 → total=4.0 rad > π.
    After wrap: (4.0 + π) % (2π) - π = 4.0 - 2π ≈ -2.283
    """
    odom = _odom()
    for _ in range(10):
        odom.update(-0.1, 0.1, dt=0.1)
    assert -math.pi <= odom.theta <= math.pi


def test_angle_wraps_past_minus_pi():
    """Spin right past -π: theta stays in (-π, π]."""
    odom = _odom()
    for _ in range(10):
        odom.update(0.1, -0.1, dt=0.1)
    assert -math.pi <= odom.theta <= math.pi


# ---------------------------------------------------------------------------
# Multi-step dead reckoning
# ---------------------------------------------------------------------------

def test_ten_steps_straight_accumulates_distance():
    """
    10 steps of left=right=0.1 → total x = 1.0, y = 0, theta = 0.
    """
    odom = _odom()
    for _ in range(10):
        result = odom.update(0.1, 0.1, dt=0.1)
    assert result.x     == pytest.approx(1.0, abs=1e-6)
    assert result.y     == pytest.approx(0.0, abs=1e-6)
    assert result.theta == pytest.approx(0.0, abs=1e-6)


def test_initial_position_offset():
    """Odometry integrates correctly from a non-zero starting pose."""
    odom = DiffDriveOdometry(track_width=0.5, initial_x=1.0, initial_y=2.0, initial_theta=0.0)
    result = odom.update(0.1, 0.1, dt=0.1)
    assert result.x == pytest.approx(1.1, abs=1e-6)
    assert result.y == pytest.approx(2.0, abs=1e-6)
