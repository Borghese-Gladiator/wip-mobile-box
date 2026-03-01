"""
TDD — PotentialFieldObstacles (perception/obstacles.py).

The class returns (delta_v, delta_w) repulsion corrections.

Key formulas:
  vec       = robot_pos - obs_pos
  dist      = norm(vec)
  direction = vec / (dist + 1e-5)
  strength  = REPULSION_STRENGTH / (dist**2 + 0.1)   [only if dist < radius * FACTOR]
  force    += direction * strength

  desired_angle = atan2(force[1], force[0])
  angle_error   = wrap(desired_angle - robot_theta)
  delta_w  = clip(2.0 * angle_error, -2.0,  2.0)
  delta_v  = clip(-0.5 * norm(force), -1.0, 0.0)

Constants: REPULSION_STRENGTH=3.0, INFLUENCE_RADIUS_FACTOR=2.0
"""

import math
import numpy as np
import pytest

from robot_follower.perception.obstacles import PotentialFieldObstacles


def _pf(obstacles, radius: float = 1.0) -> PotentialFieldObstacles:
    return PotentialFieldObstacles(obstacles=obstacles, radius=radius)


def _state(x: float, y: float, theta: float = 0.0) -> np.ndarray:
    return np.array([x, y, theta], dtype=float)


# ---------------------------------------------------------------------------
# Outside influence radius — no force
# ---------------------------------------------------------------------------

def test_outside_influence_radius_no_correction():
    """
    Single obstacle at origin, radius=1.0, FACTOR=2.0 → influence < 2.0 m.
    Robot at (3, 0): dist=3 >= 2 → no repulsion.
    """
    dv, dw = _pf([[0.0, 0.0]]).avoidance_command(_state(3.0, 0.0))
    assert dv == pytest.approx(0.0, abs=1e-9)
    assert dw == pytest.approx(0.0, abs=1e-9)


def test_exactly_at_influence_boundary_no_correction():
    """dist == radius * FACTOR → condition is <, so no repulsion."""
    dv, dw = _pf([[0.0, 0.0]], radius=1.0).avoidance_command(_state(2.0, 0.0))
    assert dv == pytest.approx(0.0, abs=1e-9)
    assert dw == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# Inside influence radius — repulsion fires
# ---------------------------------------------------------------------------

def test_inside_influence_radius_produces_negative_dv():
    """Robot inside influence radius must get a braking correction (dv < 0)."""
    dv, dw = _pf([[0.0, 0.0]]).avoidance_command(_state(1.0, 0.0))
    assert dv < 0.0


def test_obstacle_directly_behind_robot_no_angular_correction():
    """
    Obstacle at (0,0), robot at (1,0) facing +x (theta=0).
    Force points in +x direction (away from obs).
    desired_angle = 0, angle_error = wrap(0-0) = 0 → dw = 0.
    """
    dv, dw = _pf([[0.0, 0.0]]).avoidance_command(_state(1.0, 0.0, theta=0.0))
    assert dw == pytest.approx(0.0, abs=1e-6)
    assert dv < 0.0


def test_obstacle_to_the_right_robot_turns_left():
    """
    Obstacle at (0,-1), robot at (0,0) facing +x (theta=0).
    Force points in +y direction (away from obstacle below).
    desired_angle = π/2, angle_error = π/2 → dw = clip(2*π/2, -2, 2) = 2.0.
    """
    dv, dw = _pf([[0.0, -1.0]], radius=2.0).avoidance_command(_state(0.0, 0.0, theta=0.0))
    assert dw == pytest.approx(2.0, abs=1e-6)
    assert dv < 0.0


def test_obstacle_to_the_left_robot_turns_right():
    """
    Obstacle at (0,1), robot at (0,0) facing +x.
    Force points in -y direction → desired_angle = -π/2.
    angle_error = -π/2 → dw = clip(-π, -2, 2) = -2.0.
    """
    dv, dw = _pf([[0.0, 1.0]], radius=2.0).avoidance_command(_state(0.0, 0.0, theta=0.0))
    assert dw == pytest.approx(-2.0, abs=1e-6)
    assert dv < 0.0


# ---------------------------------------------------------------------------
# delta_v is always <= 0 (braking only, never acceleration)
# ---------------------------------------------------------------------------

def test_dv_never_positive():
    """Repulsion must only slow, never accelerate."""
    pf = _pf([[0.0, 0.0]], radius=2.0)
    for pos in [(0.5, 0.0), (0.0, 0.5), (-0.5, 0.0), (0.0, -0.5), (1.0, 1.0)]:
        dv, _ = pf.avoidance_command(_state(*pos))
        assert dv <= 0.0, f"dv={dv} > 0 at position {pos}"


# ---------------------------------------------------------------------------
# delta_v and delta_w clamping
# ---------------------------------------------------------------------------

def test_dv_clamped_to_minus_one():
    """Very close to obstacle: delta_v must not go below -1.0."""
    dv, _ = _pf([[0.0, 0.0]], radius=2.0).avoidance_command(_state(0.01, 0.0))
    assert dv >= -1.0


def test_dw_clamped_to_plus_two():
    """delta_w must not exceed +2.0."""
    _, dw = _pf([[0.0, -1.0]], radius=2.0).avoidance_command(_state(0.0, 0.0))
    assert dw <= 2.0


def test_dw_clamped_to_minus_two():
    """delta_w must not go below -2.0."""
    _, dw = _pf([[0.0, 1.0]], radius=2.0).avoidance_command(_state(0.0, 0.0))
    assert dw >= -2.0


# ---------------------------------------------------------------------------
# Multiple obstacles: forces sum
# ---------------------------------------------------------------------------

def test_two_obstacles_same_side_stronger_repulsion():
    """
    Two obstacles both below the robot: combined force in +y is stronger
    than one obstacle alone → |dw| should be at least as large.
    """
    robot = _state(0.0, 0.0, theta=0.0)
    _, dw_one = _pf([[0.0, -0.5]], radius=2.0).avoidance_command(robot)
    _, dw_two = _pf([[0.0, -0.5], [0.2, -0.5]], radius=2.0).avoidance_command(robot)
    # Both push in same direction; two-obstacle correction should be >= one
    assert dw_two >= dw_one


def test_two_opposing_obstacles_partially_cancel():
    """
    Obstacle above and below the robot at equal distances.
    Forces cancel in y → near-zero dw.
    """
    robot = _state(0.0, 0.0, theta=0.0)
    dv, dw = _pf([[0.0, 1.0], [0.0, -1.0]], radius=2.0).avoidance_command(robot)
    assert abs(dw) < 1e-4
