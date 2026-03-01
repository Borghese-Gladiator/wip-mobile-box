"""
TDD — sectorize() in perception/obstacles.py.

Default sector boundaries (after _wrap_deg):
  front:       330° to  30°  (wraps through 0°)
  front_left:   30° to  90°
  front_right: 270° to 330°
  rear:        150° to 210°

Boundary membership: lo_w <= hi_w  →  angles >= lo_w AND angles < hi_w
                     wrapping      →  angles >= lo_w OR  angles < hi_w

The front sector wraps: lo=330, hi=30  → angles>=330 OR angles<30.
So angle=30° does NOT match front (not <30), but matches front_left (30≤30<90 ✓).
"""

import math
import pytest
import numpy as np

from robot_follower.perception.obstacles import sectorize


def _scan(angles, ranges) -> tuple[np.ndarray, np.ndarray]:
    return np.array(angles, dtype=float), np.array(ranges, dtype=float)


# ---------------------------------------------------------------------------
# Single-point routing to correct sector
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("angle, rng, exp_front, exp_fl, exp_fr, exp_rear", [
    # Dead ahead (10°): front sector (330–30 wrapping)
    (10,  2.0, 2.0,  math.inf, math.inf, math.inf),
    # Dead ahead using 350° (also in front sector: 350>=330)
    (350, 2.0, 2.0,  math.inf, math.inf, math.inf),
    # Front-left (45°)
    (45,  1.5, math.inf, 1.5, math.inf, math.inf),
    # Front-right (315°): 315 >= 270 and 315 < 330 ✓
    (315, 3.0, math.inf, math.inf, 3.0, math.inf),
    # Rear (180°)
    (180, 4.0, math.inf, math.inf, math.inf, 4.0),
])
def test_single_point_routes_to_correct_sector(angle, rng, exp_front, exp_fl, exp_fr, exp_rear):
    sectors = sectorize(*_scan([angle], [rng]))
    assert sectors.front       == pytest.approx(exp_front, abs=1e-6)
    assert sectors.front_left  == pytest.approx(exp_fl,    abs=1e-6)
    assert sectors.front_right == pytest.approx(exp_fr,    abs=1e-6)
    assert sectors.rear        == pytest.approx(exp_rear,  abs=1e-6)


# ---------------------------------------------------------------------------
# Boundary: 30° is front_left NOT front
# ---------------------------------------------------------------------------

def test_boundary_30_deg_goes_to_front_left_not_front():
    """
    angle=30: front needs <30 → False. front_left needs 30<=30<90 → True.
    """
    sectors = sectorize(*_scan([30], [1.0]))
    assert sectors.front      == pytest.approx(math.inf, abs=1e-6)
    assert sectors.front_left == pytest.approx(1.0,      abs=1e-6)


def test_boundary_330_deg_goes_to_front_not_front_right():
    """
    angle=330: front_right needs 270<=330<330 → False (not <330).
    front needs 330>=330 → True.
    """
    sectors = sectorize(*_scan([330], [1.0]))
    assert sectors.front       == pytest.approx(1.0,      abs=1e-6)
    assert sectors.front_right == pytest.approx(math.inf, abs=1e-6)


# ---------------------------------------------------------------------------
# Minimum selection within a sector
# ---------------------------------------------------------------------------

def test_front_sector_returns_minimum():
    """Two readings in front (5° and 25°): sector returns the closer one."""
    sectors = sectorize(*_scan([5, 25], [3.0, 1.0]))
    assert sectors.front == pytest.approx(1.0, abs=1e-6)


# ---------------------------------------------------------------------------
# Empty scan
# ---------------------------------------------------------------------------

def test_empty_scan_all_inf():
    sectors = sectorize(np.array([]), np.array([]))
    assert sectors.front       == math.inf
    assert sectors.front_left  == math.inf
    assert sectors.front_right == math.inf
    assert sectors.rear        == math.inf


# ---------------------------------------------------------------------------
# Multiple sectors populated simultaneously
# ---------------------------------------------------------------------------

def test_obstacle_ahead_and_left():
    """Readings in front (10°) and front_left (60°): both populated."""
    sectors = sectorize(*_scan([10, 60], [1.0, 2.0]))
    assert sectors.front      == pytest.approx(1.0, abs=1e-6)
    assert sectors.front_left == pytest.approx(2.0, abs=1e-6)
    assert sectors.front_right == pytest.approx(math.inf, abs=1e-6)
    assert sectors.rear        == pytest.approx(math.inf, abs=1e-6)


def test_multiple_readings_each_sector():
    """Two readings per sector — each sector returns its own minimum."""
    angles = [10, 20,   45, 60,   300, 310,   160, 190]
    ranges = [3.0, 1.0, 2.0, 4.0,  5.0, 2.5,   1.5, 3.0]
    sectors = sectorize(*_scan(angles, ranges))
    assert sectors.front       == pytest.approx(1.0, abs=1e-6)
    assert sectors.front_left  == pytest.approx(2.0, abs=1e-6)
    assert sectors.front_right == pytest.approx(2.5, abs=1e-6)
    assert sectors.rear        == pytest.approx(1.5, abs=1e-6)
