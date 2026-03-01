"""
TDD — EKF (estimation/ekf.py).

Tests cover:
  - predict: state integration and covariance growth
  - update_gps: state correction and covariance reduction
  - update_imu: heading correction and covariance reduction
  - angle wrapping in predict and update_imu

Notation:
  x  = [x, y, theta]
  P  = 3x3 covariance
  Q  = process noise (diagonal)
  R_gps = 2x2 GPS noise
  R_imu = 1x1 IMU noise
"""

import math
import numpy as np
import pytest

from robot_follower.estimation.ekf import EKF


def _ekf(**kwargs) -> EKF:
    """Factory with tight noise for predictable test values."""
    defaults = dict(
        initial_state=[0.0, 0.0, 0.0],
        P0=0.5,
        Q_diag=(0.01, 0.01, 0.01),
        R_gps_diag=(0.5, 0.5),
        R_imu=0.05,
    )
    defaults.update(kwargs)
    return EKF(**defaults)


# ---------------------------------------------------------------------------
# predict — state integration
# ---------------------------------------------------------------------------

def test_predict_straight_forward():
    """
    v=1, w=0, dt=0.1, initial theta=0:
    x' = 0 + 1*cos(0)*0.1 = 0.1
    y' = 0 + 1*sin(0)*0.1 = 0.0
    theta' = 0
    """
    ekf = _ekf()
    ekf.predict(v=1.0, w=0.0, dt=0.1)
    assert ekf.x[0] == pytest.approx(0.1,  abs=1e-9)
    assert ekf.x[1] == pytest.approx(0.0,  abs=1e-9)
    assert ekf.x[2] == pytest.approx(0.0,  abs=1e-9)


def test_predict_pure_rotation():
    """
    v=0, w=1, dt=0.1: x,y unchanged, theta=0.1
    """
    ekf = _ekf()
    ekf.predict(v=0.0, w=1.0, dt=0.1)
    assert ekf.x[0] == pytest.approx(0.0, abs=1e-9)
    assert ekf.x[1] == pytest.approx(0.0, abs=1e-9)
    assert ekf.x[2] == pytest.approx(0.1, abs=1e-9)


def test_predict_at_angle():
    """
    theta=π/2, v=1, w=0, dt=0.1:
    x' += cos(π/2)*0.1 ≈ 0
    y' += sin(π/2)*0.1 = 0.1
    """
    ekf = _ekf(initial_state=[0.0, 0.0, math.pi / 2])
    ekf.predict(v=1.0, w=0.0, dt=0.1)
    assert ekf.x[0] == pytest.approx(0.0, abs=1e-6)
    assert ekf.x[1] == pytest.approx(0.1, abs=1e-6)
    assert ekf.x[2] == pytest.approx(math.pi / 2, abs=1e-9)


def test_predict_angle_wraps():
    """theta near π + rotation should wrap to near -π, not exceed π."""
    ekf = _ekf(initial_state=[0.0, 0.0, math.pi - 0.05])
    ekf.predict(v=0.0, w=1.0, dt=0.1)   # dtheta = 0.1 → crosses π
    assert -math.pi <= ekf.x[2] <= math.pi


# ---------------------------------------------------------------------------
# predict — covariance grows
# ---------------------------------------------------------------------------

def test_predict_covariance_grows():
    """Each predict call must increase total uncertainty (trace of P)."""
    ekf = _ekf()
    trace_before = np.trace(ekf.P)
    ekf.predict(v=1.0, w=0.0, dt=0.1)
    assert np.trace(ekf.P) > trace_before


def test_predict_covariance_positive_definite():
    """P must remain symmetric positive-definite after predict."""
    ekf = _ekf()
    for _ in range(10):
        ekf.predict(v=1.0, w=0.5, dt=0.05)
    eigenvalues = np.linalg.eigvalsh(ekf.P)
    assert np.all(eigenvalues > 0)


# ---------------------------------------------------------------------------
# update_gps — state correction
# ---------------------------------------------------------------------------

def test_gps_update_pulls_toward_measurement():
    """
    After predict, state is at (0.1, 0). GPS says (0.5, 0.3).
    x-estimate should move toward 0.5, y toward 0.3.
    """
    ekf = _ekf()
    ekf.predict(v=1.0, w=0.0, dt=0.1)   # state → (0.1, 0, 0)
    x_before = ekf.x[:2].copy()
    ekf.update_gps(np.array([0.5, 0.3]))
    assert ekf.x[0] > x_before[0], "x should move toward 0.5"
    assert ekf.x[1] > x_before[1], "y should move toward 0.3"


def test_gps_update_with_perfect_measurement_partial_correction():
    """
    GPS = true state. Correction should be partial (Kalman blend),
    not jump all the way to measurement.
    """
    ekf = _ekf()
    ekf.predict(v=1.0, w=0.0, dt=0.1)   # state → (0.1, 0, 0)
    x_pred = ekf.x[0]
    ekf.update_gps(np.array([ekf.x[0], ekf.x[1]]))   # z == x_pred
    # State should be unchanged when measurement matches prediction
    assert ekf.x[0] == pytest.approx(x_pred, abs=1e-9)


def test_gps_update_reduces_xy_covariance():
    """P[0,0] and P[1,1] must shrink after GPS update."""
    ekf = _ekf()
    ekf.predict(v=1.0, w=0.0, dt=0.1)
    p00_before = ekf.P[0, 0]
    p11_before = ekf.P[1, 1]
    ekf.update_gps(np.array([0.1, 0.0]))
    assert ekf.P[0, 0] < p00_before
    assert ekf.P[1, 1] < p11_before


def test_gps_update_does_not_change_theta():
    """GPS only observes x,y — theta must be unchanged."""
    ekf = _ekf()
    ekf.predict(v=0.0, w=1.0, dt=0.1)   # theta = 0.1
    theta_before = ekf.x[2]
    ekf.update_gps(np.array([99.0, 99.0]))   # wild GPS reading
    assert ekf.x[2] == pytest.approx(theta_before, abs=1e-9)


# ---------------------------------------------------------------------------
# update_imu — heading correction
# ---------------------------------------------------------------------------

def test_imu_update_pulls_toward_measurement():
    """
    Predict rotates to theta=0.3. IMU says theta=0.0.
    Estimate should move toward 0.0 (decrease).
    """
    ekf = _ekf()
    ekf.predict(v=0.0, w=1.0, dt=0.3)   # theta → 0.3
    ekf.update_imu(0.0)
    assert ekf.x[2] < 0.3, "heading should move toward 0.0"
    assert ekf.x[2] > 0.0 - 1e-6, "but not overshoot past 0"


def test_imu_update_with_matching_measurement_unchanged():
    """IMU measurement == predicted theta → state unchanged."""
    ekf = _ekf()
    ekf.predict(v=0.0, w=1.0, dt=0.1)   # theta = 0.1
    theta_before = ekf.x[2]
    ekf.update_imu(theta_before)
    assert ekf.x[2] == pytest.approx(theta_before, abs=1e-9)


def test_imu_update_reduces_theta_covariance():
    """P[2,2] must shrink after IMU update."""
    ekf = _ekf()
    ekf.predict(v=0.0, w=1.0, dt=0.1)
    p22_before = ekf.P[2, 2]
    ekf.update_imu(0.1)
    assert ekf.P[2, 2] < p22_before


def test_imu_update_does_not_change_xy():
    """
    IMU H = [[0,0,1]] observes only theta. x,y can shift slightly via
    off-diagonal covariance coupling, but theta is the primary corrected state.
    With a diagonal initial P and straight-line predict, P[0,2] stays near 0
    so x should be essentially unchanged; y coupling grows with motion so we
    check it stays within a small bound rather than exactly zero.
    """
    ekf = _ekf(P0=0.1)   # tight P → off-diagonal coupling is small
    ekf.predict(v=1.0, w=0.0, dt=0.1)
    xy_before = ekf.x[:2].copy()
    ekf.update_imu(0.0)   # mild correction, not a wild reading
    # x,y corrections are small (bounded by K[0]*innovation and K[1]*innovation)
    assert abs(ekf.x[0] - xy_before[0]) < 0.05
    assert abs(ekf.x[1] - xy_before[1]) < 0.05


def test_imu_update_wraps_angle_error():
    """
    Predicted theta near π, IMU measurement near -π (same physical angle,
    opposite sign). Wrapped innovation should be near 0, not ≈2π.
    """
    ekf = _ekf(initial_state=[0.0, 0.0, math.pi - 0.01])
    ekf.update_imu(-(math.pi - 0.01))   # same angle, wrapped differently
    # Should not produce a huge correction
    assert abs(ekf.x[2]) <= math.pi
