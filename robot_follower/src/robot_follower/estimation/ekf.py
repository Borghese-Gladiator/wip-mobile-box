"""Extended Kalman Filter for 2-D differential-drive pose estimation.

State vector: [x, y, theta]

Prediction (motion model):
    x'     = x + v * cos(theta) * dt
    y'     = y + v * sin(theta) * dt
    theta' = wrap(theta + w * dt)

    Jacobian F:
        [[1, 0, -v*sin(theta)*dt],
         [0, 1,  v*cos(theta)*dt],
         [0, 0,  1              ]]

    P' = F @ P @ F.T + Q

GPS position update (z = [x_meas, y_meas]):
    H  = [[1, 0, 0], [0, 1, 0]]
    y  = z - H @ x'
    S  = H @ P' @ H.T + R_gps
    K  = P' @ H.T @ inv(S)
    x'' = x' + K @ y
    P'' = (I - K @ H) @ P'

IMU heading update (z = theta_meas):
    H  = [[0, 0, 1]]
    y  = wrap(z - x''[2])
    S  = H @ P'' @ H.T + R_imu
    K  = P'' @ H.T @ inv(S)
    x'''= x'' + K.flatten() * y
    P'''= (I - K @ H) @ P''
"""

import math
import numpy as np


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class EKF:
    """
    2-D pose EKF for a differential-drive robot.

    Parameters
    ----------
    initial_state : array-like [x, y, theta], default zeros
    P0            : initial covariance scalar (applied to identity)
    Q_diag        : process noise diagonal [qx, qy, qth]
    R_gps_diag    : GPS measurement noise diagonal [rx, ry]
    R_imu         : IMU heading noise variance (scalar)
    """

    def __init__(
        self,
        initial_state=None,
        P0: float = 0.5,
        Q_diag: tuple[float, float, float] = (0.01, 0.01, 0.01),
        R_gps_diag: tuple[float, float] = (0.5, 0.5),
        R_imu: float = 0.05,
    ):
        self.x = np.zeros(3) if initial_state is None else np.array(initial_state, dtype=float)
        self.P = np.eye(3) * P0
        self.Q = np.diag(Q_diag)
        self.R_gps = np.diag(R_gps_diag)
        self.R_imu = np.array([[R_imu]])

    # ------------------------------------------------------------------
    # Prediction step
    # ------------------------------------------------------------------

    def predict(self, v: float, w: float, dt: float) -> None:
        """Propagate state and covariance with motion model."""
        th = self.x[2]
        self.x[0] += v * math.cos(th) * dt
        self.x[1] += v * math.sin(th) * dt
        self.x[2] = _wrap(self.x[2] + w * dt)

        F = np.array([
            [1, 0, -v * math.sin(th) * dt],
            [0, 1,  v * math.cos(th) * dt],
            [0, 0,  1],
        ])
        self.P = F @ self.P @ F.T + self.Q

    # ------------------------------------------------------------------
    # GPS update (position measurement)
    # ------------------------------------------------------------------

    def update_gps(self, z: np.ndarray) -> None:
        """Update with a 2-D position measurement [x_meas, y_meas]."""
        H = np.array([[1, 0, 0],
                      [0, 1, 0]])
        y = z - self.x[:2]
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

    # ------------------------------------------------------------------
    # IMU update (heading measurement)
    # ------------------------------------------------------------------

    def update_imu(self, z: float) -> None:
        """Update with a heading measurement (radians)."""
        H = np.array([[0, 0, 1]])
        y = _wrap(z - self.x[2])
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K.flatten() * y
        self.x[2] = _wrap(self.x[2])
        self.P = (np.eye(3) - K @ H) @ self.P
