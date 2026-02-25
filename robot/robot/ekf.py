import numpy as np
from .utils import wrap_angle


class EKF:
    def __init__(self, initial_state=None):
        self.x = np.zeros(3) if initial_state is None else np.array(initial_state, dtype=float)
        self.P = np.eye(3) * 0.5

        self.Q = np.eye(3) * 0.01   # process noise
        self.R_gps = np.eye(2) * 0.5
        self.R_imu = np.array([[0.05]])

    def predict(self, v, w, dt):
        theta = self.x[2]
        self.x[0] += v * np.cos(theta) * dt
        self.x[1] += v * np.sin(theta) * dt
        self.x[2] = wrap_angle(self.x[2] + w * dt)

        F = np.array([
            [1, 0, -v * np.sin(theta) * dt],
            [0, 1,  v * np.cos(theta) * dt],
            [0, 0,  1]
        ])
        self.P = F @ self.P @ F.T + self.Q

    def update_gps(self, z):
        H = np.array([[1, 0, 0],
                      [0, 1, 0]])
        y = z - self.x[:2]
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

    def update_imu(self, z):
        H = np.array([[0, 0, 1]])
        y = wrap_angle(z - self.x[2])
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K.flatten() * y
        self.P = (np.eye(3) - K @ H) @ self.P
