import numpy as np
from .pid import PID
from .ekf import EKF
from .utils import wrap_angle


class Robot:
    """
    True physics + PID wheel velocity control + EKF state estimation.

    Sensors:
      - GPS (noisy + slow drift bias)
      - IMU (noisy heading)
      - Wheel encoders (quantized ticks)
    """

    WHEEL_BASE = 0.5
    WHEEL_RADIUS = 0.1
    ENCODER_TICKS_PER_REV = 360
    MAX_SPEED = 2.0
    IMU_NOISE_STD = 0.02
    GPS_NOISE_STD = 0.2
    WHEEL_SLIP_STD = 0.05

    def __init__(self, x=-5.0, y=0.0, theta=0.0):
        self.true_state = np.array([x, y, theta], dtype=float)

        self.left_vel = 0.0
        self.right_vel = 0.0
        self.left_cmd = 0.0
        self.right_cmd = 0.0

        self.left_pid = PID(kp=8, ki=2, kd=0.1)
        self.right_pid = PID(kp=8, ki=2, kd=0.1)

        self.ekf = EKF(initial_state=[x, y, theta])
        self.gps_bias = np.zeros(2)

    # ---- Command interface ----

    def set_velocity_command(self, v, w):
        """Convert (v, w) to per-wheel velocity commands."""
        self.left_cmd  = v - (w * self.WHEEL_BASE / 2)
        self.right_cmd = v + (w * self.WHEEL_BASE / 2)

        # Clamp to max speed
        self.left_cmd  = np.clip(self.left_cmd,  -self.MAX_SPEED, self.MAX_SPEED)
        self.right_cmd = np.clip(self.right_cmd, -self.MAX_SPEED, self.MAX_SPEED)

    # ---- Motor PID update ----

    def motor_update(self, dt):
        l_force = self.left_pid.update(self.left_cmd  - self.left_vel,  dt)
        r_force = self.right_pid.update(self.right_cmd - self.right_vel, dt)
        self.left_vel  += l_force * dt
        self.right_vel += r_force * dt

    # ---- Physics (with wheel slip) ----

    def physics_update(self, dt):
        slip_l = np.random.normal(1.0, self.WHEEL_SLIP_STD)
        slip_r = np.random.normal(1.0, self.WHEEL_SLIP_STD)

        v_l = self.left_vel  * slip_l
        v_r = self.right_vel * slip_r

        v = (v_l + v_r) / 2
        w = (v_r - v_l) / self.WHEEL_BASE

        self.true_state[0] += v * np.cos(self.true_state[2]) * dt
        self.true_state[1] += v * np.sin(self.true_state[2]) * dt
        self.true_state[2] = wrap_angle(self.true_state[2] + w * dt)

        return v, w

    # ---- Sensors ----

    def read_imu(self):
        return wrap_angle(self.true_state[2] + np.random.normal(0, self.IMU_NOISE_STD))

    def read_gps(self):
        self.gps_bias += np.random.normal(0, 0.001, 2)  # slow drift
        noise = np.random.normal(0, self.GPS_NOISE_STD, 2)
        return self.true_state[:2] + self.gps_bias + noise

    def read_encoders(self, dt):
        ticks_per_meter = self.ENCODER_TICKS_PER_REV / (2 * np.pi * self.WHEEL_RADIUS)
        l_ticks = int(self.left_vel  * dt * ticks_per_meter)
        r_ticks = int(self.right_vel * dt * ticks_per_meter)
        return l_ticks, r_ticks

    # ---- EKF step ----

    def ekf_step(self, v, w, dt):
        self.ekf.predict(v, w, dt)
        self.ekf.update_gps(self.read_gps())
        self.ekf.update_imu(self.read_imu())

    @property
    def estimated_pose(self):
        return self.ekf.x
