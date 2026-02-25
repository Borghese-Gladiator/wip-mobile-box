import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ==========================
# Utility
# ==========================

def wrap_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi


# ==========================
# PID Controller
# ==========================

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return (self.kp * error +
                self.ki * self.integral +
                self.kd * derivative)


# ==========================
# Kalman Filter (EKF)
# ==========================

class EKF:
    def __init__(self):
        self.x = np.zeros(3)  # x, y, theta
        self.P = np.eye(3) * 0.5

        self.Q = np.eye(3) * 0.01   # process noise
        self.R_gps = np.eye(2) * 0.5
        self.R_imu = np.array([[0.05]])

    def predict(self, v, w, dt):
        theta = self.x[2]

        # Motion model
        self.x[0] += v * np.cos(theta) * dt
        self.x[1] += v * np.sin(theta) * dt
        self.x[2] = wrap_angle(self.x[2] + w * dt)

        # Jacobian
        F = np.array([
            [1, 0, -v*np.sin(theta)*dt],
            [0, 1,  v*np.cos(theta)*dt],
            [0, 0, 1]
        ])

        self.P = F @ self.P @ F.T + self.Q

    def update_gps(self, z):
        H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])

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

        self.x += (K.flatten() * y)
        self.P = (np.eye(3) - K @ H) @ self.P


# ==========================
# Robot
# ==========================

class Robot:
    def __init__(self):
        self.true_state = np.array([-5.0, 0.0, 0.0])
        self.wheel_base = 0.5
        self.wheel_radius = 0.1
        self.encoder_ticks_per_rev = 360

        self.left_pid = PID(8, 2, 0.1)
        self.right_pid = PID(8, 2, 0.1)

        self.left_vel = 0
        self.right_vel = 0

        self.left_cmd = 0
        self.right_cmd = 0

        self.ekf = EKF()

        self.gps_bias = np.zeros(2)

    def set_velocity_command(self, v, w):
        self.left_cmd = v - (w * self.wheel_base / 2)
        self.right_cmd = v + (w * self.wheel_base / 2)

    def motor_update(self, dt):
        # PID velocity control
        l_error = self.left_cmd - self.left_vel
        r_error = self.right_cmd - self.right_vel

        l_force = self.left_pid.update(l_error, dt)
        r_force = self.right_pid.update(r_error, dt)

        # Motor inertia simulation
        self.left_vel += l_force * dt
        self.right_vel += r_force * dt

    def physics_update(self, dt):
        v = (self.left_vel + self.right_vel) / 2
        w = (self.right_vel - self.left_vel) / self.wheel_base

        self.true_state[0] += v * np.cos(self.true_state[2]) * dt
        self.true_state[1] += v * np.sin(self.true_state[2]) * dt
        self.true_state[2] = wrap_angle(self.true_state[2] + w * dt)

        return v, w

    def read_encoders(self, dt):
        # convert velocity to ticks
        l_ticks = (self.left_vel * dt) / (2*np.pi*self.wheel_radius) \
                  * self.encoder_ticks_per_rev
        r_ticks = (self.right_vel * dt) / (2*np.pi*self.wheel_radius) \
                  * self.encoder_ticks_per_rev

        # quantize
        return int(l_ticks), int(r_ticks)

    def read_imu(self):
        noise = np.random.normal(0, 0.02)
        return wrap_angle(self.true_state[2] + noise)

    def read_gps(self):
        # slow drifting bias
        self.gps_bias += np.random.normal(0, 0.001, 2)
        noise = np.random.normal(0, 0.2, 2)
        return self.true_state[:2] + self.gps_bias + noise

    def ekf_step(self, v, w, dt):
        self.ekf.predict(v, w, dt)

        gps = self.read_gps()
        imu = self.read_imu()

        self.ekf.update_gps(gps)
        self.ekf.update_imu(imu)


# ==========================
# Phone
# ==========================

class Phone:
    def __init__(self):
        self.time = 0

    def update(self, dt):
        self.time += dt

    def true_position(self):
        r = 4
        angle = 0.3 * self.time
        return np.array([r*np.cos(angle), r*np.sin(angle)])

    def compute_command(self, robot_estimate):
        error = self.true_position() - robot_estimate[:2]
        dist = np.linalg.norm(error)

        desired_angle = np.arctan2(error[1], error[0])
        angle_error = wrap_angle(desired_angle - robot_estimate[2])

        v = 1.0 * dist
        w = 3.0 * angle_error

        return v, w


# ==========================
# World
# ==========================

class World:
    def __init__(self):
        self.robot = Robot()
        self.phone = Phone()
        self.dt = 0.05
        self.trail_true = []
        self.trail_est = []

    def step(self):
        self.phone.update(self.dt)

        # Phone uses EKF estimate
        v_cmd, w_cmd = self.phone.compute_command(self.robot.ekf.x)

        self.robot.set_velocity_command(v_cmd, w_cmd)
        self.robot.motor_update(self.dt)

        v_true, w_true = self.robot.physics_update(self.dt)

        # EKF update using true motion input
        self.robot.ekf_step(v_true, w_true, self.dt)

        self.trail_true.append(self.robot.true_state.copy())
        self.trail_est.append(self.robot.ekf.x.copy())

        if len(self.trail_true) > 200:
            self.trail_true.pop(0)
            self.trail_est.pop(0)


# ==========================
# Visualization
# ==========================

plt.style.use("dark_background")
fig, ax = plt.subplots(figsize=(8,8))
ax.set_xlim(-10,10)
ax.set_ylim(-10,10)
ax.set_aspect("equal")

world = World()

robot_true_dot, = ax.plot([], [], 'o', color='orange')
robot_est_dot, = ax.plot([], [], 'o', color='cyan')
phone_dot, = ax.plot([], [], 'o', color='yellow')

def animate(frame):
    world.step()

    true = world.robot.true_state
    est = world.robot.ekf.x
    phone = world.phone.true_position()

    robot_true_dot.set_data(true[0], true[1])
    robot_est_dot.set_data(est[0], est[1])
    phone_dot.set_data(phone[0], phone[1])

    ax.collections.clear()
    for p in world.trail_true:
        ax.scatter(p[0], p[1], s=3, color='orange', alpha=0.2)
    for p in world.trail_est:
        ax.scatter(p[0], p[1], s=3, color='cyan', alpha=0.2)

    return robot_true_dot, robot_est_dot, phone_dot

ani = FuncAnimation(fig, animate, interval=50)
plt.show()