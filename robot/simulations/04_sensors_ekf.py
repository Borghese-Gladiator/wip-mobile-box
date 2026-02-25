"""
Simulation 04 — Noisy sensors + EKF.

Adds over 03:   GPS noise + drift bias, IMU noise, EKF fusing both
Still disabled: BLE latency/drops, obstacles

Orange = true robot path.  Cyan = EKF estimated pose.
Watch the cyan dot lag and jitter relative to the true position.
The phone commands are based on the EKF estimate, not ground-truth.
"""

import numpy as np

from robot.robot import Robot
from robot.phone import Phone
from robot.utils import wrap_angle
from simulations.utils import make_fig, make_robot_artists, draw_fading_trail, clear_collections, run

DT = 0.05


class World:
    def __init__(self):
        self.dt = DT
        self.robot = Robot(x=-5.0, y=0.0, theta=0.0)
        self.phone = Phone(dt=self.dt)
        self.trail_true = []
        self.trail_est  = []

    def step(self):
        self.phone.update()

        # Follow command from EKF estimate (not ground-truth)
        est = self.robot.estimated_pose
        error = self.phone.true_position() - est[:2]
        dist = np.linalg.norm(error)

        if dist < Phone.FOLLOW_DISTANCE:
            v_cmd, w_cmd = 0.0, 0.0
        else:
            desired_angle = np.arctan2(error[1], error[0])
            angle_error = wrap_angle(desired_angle - est[2])
            v_cmd = Phone.Kp_DIST * dist
            w_cmd = Phone.Kp_ANGLE * angle_error

        self.robot.set_velocity_command(v_cmd, w_cmd)
        self.robot.motor_update(self.dt)
        v_true, w_true = self.robot.physics_update(self.dt)
        self.robot.ekf_step(v_true, w_true, self.dt)

        self.trail_true.append(self.robot.true_state.copy())
        self.trail_est.append(self.robot.estimated_pose.copy())
        if len(self.trail_true) > 300:
            self.trail_true.pop(0)
            self.trail_est.pop(0)


fig, ax = make_fig("04 — Noisy Sensors + EKF")
world = World()
a = make_robot_artists(ax, with_ekf=True)


def animate(frame):
    world.step()
    true = world.robot.true_state
    est  = world.robot.estimated_pose
    phone_pos = world.phone.true_position()

    a["robot_body"].center = (true[0], true[1])
    a["heading_line"].set_data(
        [true[0], true[0] + np.cos(true[2])],
        [true[1], true[1] + np.sin(true[2])],
    )
    a["robot_est_dot"].set_data([est[0]], [est[1]])
    a["phone_dot"].set_data([phone_pos[0]], [phone_pos[1]])

    clear_collections(ax)
    draw_fading_trail(ax, world.trail_true, "orange")
    draw_fading_trail(ax, world.trail_est,  "cyan")

    return a["robot_body"], a["heading_line"], a["robot_est_dot"], a["phone_dot"]


run(animate, DT)
