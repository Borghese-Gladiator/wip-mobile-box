"""
Simulation 03 — Wheel slip.

Adds over 02:   random per-wheel slip factor each physics step
Still disabled: sensors/EKF (pose is still ground-truth), BLE, obstacles

The robot drifts from its commanded path due to imperfect traction.
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
        self.trail = []

    def step(self):
        self.phone.update()

        # Follow command from true pose (no EKF, no BLE)
        pose = self.robot.true_state
        error = self.phone.true_position() - pose[:2]
        dist = np.linalg.norm(error)

        if dist < Phone.FOLLOW_DISTANCE:
            v_cmd, w_cmd = 0.0, 0.0
        else:
            desired_angle = np.arctan2(error[1], error[0])
            angle_error = wrap_angle(desired_angle - pose[2])
            v_cmd = min(Phone.Kp_DIST * dist, Phone.MAX_V_CMD)
            w_cmd = Phone.Kp_ANGLE * angle_error

        self.robot.set_velocity_command(v_cmd, w_cmd)
        self.robot.motor_update(self.dt)

        # Physics with wheel slip (uses Robot.WHEEL_SLIP_STD)
        self.robot.physics_update(self.dt)

        self.trail.append(self.robot.true_state.copy())
        if len(self.trail) > 300:
            self.trail.pop(0)


fig, ax = make_fig("03 — Wheel Slip")
world = World()
a = make_robot_artists(ax)

step_label = ax.text(0.02, 0.02, "step: 0", transform=ax.transAxes,
                     color="white", fontsize=9, va="bottom", ha="left")


def animate(frame):
    world.step()
    true = world.robot.true_state
    phone_pos = world.phone.true_position()

    a["robot_body"].center = (true[0], true[1])
    a["heading_line"].set_data(
        [true[0], true[0] + np.cos(true[2])],
        [true[1], true[1] + np.sin(true[2])],
    )
    a["phone_dot"].set_data([phone_pos[0]], [phone_pos[1]])

    clear_collections(ax)
    draw_fading_trail(ax, world.trail, "orange")

    step_label.set_text(f"step: {frame}")
    return a["robot_body"], a["heading_line"], a["phone_dot"]


run(animate, DT)
