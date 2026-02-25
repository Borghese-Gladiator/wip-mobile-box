"""
Simulation 01 — Pure follow controller.

What's active:    phone path, proportional follow controller
What's disabled:  PID motor lag, wheel slip, sensors/EKF, BLE, obstacles

The robot teleports to the commanded velocity each step (no inertia).
Pose is ground-truth only — there is no estimation error.
"""

import numpy as np
from matplotlib.animation import FuncAnimation

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

        # Follow command from true pose (no EKF, no BLE delay)
        pose = self.robot.true_state
        error = self.phone.true_position() - pose[:2]
        dist = np.linalg.norm(error)

        if dist < Phone.FOLLOW_DISTANCE:
            v_cmd, w_cmd = 0.0, 0.0
        else:
            desired_angle = np.arctan2(error[1], error[0])
            angle_error = wrap_angle(desired_angle - pose[2])
            v_cmd = Phone.Kp_DIST * dist
            w_cmd = Phone.Kp_ANGLE * angle_error

        # Apply command directly — no PID lag
        self.robot.left_vel  = v_cmd - (w_cmd * Robot.WHEEL_BASE / 2)
        self.robot.right_vel = v_cmd + (w_cmd * Robot.WHEEL_BASE / 2)

        # Physics — no wheel slip (slip factor = 1.0)
        v = (self.robot.left_vel + self.robot.right_vel) / 2
        w = (self.robot.right_vel - self.robot.left_vel) / Robot.WHEEL_BASE
        self.robot.true_state[0] += v * np.cos(self.robot.true_state[2]) * self.dt
        self.robot.true_state[1] += v * np.sin(self.robot.true_state[2]) * self.dt
        self.robot.true_state[2] = wrap_angle(self.robot.true_state[2] + w * self.dt)

        self.trail.append(self.robot.true_state.copy())
        if len(self.trail) > 300:
            self.trail.pop(0)


fig, ax = make_fig("01 — Pure Follow Controller")
world = World()
a = make_robot_artists(ax)


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

    return a["robot_body"], a["heading_line"], a["phone_dot"]


run(animate, DT)
