"""
Combined robot simulation: OOP, obstacles, noise, BLE, EKF, PID motors.

Visualization layer — logic lives in the `robot` package.
"""

import numpy as np
import matplotlib.pyplot as plt

from robot import World
from simulations.utils import make_fig, make_robot_artists, draw_fading_trail, clear_collections, run

fig, ax = make_fig("Mobile Box — Combined Simulation")
world = World()
a = make_robot_artists(ax, with_ekf=True)

for obs in world.obstacles.obstacles:
    ax.add_patch(plt.Circle(obs, world.obstacles.radius, color="red", alpha=0.25))
    ax.add_patch(plt.Circle(obs, 0.15, color="red", alpha=0.8))

step_label = ax.text(0.02, 0.02, "step: 0", transform=ax.transAxes,
                     color="white", fontsize=9, va="bottom", ha="left")


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

    step_label.set_text(f"step: {frame}")
    return a["robot_body"], a["heading_line"], a["robot_est_dot"], a["phone_dot"]


run(animate, world.dt)
