"""
Combined robot simulation: OOP, obstacles, noise, BLE, EKF, PID motors.

Visualization layer — logic lives in the `robot` package.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from robot import World


# ======================================================
# Visualization
# ======================================================

plt.style.use("dark_background")
fig, ax = plt.subplots(figsize=(9, 9))
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_aspect("equal")
ax.set_title("Mobile Box — Combined Simulation", fontsize=14)

world = World()

# Robot: true (orange circle) and EKF estimate (cyan dot)
robot_body = plt.Circle((-5, 0), 0.4, color="orange", zorder=5)
ax.add_patch(robot_body)

heading_line, = ax.plot([], [], color="orange", linewidth=2, zorder=6)
robot_est_dot, = ax.plot([], [], "o", color="cyan", markersize=6, zorder=7,
                          label="EKF estimate")
phone_dot, = ax.plot([], [], "o", color="yellow", markersize=10, zorder=7,
                      label="Phone")

# Obstacles
for obs in world.obstacles.obstacles:
    ax.add_patch(plt.Circle(obs, world.obstacles.radius, color="red", alpha=0.25))
    ax.add_patch(plt.Circle(obs, 0.15, color="red", alpha=0.8))

ax.legend(loc="upper right", fontsize=9)


def animate(frame):
    world.step()

    true = world.robot.true_state
    est  = world.robot.estimated_pose
    phone_pos = world.phone.true_position()

    # Robot body + heading
    robot_body.center = (true[0], true[1])
    heading_line.set_data(
        [true[0], true[0] + np.cos(true[2])],
        [true[1], true[1] + np.sin(true[2])]
    )

    robot_est_dot.set_data([est[0]], [est[1]])
    phone_dot.set_data([phone_pos[0]], [phone_pos[1]])

    # Fading trails
    for coll in ax.collections[:]:
        coll.remove()

    n_true = len(world.trail_true)
    for i, p in enumerate(world.trail_true):
        ax.scatter(p[0], p[1], s=4, color="orange", alpha=i / max(n_true, 1))

    n_est = len(world.trail_est)
    for i, p in enumerate(world.trail_est):
        ax.scatter(p[0], p[1], s=4, color="cyan", alpha=i / max(n_est, 1))

    # Re-draw obstacle zones (cleared above)
    for obs in world.obstacles.obstacles:
        ax.add_patch(plt.Circle(obs, world.obstacles.radius,
                                color="red", alpha=0.25))

    return robot_body, heading_line, robot_est_dot, phone_dot


ani = FuncAnimation(fig, animate, interval=int(world.dt * 1000), blit=False)
plt.tight_layout()
plt.show()
