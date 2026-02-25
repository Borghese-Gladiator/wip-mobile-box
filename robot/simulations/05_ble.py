"""
Simulation 05 — BLE latency + packet drops.

Adds over 04:   BLE buffer delay (400 ms) and 10% packet drop rate
Still disabled: obstacles

Commands from the phone are delayed and occasionally lost.
The robot holds its last good command when packets drop.
Watch the robot overshoot corners as stale commands arrive late.
"""

from robot.robot import Robot
from robot.phone import Phone
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

        # BLE transmit: latency buffer + packet drops applied inside Phone.transmit()
        v_cmd, w_cmd = self.phone.transmit(self.robot.estimated_pose)

        self.robot.set_velocity_command(v_cmd, w_cmd)
        self.robot.motor_update(self.dt)
        v_true, w_true = self.robot.physics_update(self.dt)
        self.robot.ekf_step(v_true, w_true, self.dt)

        self.trail_true.append(self.robot.true_state.copy())
        self.trail_est.append(self.robot.estimated_pose.copy())
        if len(self.trail_true) > 300:
            self.trail_true.pop(0)
            self.trail_est.pop(0)


fig, ax = make_fig("05 — BLE Latency + Packet Drops")
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
