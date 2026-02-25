from .robot import Robot
from .phone import Phone
from .obstacles import ObstacleField


class World:
    def __init__(self):
        self.dt = 0.05
        self.time = 0.0

        self.robot = Robot(x=-5.0, y=0.0, theta=0.0)
        self.phone = Phone(dt=self.dt)
        self.obstacles = ObstacleField()

        self.trail_true = []
        self.trail_est  = []
        self.max_trail  = 300

    def step(self):
        self.time += self.dt
        self.phone.update()

        # Phone sends BLE command based on EKF estimate
        v_cmd, w_cmd = self.phone.transmit(self.robot.estimated_pose)

        # Obstacle avoidance correction
        dv, dw = self.obstacles.avoidance_command(self.robot)
        v_cmd += dv
        w_cmd += dw

        # Apply to robot motors
        self.robot.set_velocity_command(v_cmd, w_cmd)
        self.robot.motor_update(self.dt)

        # Physics
        v_true, w_true = self.robot.physics_update(self.dt)

        # EKF update
        self.robot.ekf_step(v_true, w_true, self.dt)

        # Record trails
        self.trail_true.append(self.robot.true_state.copy())
        self.trail_est.append(self.robot.estimated_pose.copy())

        if len(self.trail_true) > self.max_trail:
            self.trail_true.pop(0)
            self.trail_est.pop(0)
