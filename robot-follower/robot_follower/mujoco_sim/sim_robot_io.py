"""MuJoCo implementation of RobotIO — wires sim sensors to the abstract interface."""

from __future__ import annotations

import mujoco

from robot_follower.core.types import (
    Pose2D, Twist2D, WheelCommand, WheelState, IMUReading,
    TargetMeasurement, ObstacleMeasurement,
)
from robot_follower.interfaces.robot_io import RobotIO
from .mujoco_robot import MuJoCoRobot
from .mujoco_sensors import MuJoCoRangeSensors, MuJoCoBLETracker
from .mujoco_noise import MotorNoise, IMUNoise, EncoderNoise


class SimRobotIO(RobotIO):
    """Concrete RobotIO backed by MuJoCo, with noise injection."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: dict,
    ):
        self._model = model
        self._data = data
        self._robot = MuJoCoRobot(model, data, config["robot"]["max_wheel_speed"])
        self._range_sensors = MuJoCoRangeSensors(model, data, config)
        self._ble_tracker = MuJoCoBLETracker(model, data, config)
        self._motor_noise = MotorNoise()
        self._imu_noise = IMUNoise()
        self._encoder_noise = EncoderNoise()
        self._sim_dt = model.opt.timestep

    def read_pose(self) -> Pose2D:
        return self._robot.get_pose()

    def read_velocity(self) -> Twist2D:
        return self._robot.get_velocity()

    def read_wheels(self) -> WheelState:
        left_spd, right_spd = self._robot.get_wheel_speeds()
        left_ticks, right_ticks = self._encoder_noise.update(left_spd, right_spd, self._sim_dt)
        return WheelState(
            left_speed=left_spd,
            right_speed=right_spd,
            left_ticks=left_ticks,
            right_ticks=right_ticks,
        )

    def read_imu(self) -> IMUReading:
        vel = self._robot.get_velocity()
        noisy_gz = self._imu_noise.apply(vel.angular, self._sim_dt)
        return IMUReading(gyro_z=noisy_gz, timestamp=self._data.time)

    def read_target(self) -> TargetMeasurement:
        pose = self._robot.get_pose()
        return self._ble_tracker.read(pose.x, pose.y, pose.yaw, self._data.time)

    def read_obstacles(self) -> ObstacleMeasurement:
        pose = self._robot.get_pose()
        return self._range_sensors.read(pose.x, pose.y, pose.yaw)

    def send_wheel_command(self, cmd: WheelCommand) -> None:
        left, right = self._motor_noise.apply(cmd.left, cmd.right, self._sim_dt)
        self._robot.set_ctrl(left, right)

    def get_timestamp(self) -> float:
        return float(self._data.time)
