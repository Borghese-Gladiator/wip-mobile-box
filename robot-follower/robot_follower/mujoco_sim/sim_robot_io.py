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
    """Concrete RobotIO backed by MuJoCo, with noise injection.

    Sensor reads that depend on pose (target, obstacles) use the estimated
    pose passed in, not ground truth. The controller never sees the true
    MuJoCo state.
    """

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

    # --- Ground truth (sim-level only, not exposed to controller) ---

    def get_true_pose(self) -> Pose2D:
        """Ground-truth pose — for sim-level goal checks only, not the controller."""
        return self._robot.get_pose()

    # --- RobotIO interface ---

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

    def read_target(self, estimated_pose: Pose2D) -> TargetMeasurement:
        """BLE target sensing uses the estimated pose, not ground truth."""
        return self._ble_tracker.read(
            estimated_pose.x, estimated_pose.y, estimated_pose.yaw,
            self._data.time,
        )

    def read_obstacles(self, estimated_pose: Pose2D) -> ObstacleMeasurement:
        """Raycast from the estimated pose, not ground truth.

        Note: MuJoCo raycasts from a world-space origin, so small pose
        estimation errors will shift where rays originate — matching how
        a real robot's sensor frame drifts with odometry error.
        """
        return self._range_sensors.read(
            estimated_pose.x, estimated_pose.y, estimated_pose.yaw,
        )

    def send_wheel_command(self, cmd: WheelCommand) -> None:
        left, right = self._motor_noise.apply(cmd.left, cmd.right, self._sim_dt)
        self._robot.set_ctrl(left, right)

    def get_timestamp(self) -> float:
        return float(self._data.time)
