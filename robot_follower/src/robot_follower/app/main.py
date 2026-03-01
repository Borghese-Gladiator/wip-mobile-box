"""
Main control loop — orchestrates all subsystems.

Simulation entry-point:  instantiate World(), call world.step() each frame.
Hardware entry-point:    call run_hardware_loop() from a script.
"""

import math
import numpy as np

from ..types.messages import PersonTarget, ObstacleSectors, Twist
from ..behaviors.follow import FollowBehavior
from ..behaviors.avoid import AvoidBehavior
from ..behaviors.arbiter import Arbiter
from ..control.diff_drive import DiffDriveKinematics
from ..control.limits import AccelLimiter
from ..perception.obstacles import PotentialFieldObstacles
from ..estimation.ekf import EKF
from .logging import get_logger

log = get_logger(__name__)


# ---------------------------------------------------------------------------
# Simulation world (ported from robot/src/robot/sim/pygame/sim.py)
# ---------------------------------------------------------------------------

class _SimRobot:
    """Minimal physics + PID + EKF simulation of the robot."""

    WHEEL_BASE = 0.5
    WHEEL_RADIUS = 0.1
    ENCODER_TICKS_PER_REV = 360
    MAX_SPEED = 2.0
    IMU_NOISE_STD = 0.02
    GPS_NOISE_STD = 0.2
    WHEEL_SLIP_STD = 0.05

    def __init__(self, x: float = -5.0, y: float = 0.0, theta: float = 0.0):
        self.true_state = np.array([x, y, theta], dtype=float)
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.left_cmd = 0.0
        self.right_cmd = 0.0

        # Simple P controllers stand in for full PID to reduce imports
        self._kp = 8.0

        self._ekf = EKF(initial_state=[x, y, theta])
        self._gps_bias = np.zeros(2)

    # -- command --

    def set_velocity_command(self, v: float, w: float) -> None:
        half = self.WHEEL_BASE / 2.0
        self.left_cmd = np.clip(v - w * half, -self.MAX_SPEED, self.MAX_SPEED)
        self.right_cmd = np.clip(v + w * half, -self.MAX_SPEED, self.MAX_SPEED)

    # -- motor update (simple P) --

    def motor_update(self, dt: float) -> None:
        self.left_vel += self._kp * (self.left_cmd - self.left_vel) * dt
        self.right_vel += self._kp * (self.right_cmd - self.right_vel) * dt

    # -- physics --

    def physics_update(self, dt: float) -> tuple[float, float]:
        sl = np.random.normal(1.0, self.WHEEL_SLIP_STD)
        sr = np.random.normal(1.0, self.WHEEL_SLIP_STD)
        vl = self.left_vel * sl
        vr = self.right_vel * sr
        v = (vl + vr) / 2.0
        w = (vr - vl) / self.WHEEL_BASE
        self.true_state[0] += v * math.cos(self.true_state[2]) * dt
        self.true_state[1] += v * math.sin(self.true_state[2]) * dt
        self.true_state[2] = (self.true_state[2] + w * dt + math.pi) % (2 * math.pi) - math.pi
        return v, w

    # -- sensors --

    def _read_imu(self) -> float:
        a = self.true_state[2] + np.random.normal(0, self.IMU_NOISE_STD)
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _read_gps(self) -> np.ndarray:
        self._gps_bias += np.random.normal(0, 0.001, 2)
        return self.true_state[:2] + self._gps_bias + np.random.normal(0, self.GPS_NOISE_STD, 2)

    # -- EKF --

    def ekf_step(self, v: float, w: float, dt: float) -> None:
        self._ekf.predict(v, w, dt)
        self._ekf.update_gps(self._read_gps())
        self._ekf.update_imu(self._read_imu())

    @property
    def estimated_pose(self) -> np.ndarray:
        return self._ekf.x.copy()


class _SimPhone:
    """Circular-path phone target (simulation only)."""

    PATH_RADIUS = 4.0
    PATH_SPEED = 0.3

    def __init__(self, dt: float):
        self.dt = dt
        self.time = 0.0

    def update(self) -> None:
        self.time += self.dt

    def true_position(self) -> np.ndarray:
        angle = self.PATH_SPEED * self.time
        return np.array([
            self.PATH_RADIUS * math.cos(angle),
            self.PATH_RADIUS * math.sin(angle),
        ])


# ---------------------------------------------------------------------------
# World — public simulation facade
# ---------------------------------------------------------------------------

class World:
    """
    Simulation world: orchestrates robot, phone target, and obstacle field.

    Usage
    -----
    world = World()
    for frame in range(1000):
        world.step()
        # read world.robot.true_state, world.robot.estimated_pose, etc.
    """

    def __init__(self, dt: float = 0.05):
        self.dt = dt
        self.time = 0.0

        self.robot = _SimRobot(x=-5.0, y=0.0, theta=0.0)
        self.phone = _SimPhone(dt=dt)
        self.obstacles = PotentialFieldObstacles()

        self._follow = FollowBehavior(dt=dt)
        self._avoid_behavior = AvoidBehavior()
        self._arbiter = Arbiter()
        self._kinematics = DiffDriveKinematics()
        self._limiter = AccelLimiter(dt=dt)

        self.trail_true: list[np.ndarray] = []
        self.trail_est: list[np.ndarray] = []
        self.max_trail = 300

    def step(self) -> None:
        self.time += self.dt
        self.phone.update()

        # Target from phone
        target = PersonTarget(
            position=self.phone.true_position(),
            confidence=1.0,
            timestamp=self.time,
        )

        # Follow command (with BLE simulation)
        follow_cmd = self._follow.compute(target, self.robot.estimated_pose)

        # Obstacle avoidance from potential field (simulation)
        dv, dw = self.obstacles.avoidance_command(self.robot.true_state)
        avoid_correction = Twist(v=dv, w=dw)

        # Sector dummy for arbiter (potential field handles avoidance in sim)
        sectors = ObstacleSectors()

        # Blend
        cmd = self._arbiter.blend(follow_cmd, avoid_correction, sectors)

        # Accel limiting
        cmd = self._limiter.apply(cmd)

        # Apply to robot
        self.robot.set_velocity_command(cmd.v, cmd.w)
        self.robot.motor_update(self.dt)

        # Physics
        v_true, w_true = self.robot.physics_update(self.dt)

        # EKF
        self.robot.ekf_step(v_true, w_true, self.dt)

        # Trails
        self.trail_true.append(self.robot.true_state.copy())
        self.trail_est.append(self.robot.estimated_pose.copy())
        if len(self.trail_true) > self.max_trail:
            self.trail_true.pop(0)
            self.trail_est.pop(0)

        log.debug("step", extra={"time": self.time, "v": cmd.v, "w": cmd.w})
