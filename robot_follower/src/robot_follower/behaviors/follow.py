"""Follow behaviour — compute a Twist toward a PersonTarget."""

import math
from collections import deque
from ..types.messages import PersonTarget, Twist

import numpy as np


def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class FollowBehavior:
    """
    Proportional controller that drives the robot toward the target person.

    Also simulates BLE latency + packet drop when used in simulation.

    Parameters
    ----------
    desired_distance : float
        Stop when this close to the target (metres).
    kp_dist : float
        P-gain for linear speed.
    kp_angle : float
        P-gain for angular speed.
    max_linear_cmd : float
        Clamp on linear velocity output.
    ble_latency_seconds : float
        Simulated one-way BLE delay.
    ble_packet_drop_rate : float
        Probability [0, 1] that a packet is dropped.
    dt : float
        Control loop period (seconds).
    """

    def __init__(
        self,
        desired_distance: float = 1.0,
        kp_dist: float = 1.0,
        kp_angle: float = 3.0,
        max_linear_cmd: float = 1.0,
        ble_latency_seconds: float = 0.4,
        ble_packet_drop_rate: float = 0.1,
        dt: float = 0.05,
    ):
        self.desired_distance = desired_distance
        self.kp_dist = kp_dist
        self.kp_angle = kp_angle
        self.max_linear_cmd = max_linear_cmd
        self.drop_rate = ble_packet_drop_rate

        buf_size = max(1, int(ble_latency_seconds / dt))
        self._ble_buffer: deque = deque(maxlen=buf_size)
        for _ in range(buf_size):
            self._ble_buffer.append(None)
        self._last_good_cmd = Twist()

    def compute(self, target: PersonTarget, robot_estimated_pose: np.ndarray) -> Twist:
        """
        Compute a velocity command toward *target* given the robot's current
        estimated pose [x, y, theta].

        Applies BLE latency + packet-drop simulation.
        """
        cmd = self._raw_command(target, robot_estimated_pose)

        if np.random.rand() < self.drop_rate:
            self._ble_buffer.append(None)
        else:
            self._ble_buffer.append(cmd)

        received = self._ble_buffer[0]
        if received is not None:
            self._last_good_cmd = received

        return self._last_good_cmd

    def _raw_command(self, target: PersonTarget, pose: np.ndarray) -> Twist:
        error = target.position - pose[:2]
        dist = float(np.linalg.norm(error))

        if dist < self.desired_distance:
            return Twist()

        desired_angle = math.atan2(error[1], error[0])
        angle_error = _wrap_angle(desired_angle - pose[2])

        v = min(self.kp_dist * dist, self.max_linear_cmd)
        w = self.kp_angle * angle_error
        return Twist(v=v, w=w)
