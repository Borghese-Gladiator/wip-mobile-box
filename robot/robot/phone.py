import numpy as np
from collections import deque
from .utils import wrap_angle


class Phone:
    """
    Moves in a circle. Computes follow commands from the robot's EKF
    estimate and transmits them over a simulated BLE link.
    """

    LATENCY_SECONDS = 0.4
    PACKET_DROP_RATE = 0.1
    FOLLOW_DISTANCE = 1.0
    Kp_DIST = 1.0
    Kp_ANGLE = 3.0
    PATH_RADIUS = 4.0
    PATH_SPEED = 0.3
    MAX_V_CMD = 1.0  # half of Robot.MAX_SPEED, leaves headroom for w_cmd differential

    def __init__(self, dt):
        self.time = 0.0
        self.dt = dt

        buf_size = max(1, int(self.LATENCY_SECONDS / dt))
        self._ble_buffer = deque(maxlen=buf_size)

        # Seed buffer with Nones (no command yet)
        for _ in range(buf_size):
            self._ble_buffer.append(None)

        self._last_good_cmd = (0.0, 0.0)

    def update(self):
        self.time += self.dt

    def true_position(self):
        angle = self.PATH_SPEED * self.time
        return np.array([
            self.PATH_RADIUS * np.cos(angle),
            self.PATH_RADIUS * np.sin(angle)
        ])

    def _compute_raw_command(self, robot_estimated_pose):
        error = self.true_position() - robot_estimated_pose[:2]
        dist = np.linalg.norm(error)

        if dist < self.FOLLOW_DISTANCE:
            return 0.0, 0.0

        desired_angle = np.arctan2(error[1], error[0])
        angle_error = wrap_angle(desired_angle - robot_estimated_pose[2])

        v = min(self.Kp_DIST * dist, self.MAX_V_CMD)
        w = self.Kp_ANGLE * angle_error
        return v, w

    def transmit(self, robot_estimated_pose):
        """
        Compute command, simulate BLE packet drop + latency buffer,
        and return the (possibly delayed/dropped) command received now.
        """
        cmd = self._compute_raw_command(robot_estimated_pose)

        # Simulate packet drop
        if np.random.rand() < self.PACKET_DROP_RATE:
            self._ble_buffer.append(None)
        else:
            self._ble_buffer.append(cmd)

        # Oldest packet in buffer is what robot receives now
        received = self._ble_buffer[0]
        if received is not None:
            self._last_good_cmd = received

        return self._last_good_cmd
