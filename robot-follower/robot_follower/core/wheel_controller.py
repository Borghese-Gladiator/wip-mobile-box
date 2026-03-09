"""Low-level wheel controller: twist → wheel commands with deadband/saturation."""

from __future__ import annotations

from .types import Twist2D, WheelCommand
from .kinematics import twist_to_wheels
from .math_utils import clamp


class WheelController:
    """Converts body twist to wheel commands, applying motor realism."""

    def __init__(
        self,
        wheel_radius: float,
        wheelbase: float,
        max_wheel_speed: float = 10.0,
        deadband_threshold: float = 0.3,
        left_gain: float = 1.0,
        right_gain: float = 1.0,
    ):
        self._wheel_radius = wheel_radius
        self._wheelbase = wheelbase
        self._max = max_wheel_speed
        self._deadband = deadband_threshold
        self._left_gain = left_gain
        self._right_gain = right_gain

    def compute(self, twist: Twist2D) -> WheelCommand:
        cmd = twist_to_wheels(
            twist.linear, twist.angular,
            self._wheel_radius, self._wheelbase, self._max,
        )
        left = self._apply_motor(cmd.left, self._left_gain)
        right = self._apply_motor(cmd.right, self._right_gain)
        return WheelCommand(left=left, right=right)

    def _apply_motor(self, speed: float, gain: float) -> float:
        if abs(speed) < self._deadband:
            return 0.0
        return clamp(speed * gain, -self._max, self._max)
