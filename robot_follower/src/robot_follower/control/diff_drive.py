"""Convert a Twist (v, w) into per-wheel speed commands."""

from ..types.messages import Twist


class DiffDriveKinematics:
    """
    Inverse kinematics for a differential-drive robot.

    v_left  = v - (w * track_width / 2)
    v_right = v + (w * track_width / 2)

    Parameters
    ----------
    track_width : float
        Distance between wheels (metres).
    max_speed : float
        Per-wheel speed limit (m/s).
    """

    def __init__(self, track_width: float = 0.5, max_speed: float = 2.0):
        self.track_width = track_width
        self.max_speed = max_speed

    def twist_to_wheels(self, cmd: Twist) -> tuple[float, float]:
        """Return (left_speed, right_speed) in m/s, clamped to max_speed."""
        half = self.track_width / 2.0
        left = cmd.v - cmd.w * half
        right = cmd.v + cmd.w * half
        left = max(-self.max_speed, min(self.max_speed, left))
        right = max(-self.max_speed, min(self.max_speed, right))
        return left, right

    def wheels_to_twist(self, left: float, right: float) -> Twist:
        """Forward kinematics: wheel speeds -> body Twist."""
        v = (left + right) / 2.0
        w = (right - left) / self.track_width
        return Twist(v=v, w=w)
