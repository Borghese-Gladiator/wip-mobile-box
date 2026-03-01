"""Acceleration limits and speed clamping."""

from ..types.messages import Twist


class AccelLimiter:
    """
    Clamp the rate-of-change of v and w between control steps.

    Prevents the robot from commanding sudden jerks that could cause
    wheel slip or tip-over.

    Parameters
    ----------
    max_accel : float
        Maximum linear acceleration (m/s²).
    max_alpha : float
        Maximum angular acceleration (rad/s²).
    dt : float
        Control loop period (seconds).
    """

    def __init__(self, max_accel: float = 1.0, max_alpha: float = 2.0, dt: float = 0.05):
        self.max_accel = max_accel
        self.max_alpha = max_alpha
        self.dt = dt
        self._prev = Twist()

    def apply(self, desired: Twist) -> Twist:
        """Return a new Twist with accelerations clamped."""
        dv = desired.v - self._prev.v
        dw = desired.w - self._prev.w

        max_dv = self.max_accel * self.dt
        max_dw = self.max_alpha * self.dt

        dv = max(-max_dv, min(max_dv, dv))
        dw = max(-max_dw, min(max_dw, dw))

        out = Twist(v=self._prev.v + dv, w=self._prev.w + dw)
        self._prev = out
        return out

    def reset(self) -> None:
        self._prev = Twist()
