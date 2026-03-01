"""Differential-drive odometry computed from wheel encoder ticks."""

import math
from ..types.messages import Odometry


def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class DiffDriveOdometry:
    """
    Integrates left/right encoder distances into a 2-D pose estimate.

    Parameters
    ----------
    track_width : float
        Distance between the two wheels (metres).
    initial_x, initial_y, initial_theta : float
        Starting pose.
    """

    def __init__(
        self,
        track_width: float = 0.5,
        initial_x: float = 0.0,
        initial_y: float = 0.0,
        initial_theta: float = 0.0,
    ):
        self.track_width = track_width
        self.x = initial_x
        self.y = initial_y
        self.theta = initial_theta
        self.v = 0.0
        self.w = 0.0

    def update(self, left_dist: float, right_dist: float, dt: float) -> Odometry:
        """
        Integrate one step.

        Parameters
        ----------
        left_dist  : metres travelled by the left wheel this step
        right_dist : metres travelled by the right wheel this step
        dt         : elapsed time for this step (seconds)

        Returns
        -------
        Current Odometry estimate.
        """
        d = (left_dist + right_dist) / 2.0
        dtheta = (right_dist - left_dist) / self.track_width

        self.x += d * math.cos(self.theta + dtheta / 2.0)
        self.y += d * math.sin(self.theta + dtheta / 2.0)
        self.theta = _wrap_angle(self.theta + dtheta)

        self.v = d / dt if dt > 0 else 0.0
        self.w = dtheta / dt if dt > 0 else 0.0

        return Odometry(x=self.x, y=self.y, theta=self.theta, v=self.v, w=self.w)
