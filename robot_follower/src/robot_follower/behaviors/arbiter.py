"""Arbiter — safety gating and blending of follow + avoid commands."""

from ..types.messages import Twist, ObstacleSectors


class Arbiter:
    """
    Combines follow and avoidance Twists with safety override logic.

    Priority
    --------
    1. Hard stop if any front sector is within *estop_distance*.
    2. Avoidance correction blended additively onto follow command.
    3. Output clamped to configured speed limits.

    Parameters
    ----------
    estop_distance : float
        Absolute stop threshold (metres).
    max_linear : float
        Maximum allowed linear speed after blending (m/s).
    max_angular : float
        Maximum allowed angular speed after blending (rad/s).
    """

    def __init__(
        self,
        estop_distance: float = 0.3,
        max_linear: float = 1.5,
        max_angular: float = 4.0,
    ):
        self.estop_distance = estop_distance
        self.max_linear = max_linear
        self.max_angular = max_angular

    def blend(
        self,
        follow_cmd: Twist,
        avoid_correction: Twist,
        sectors: ObstacleSectors,
    ) -> Twist:
        """
        Return the final Twist to send to the motors.

        Emergency-stop overrides everything when an obstacle is too close.
        """
        if sectors.front <= self.estop_distance:
            return Twist(v=0.0, w=0.0)

        v = follow_cmd.v + avoid_correction.v
        w = follow_cmd.w + avoid_correction.w

        v = max(-self.max_linear, min(self.max_linear, v))
        w = max(-self.max_angular, min(self.max_angular, w))

        return Twist(v=v, w=w)
