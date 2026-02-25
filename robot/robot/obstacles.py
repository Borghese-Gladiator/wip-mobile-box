import numpy as np
from .utils import wrap_angle


class ObstacleField:
    """
    Potential-field repulsion. Returns a (v, w) correction to overlay
    on top of the follow controller output.
    """

    REPULSION_STRENGTH = 3.0
    INFLUENCE_RADIUS_FACTOR = 2.0  # multiples of obstacle radius

    def __init__(self, obstacles=None, radius=1.0):
        if obstacles is None:
            obstacles = [
                np.array([2.5,  1.5]),
                np.array([-3.0, -2.0]),
                np.array([0.0,  4.0]),
            ]
        self.obstacles = [np.array(o, dtype=float) for o in obstacles]
        self.radius = radius

    def avoidance_command(self, robot):
        """Return (delta_v, delta_w) repulsion corrections."""
        force = np.zeros(2)
        pos = robot.true_state[:2]

        for obs in self.obstacles:
            vec = pos - obs
            dist = np.linalg.norm(vec)

            if dist < self.radius * self.INFLUENCE_RADIUS_FACTOR:
                direction = vec / (dist + 1e-5)
                strength = self.REPULSION_STRENGTH / (dist ** 2 + 0.1)
                force += direction * strength

        if np.linalg.norm(force) == 0:
            return 0.0, 0.0

        desired_angle = np.arctan2(force[1], force[0])
        angle_error = wrap_angle(desired_angle - robot.true_state[2])

        # Translate to differential drive: turn toward safe direction
        delta_w = 2.0 * angle_error
        delta_v = -0.5 * np.linalg.norm(force)  # slow down near obstacles
        return delta_v, delta_w
