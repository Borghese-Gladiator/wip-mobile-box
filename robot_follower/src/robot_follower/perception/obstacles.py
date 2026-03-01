"""LiDAR obstacle perception — sectorize a scan and compute per-sector minimums."""

import math
import numpy as np
from ..types.messages import ObstacleSectors


def _wrap_deg(a: float) -> float:
    return a % 360.0


def sectorize(
    angles_deg: np.ndarray,
    ranges_m: np.ndarray,
    sector_defs: dict[str, tuple[float, float]] | None = None,
) -> ObstacleSectors:
    """
    Compute the minimum range in each named angular sector.

    Parameters
    ----------
    angles_deg : shape (N,) — bearing of each sample, 0=forward, CCW positive
    ranges_m   : shape (N,) — range in metres (inf for no return)
    sector_defs : mapping of sector name -> (lo_deg, hi_deg).
                  Angles are in [0, 360).  A sector that wraps through 0°
                  (e.g. rear: 150–210) is handled by checking both halves.

    Returns
    -------
    ObstacleSectors with the minimum range found in each sector.
    """
    if sector_defs is None:
        sector_defs = {
            "front":       (-30.0,  30.0),
            "front_left":  (30.0,   90.0),
            "front_right": (-90.0, -30.0),
            "rear":        (150.0, 210.0),
        }

    def min_in_sector(lo: float, hi: float) -> float:
        lo_w = _wrap_deg(lo)
        hi_w = _wrap_deg(hi)
        if lo_w <= hi_w:
            mask = (angles_deg >= lo_w) & (angles_deg < hi_w)
        else:  # wraps through 0°
            mask = (angles_deg >= lo_w) | (angles_deg < hi_w)
        vals = ranges_m[mask]
        return float(np.min(vals)) if len(vals) > 0 else math.inf

    return ObstacleSectors(
        front=min_in_sector(*sector_defs["front"]),
        front_left=min_in_sector(*sector_defs["front_left"]),
        front_right=min_in_sector(*sector_defs["front_right"]),
        rear=min_in_sector(*sector_defs["rear"]),
    )


class PotentialFieldObstacles:
    """
    Potential-field repulsion from a list of obstacle positions.

    Used in simulation where true obstacle locations are known.
    Returns a (delta_v, delta_w) correction to overlay on follow output.
    """

    REPULSION_STRENGTH = 3.0
    INFLUENCE_RADIUS_FACTOR = 2.0

    def __init__(self, obstacles=None, radius: float = 1.0):
        if obstacles is None:
            obstacles = [
                np.array([2.5,  1.5]),
                np.array([-3.0, -2.0]),
                np.array([0.0,  4.0]),
            ]
        self.obstacles = [np.array(o, dtype=float) for o in obstacles]
        self.radius = radius

    def avoidance_command(self, robot_true_state: np.ndarray) -> tuple[float, float]:
        """Return (delta_v, delta_w) repulsion corrections."""
        force = np.zeros(2)
        pos = robot_true_state[:2]

        for obs in self.obstacles:
            vec = pos - obs
            dist = np.linalg.norm(vec)
            if dist < self.radius * self.INFLUENCE_RADIUS_FACTOR:
                direction = vec / (dist + 1e-5)
                strength = self.REPULSION_STRENGTH / (dist ** 2 + 0.1)
                force += direction * strength

        if np.linalg.norm(force) == 0:
            return 0.0, 0.0

        desired_angle = math.atan2(force[1], force[0])
        angle_error = (desired_angle - robot_true_state[2] + math.pi) % (2 * math.pi) - math.pi

        delta_w = float(np.clip(2.0 * angle_error, -2.0, 2.0))
        delta_v = float(np.clip(-0.5 * np.linalg.norm(force), -1.0, 0.0))
        return delta_v, delta_w
