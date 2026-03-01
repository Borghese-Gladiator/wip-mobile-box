"""LiDAR driver — reads a full 360° scan."""

from typing import Optional
import numpy as np


class LidarDriver:
    """
    Returns a 1-D array of range measurements (metres) indexed by angle.

    Hardware: RPLidar A1 via serial.  Replace stub with rplidar SDK calls.
    angles_deg: np.ndarray of shape (N,) — bearing of each range sample
    ranges_m:   np.ndarray of shape (N,) — corresponding range in metres
    """

    def read_scan(self) -> Optional[tuple[np.ndarray, np.ndarray]]:
        """
        Return (angles_deg, ranges_m) or None if no scan available.
        angles_deg: 0–360, measured CCW from robot forward.
        ranges_m:   inf where no return.
        """
        # TODO: read from RPLidar SDK
        return None
