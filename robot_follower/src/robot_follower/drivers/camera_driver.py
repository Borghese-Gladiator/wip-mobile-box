"""Camera driver — captures frames for person detection."""

from typing import Optional
import numpy as np


class CameraDriver:
    """
    Returns raw BGR frames from the camera.

    Hardware: Raspberry Pi camera module via picamera2 or OpenCV.
    Replace stub with real capture calls.
    """

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self.width = width
        self.height = height
        self.fps = fps

    def read_frame(self) -> Optional[np.ndarray]:
        """Return a (H, W, 3) uint8 BGR frame, or None if unavailable."""
        # TODO: cap.read() from OpenCV / picamera2
        return None
