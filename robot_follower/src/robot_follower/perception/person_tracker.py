"""Person tracker — wraps camera driver + detection model."""

import numpy as np
from collections import deque
from ..types.messages import PersonTarget


class PersonTracker:
    """
    Detects and tracks the nearest person using camera frames.

    In simulation the target position is injected directly via
    `inject_target()`; on hardware replace with a real detection model
    (e.g. MediaPipe, YOLOv8-pose).

    Parameters
    ----------
    smoothing : int
        Number of frames to average the position over.
    """

    def __init__(self, smoothing: int = 5):
        self._history: deque[np.ndarray] = deque(maxlen=smoothing)
        self._last: PersonTarget = PersonTarget()

    # ------------------------------------------------------------------
    # Simulation helpers
    # ------------------------------------------------------------------

    def inject_target(self, position: np.ndarray, confidence: float, timestamp: float) -> None:
        """Feed a ground-truth position directly (simulation only)."""
        self._history.append(position.copy())
        smoothed = np.mean(self._history, axis=0)
        self._last = PersonTarget(position=smoothed, confidence=confidence, timestamp=timestamp)

    # ------------------------------------------------------------------
    # Hardware path
    # ------------------------------------------------------------------

    def update_from_frame(self, frame: np.ndarray, timestamp: float) -> PersonTarget:
        """
        Run detection on a raw camera frame.

        Returns PersonTarget with confidence=0 if no person found.
        TODO: implement real detection.
        """
        return PersonTarget(confidence=0.0, timestamp=timestamp)

    @property
    def target(self) -> PersonTarget:
        return self._last
