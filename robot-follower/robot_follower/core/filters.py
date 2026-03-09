"""Signal filters for noisy sensor data."""

from __future__ import annotations

import math
from collections import deque


class ExponentialMovingAverage:
    """EMA filter with configurable smoothing factor."""

    def __init__(self, alpha: float = 0.2):
        self._alpha = alpha
        self._value: float | None = None

    def update(self, raw: float) -> float:
        if self._value is None:
            self._value = raw
        else:
            self._value = self._alpha * raw + (1.0 - self._alpha) * self._value
        return self._value

    @property
    def value(self) -> float:
        return self._value if self._value is not None else 0.0

    def reset(self) -> None:
        self._value = None


class MedianFilter:
    """Sliding-window median filter."""

    def __init__(self, window: int = 5):
        self._window = window
        self._buf: deque[float] = deque(maxlen=window)

    def update(self, raw: float) -> float:
        self._buf.append(raw)
        return sorted(self._buf)[len(self._buf) // 2]

    @property
    def value(self) -> float:
        if not self._buf:
            return 0.0
        return sorted(self._buf)[len(self._buf) // 2]

    def reset(self) -> None:
        self._buf.clear()


def deadband(value: float, threshold: float) -> float:
    """Return 0 if abs(value) < threshold, else value."""
    return 0.0 if abs(value) < threshold else value


class StaleTracker:
    """Tracks whether a data source is stale based on timestamps."""

    def __init__(self, max_age: float = 0.5):
        self._max_age = max_age
        self._last_update: float = -math.inf

    def mark_updated(self, timestamp: float) -> None:
        self._last_update = timestamp

    def is_stale(self, current_time: float) -> bool:
        return (current_time - self._last_update) > self._max_age
