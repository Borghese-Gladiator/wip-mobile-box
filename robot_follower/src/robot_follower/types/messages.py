"""Shared dataclasses that flow between modules."""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class Twist:
    """Linear + angular velocity command."""
    v: float = 0.0   # m/s
    w: float = 0.0   # rad/s


@dataclass
class PersonTarget:
    """Person detection result from the camera tracker."""
    position: np.ndarray = field(default_factory=lambda: np.zeros(2))
    confidence: float = 0.0   # 0–1
    timestamp: float = 0.0    # seconds


@dataclass
class ObstacleSectors:
    """Minimum lidar range per angular sector."""
    front: float = float("inf")
    front_left: float = float("inf")
    front_right: float = float("inf")
    rear: float = float("inf")


@dataclass
class Odometry:
    """Differential-drive odometry estimate."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    w: float = 0.0
    timestamp: float = 0.0
