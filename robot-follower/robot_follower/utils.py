"""Shared utility functions."""

from __future__ import annotations

import math
from pathlib import Path

import yaml


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp value to [lo, hi]."""
    return max(lo, min(hi, value))


def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolation from a to b by factor t."""
    return a + (b - a) * clamp(t, 0.0, 1.0)


def load_config(path: str | Path | None = None) -> dict:
    """Load sim_config.yaml. Defaults to config/sim_config.yaml relative to project root."""
    if path is None:
        path = Path(__file__).parent.parent / "config" / "sim_config.yaml"
    with open(path) as f:
        return yaml.safe_load(f)


def distance_2d(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Euclidean distance in 2D."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def bearing_to(
    from_pos: tuple[float, float],
    from_heading: float,
    to_pos: tuple[float, float],
) -> float:
    """Relative bearing from heading to target, in [-pi, pi]."""
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]
    absolute_angle = math.atan2(dy, dx)
    return normalize_angle(absolute_angle - from_heading)
