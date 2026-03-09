"""Pure math utilities — no external dependencies beyond stdlib."""

from __future__ import annotations

import math


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * clamp(t, 0.0, 1.0)


def distance_2d(a: tuple[float, float], b: tuple[float, float]) -> float:
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


def quat_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
    """Extract yaw from quaternion (w, x, y, z)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)
