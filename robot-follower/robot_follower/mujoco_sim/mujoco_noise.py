"""Centralized noise injection for MuJoCo simulation realism."""

from __future__ import annotations

import math
import random


class MotorNoise:
    """Motor imperfections: deadband, mismatch, lag."""

    def __init__(
        self,
        left_gain: float = 1.0,
        right_gain: float = 0.97,
        response_lag: float = 0.02,
    ):
        self.left_gain = left_gain
        self.right_gain = right_gain
        self.response_lag = response_lag
        self._left_actual = 0.0
        self._right_actual = 0.0

    def apply(self, left_cmd: float, right_cmd: float, dt: float) -> tuple[float, float]:
        alpha = min(1.0, dt / max(self.response_lag, 1e-6))
        target_left = left_cmd * self.left_gain
        target_right = right_cmd * self.right_gain
        self._left_actual += alpha * (target_left - self._left_actual)
        self._right_actual += alpha * (target_right - self._right_actual)
        return (self._left_actual, self._right_actual)


class IMUNoise:
    """Simulated IMU with gyro bias drift and noise."""

    def __init__(self, gyro_noise_std: float = 0.01, gyro_bias_drift: float = 0.001):
        self._noise_std = gyro_noise_std
        self._bias_drift = gyro_bias_drift
        self._bias = 0.0

    def apply(self, true_gyro_z: float, dt: float) -> float:
        self._bias += random.gauss(0, self._bias_drift * math.sqrt(dt))
        return true_gyro_z + self._bias + random.gauss(0, self._noise_std)


class EncoderNoise:
    """Simulated encoder with quantization and occasional errors."""

    def __init__(self, ticks_per_rev: int = 1440, error_rate: float = 0.001):
        self._ticks_per_rev = ticks_per_rev
        self._error_rate = error_rate
        self._accum_left = 0.0
        self._accum_right = 0.0
        self._ticks_left = 0
        self._ticks_right = 0

    def update(self, left_speed: float, right_speed: float, dt: float) -> tuple[int, int]:
        """Convert wheel speeds to quantized tick counts."""
        rad_per_tick = 2.0 * math.pi / self._ticks_per_rev
        self._accum_left += left_speed * dt
        self._accum_right += right_speed * dt

        new_left = int(self._accum_left / rad_per_tick)
        new_right = int(self._accum_right / rad_per_tick)
        self._accum_left -= new_left * rad_per_tick
        self._accum_right -= new_right * rad_per_tick

        # Occasional missed ticks
        if random.random() < self._error_rate and new_left != 0:
            new_left -= 1 if new_left > 0 else -1
        if random.random() < self._error_rate and new_right != 0:
            new_right -= 1 if new_right > 0 else -1

        self._ticks_left += new_left
        self._ticks_right += new_right
        return (self._ticks_left, self._ticks_right)


class BLENoise:
    """Simulated BLE signal: noisy RSSI, delayed/dropped packets."""

    def __init__(
        self,
        rssi_noise_std: float = 2.0,
        bearing_noise_std: float = 0.05,
        drop_rate: float = 0.05,
        delay_steps: int = 2,
    ):
        self._rssi_noise = rssi_noise_std
        self._bearing_noise = bearing_noise_std
        self._drop_rate = drop_rate
        self._delay_steps = delay_steps
        self._buffer: list[tuple[float, float, float]] = []  # (rssi, bearing, dist)

    def apply(
        self, true_rssi: float, true_bearing: float, true_dist: float,
    ) -> tuple[float, float, float, bool]:
        """Returns (noisy_rssi, noisy_bearing, noisy_dist, valid)."""
        entry = (
            true_rssi + random.gauss(0, self._rssi_noise),
            true_bearing + random.gauss(0, self._bearing_noise),
            true_dist,
        )
        self._buffer.append(entry)

        if len(self._buffer) <= self._delay_steps:
            return (0.0, 0.0, 0.0, False)

        delayed = self._buffer.pop(0)
        if random.random() < self._drop_rate:
            return (0.0, 0.0, 0.0, False)

        return (delayed[0], delayed[1], delayed[2], True)
