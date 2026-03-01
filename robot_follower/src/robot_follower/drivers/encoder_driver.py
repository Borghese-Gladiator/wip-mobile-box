"""Wheel encoder driver — reads tick counts from hardware."""


class EncoderDriver:
    """
    Reads left/right encoder tick counts.

    Hardware: Hall-effect or optical encoders wired to GPIO interrupt pins.
    Replace stub methods with real I/O when running on hardware.
    """

    def __init__(self, ticks_per_rev: int = 360, wheel_radius: float = 0.1):
        self.ticks_per_rev = ticks_per_rev
        self.wheel_radius = wheel_radius
        self._left_ticks = 0
        self._right_ticks = 0

    def read_ticks(self) -> tuple[int, int]:
        """Return (left_ticks, right_ticks) accumulated since last call."""
        # TODO: read from hardware counters / GPIO
        left, right = self._left_ticks, self._right_ticks
        self._left_ticks = 0
        self._right_ticks = 0
        return left, right

    def ticks_to_metres(self, ticks: int) -> float:
        import math
        return ticks * (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
