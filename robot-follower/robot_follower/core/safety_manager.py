"""Safety manager: watchdog, stall detection, e-stop."""

from __future__ import annotations

from .types import SafetyStatus, WheelCommand


class SafetyManager:
    """Monitors robot health and can override commands."""

    def __init__(self, watchdog_timeout: float = 1.0, stall_speed: float = 0.02, stall_time: float = 5.0):
        self._watchdog_timeout = watchdog_timeout
        self._stall_speed = stall_speed
        self._stall_time = stall_time
        self._last_heartbeat: float = 0.0
        self._stall_start: float | None = None
        self._e_stop = False

    def heartbeat(self, timestamp: float) -> None:
        self._last_heartbeat = timestamp

    def e_stop(self) -> None:
        self._e_stop = True

    def reset_e_stop(self) -> None:
        self._e_stop = False

    def check(self, robot_speed: float, timestamp: float) -> SafetyStatus:
        watchdog_expired = (timestamp - self._last_heartbeat) > self._watchdog_timeout

        if robot_speed < self._stall_speed:
            if self._stall_start is None:
                self._stall_start = timestamp
            stalled = (timestamp - self._stall_start) > self._stall_time
        else:
            self._stall_start = None
            stalled = False

        return SafetyStatus(
            e_stop=self._e_stop,
            stalled=stalled,
            watchdog_expired=watchdog_expired,
        )

    def gate_command(self, cmd: WheelCommand, status: SafetyStatus) -> WheelCommand:
        """Override command if safety requires a stop."""
        if status.e_stop or status.watchdog_expired:
            return WheelCommand(0.0, 0.0)
        return cmd
