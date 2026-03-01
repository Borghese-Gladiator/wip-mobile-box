"""Motor driver interface — wraps hardware PWM / serial commands."""


class MotorDriver:
    """
    Sends left/right velocity commands to the motor controller.

    Hardware: L298N / TB6612FNG / Cytron MD10C via ESP32 PWM or serial.
    Replace the stub methods with real I/O when running on hardware.
    """

    def __init__(self, max_speed: float = 2.0):
        self.max_speed = max_speed
        self._left_cmd = 0.0
        self._right_cmd = 0.0

    def set_wheel_speeds(self, left: float, right: float) -> None:
        self._left_cmd = max(-self.max_speed, min(self.max_speed, left))
        self._right_cmd = max(-self.max_speed, min(self.max_speed, right))
        self._write()

    def stop(self) -> None:
        self.set_wheel_speeds(0.0, 0.0)

    def _write(self) -> None:
        # TODO: write PWM / serial bytes to hardware
        pass
