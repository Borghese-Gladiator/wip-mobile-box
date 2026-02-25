# Log
Log for Learning + Issues encountered

**02/22**

**02/21**

# Glossary
- Bleak - Python library to communicate with BLE devices
- BLE (Bluetooth Low Energy)
  - low-duty cycle, packat-based timing model for power efficiency where device remains in sleep and wake briefly to exchange data
- ROS (Robot Operating Systm) - simplify interacting with hardware, but very hard to customize and an opt-in framework (with a learning curve) -> I'm against for now unless I know a reason we should use it
- Kalman Filter - optimal, recursive, linear algorithm to estimate state of dynamic system (eg: position, velocity) from noisy measurements over time
- PID Motor Loop - feedback mechanism to measure motor's speed or position. It continuously calculates difference between desired target (Setpoint) and motor's actual state (Process Variable).
  - Proportional (P) - muscle -> corrective foce proportional to current error
  - Integral (I) - persistence -> ensures motor reaches exact target with steady "nudge" if small error persists
  - Deriviate (D) - brake -> predicts future error by measuring rate at which error is changing
- Sensor Fusion - combine data from multiple sensors (cameras, LiDAR, radar, IMUs) to produce more accurate, reliable than what any single sensor alone can do.
  - Camera
  - LiDAR
  - Radar
  - IMU - inertial measurement unit is an electronic sensor that measures and reports objects specific force (acceleration), angular rate (rotation), and sometimes magnetic field with accelerometers, gyroscopes, and magnetometers


Hardware
- Wheel Encoder - electromechanical device that measures rotational speed, position, or distance traveled by wheel
  - works by converting rotation into electrical pulses via optical or magnetic sensors