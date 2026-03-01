# Robot Follower — Development Log

## Simulated components (need real hardware implementations)

### Person / target tracking
- [ ] **Person detection** (`perception/person_tracker.py` — `update_from_frame`)
  Currently returns `confidence=0` unconditionally. Replace with a real model
  (MediaPipe Pose, YOLOv8-pose, or similar) that detects and localises a person
  in a camera frame.
- [ ] **Target position** (`app/main.py` — `_SimPhone`)
  The "person" is a point moving in a perfect circle at a fixed radius and speed.
  On hardware the position must come from the camera tracker (or BLE RSSI ranging).
- [ ] **BLE link** (`behaviors/follow.py` — `FollowBehavior`)
  Latency (400 ms) and packet-drop (10 %) are modelled as a ring buffer of random
  drops. Replace with a real BLE connection to the phone (e.g. Bleak on Pi side,
  CoreBluetooth / Android BLE on phone side).

### Obstacle sensing
- [ ] **LiDAR scan** (`drivers/lidar_driver.py` — `read_scan`)
  Always returns `None`. Replace with RPLidar A1 SDK calls (pyserial + rplidar
  library) that stream real 360° range data.
- [ ] **Obstacle avoidance input** (`app/main.py` — `World.step`)
  `ObstacleSectors` is constructed empty (all ranges = inf), so `AvoidBehavior`
  and `Arbiter` never trigger. Wire `LidarDriver → sectorize() → ObstacleSectors`
  and pass the result into `AvoidBehavior.compute` and `Arbiter.blend`.
- [ ] **Obstacle positions** (`perception/obstacles.py` — `PotentialFieldObstacles`)
  Three hard-coded world-frame positions are used. The potential-field approach
  also cheats by reading `robot.true_state` (ground truth). On hardware, obstacle
  positions must come from LiDAR sector minimums via `AvoidBehavior`.

### Robot physics & sensors
- [ ] **Motor output** (`drivers/motor_driver.py` — `_write`)
  `pass` — no PWM or serial bytes are sent. Implement for the chosen motor driver
  IC (L298N / TB6612FNG / Cytron MD10C) via GPIO PWM or UART.
- [ ] **Encoder input** (`drivers/encoder_driver.py` — `read_ticks`)
  Tick counters are always zero. Wire GPIO interrupt handlers (RPi.GPIO or
  pigpio) that increment `_left_ticks` / `_right_ticks` on each encoder pulse.
- [ ] **Camera input** (`drivers/camera_driver.py` — `read_frame`)
  Always returns `None`. Implement with `picamera2` or OpenCV `VideoCapture`.
- [ ] **Robot physics** (`app/main.py` — `_SimRobot.physics_update`)
  Gaussian wheel-slip noise is injected each step to stand in for real surface
  friction variation. On hardware this disappears — real slip will be handled by
  the encoder feedback loop.
- [ ] **IMU sensor** (`app/main.py` — `_SimRobot._read_imu`)
  Heading is the true heading plus Gaussian noise (std = 0.02 rad). Replace with
  real MPU-6050 / BNO055 reads over I²C.
- [ ] **GPS sensor** (`app/main.py` — `_SimRobot._read_gps`)
  Position is true position plus a slow random drift bias plus Gaussian noise.
  On an indoor robot there is no GPS; replace with UWB ranging or remove the GPS
  update from the EKF entirely and rely on encoders + IMU only.

### State estimation
- [ ] **EKF sensor fusion** (`app/main.py` — `_SimRobot.ekf_step`)
  Fuses simulated GPS + IMU. On hardware, replace the GPS update with encoder
  odometry (dead reckoning) and optionally UWB; keep IMU update but read from
  real hardware. Consider extracting EKF into its own `estimation/ekf.py` module.
