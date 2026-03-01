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

---

## Changelog

### 2026-03-01 — Package restructure + TDD test suite

#### What changed
- Renamed and restructured `robot/` into `robot_follower/` with a layered
  architecture: `types`, `drivers`, `estimation`, `perception`, `behaviors`,
  `control`, `app`.
- Extracted all magic numbers into `config/robot.yaml`, `config/follow.yaml`,
  `config/avoid.yaml`.
- Ported simulation logic (`World`, `_SimRobot`, `_SimPhone`) into
  `app/main.py`.
- Added hardware-stub drivers: `motor_driver.py`, `encoder_driver.py`,
  `lidar_driver.py`, `camera_driver.py` — all `pass` / `return None`.
- Added `estimation/odom.py` (`DiffDriveOdometry`) and
  `perception/obstacles.py` (`sectorize`, `PotentialFieldObstacles`).
- Added `control/diff_drive.py` (`DiffDriveKinematics`) and
  `control/limits.py` (`AccelLimiter`).

#### TDD implemented
The following modules were written with tests **before** or alongside
implementation to lock in their behavioral contract:

| Module | Test file | Cases |
|---|---|---|
| `control/diff_drive.py` | `test_diff_drive.py` | 16 — forward kinematics, inverse kinematics, clamping, roundtrip |
| `control/limits.py` | `test_limits.py` | 11 — single-step clamp, ramp, decel, reset |
| `estimation/odom.py` | `test_odom.py` | 13 — straight, arc, spin, angle wrap, multi-step, velocity output |
| `perception/obstacles.py` — `sectorize` | `test_sectorize.py` | 10 — sector routing, boundary membership, minimum selection, empty scan |

#### Tests rewritten (existing files, upgraded from smoke tests to contracts)

| File | Before | After | Key additions |
|---|---|---|---|
| `test_follow.py` | 3 cases, sign checks only | 8 cases, exact v+w | Speed cap, angle cases, rotated robot, behind-target wrap |
| `test_avoid.py` | 3 cases, `v < 0` only | 9 cases, exact v+w | Turn direction (left/right), turn_bias scaling, all boundaries |
| `test_arbiter.py` | 4 cases | 9 cases | Negative v clamp, angular clamp ±, estop boundary (== and just above) |

**Total: 77 tests, all passing.**

One notable fix during the run: `test_target_directly_behind` revealed that
`wrap(π)` returns `−π` in floating-point (the ±π boundary is ambiguous). The
assertion was corrected to `abs(w) ≈ 3π` to be implementation-agnostic.

---

## Next steps

- [ ] **Wire LiDAR into the control loop** — `LidarDriver.read_scan()` →
  `sectorize()` → `ObstacleSectors` → `AvoidBehavior` + `Arbiter`. Currently
  `ObstacleSectors` is always empty so avoidance never fires.
- [ ] **Implement `MotorDriver._write()`** — choose PWM vs serial protocol for
  the target motor controller IC, write bytes to GPIO/UART.
- [ ] **Implement `EncoderDriver.read_ticks()`** — attach GPIO interrupt
  handlers (RPi.GPIO or pigpio) that count pulses.
- [ ] **Implement `CameraDriver.read_frame()`** — picamera2 or OpenCV capture.
- [ ] **Implement `PersonTracker.update_from_frame()`** — MediaPipe Pose or
  YOLOv8-pose; output real `PersonTarget` with position + confidence.
- [ ] **Extract EKF** — move `_SimRobot.ekf_step` into `estimation/ekf.py`;
  replace simulated GPS update with encoder odometry from `DiffDriveOdometry`.
- [ ] **Add `test_ekf.py`** — TDD for the extracted EKF module (predict step,
  GPS update, IMU update, covariance growth).
- [ ] **Add `test_potential_field.py`** — TDD for `PotentialFieldObstacles`
  (robot inside influence radius gets repulsion, outside gets zero, force
  direction is away from obstacle).
