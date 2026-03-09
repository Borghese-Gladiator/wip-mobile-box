# Robot Follower — TODO

## Priority 1: Sensor Filtering & Confidence

- [ ] Wire EMA or median filter on BLE `signal_strength` and `bearing` before they reach the controller
- [ ] Add confidence decay: if no valid BLE packet for N control ticks, ramp `signal_strength` toward 0
- [ ] Wire `StaleTracker` into `SimRobotIO.read_target()` to set `stale=True` based on timestamp age
- [ ] Add temporal median filter on obstacle zone readings (front, front_left, front_right) to reject single-frame false positives
- [ ] Use `follow_distance` in `FollowerController` for proportional distance-keeping (currently only bearing + RSSI proximity)

## Priority 2: Structured Logging & Replay

- [ ] Populate `RobotTelemetry` dataclass each control tick with actual sensor/command/state data
- [ ] Write timestamped telemetry to a JSON-lines log file per run (`logs/run_YYYYMMDD_HHMMSS.jsonl`)
- [ ] Add `--log-dir` CLI flag to runner
- [ ] Create `tools/replay_logs.py` — reads a log file and prints summary stats (drift, avg speed, state transitions)
- [ ] Create `tools/plot_logs.py` — matplotlib plots of trajectory (true vs estimated), target distance over time, behavior state timeline

## Priority 3: Safety Completeness

- [ ] Add startup inhibit: don't send non-zero wheel commands until first valid IMU + wheel reading received
- [ ] Add sensor-fault detection: if `read_imu()` returns same value for N ticks, flag `sensor_fault`
- [ ] Add estimator-divergence check: if odometry drift rate exceeds threshold, flag warning
- [ ] Add `SafetyStatus.low_battery` check (simulated for now, real on hardware)
- [ ] Make `STOPPED` state in behavior manager unrecoverable without explicit reset

## Priority 4: Scenario Testing & Validation

- [ ] Create `tools/run_sweep.py` — runs N headless simulations with different random seeds, reports success rate and drift stats
- [ ] Add regression test scenarios: target behind obstacle, target reverses direction, BLE blackout for 5s
- [ ] Add failure-case unit tests: estimator fed garbage IMU data, all BLE packets dropped, front+left+right all blocked
- [ ] Define acceptance criteria: success rate > X% across 100 seeds, max drift < Y meters at 60s

## Priority 5: Simulation Realism Improvements

- [ ] Battery-voltage scaling: multiply motor output by `battery_factor` (0.7–1.0) that decays over time
- [ ] Wheel slip: add small random perturbation to effective wheel radius under high torque
- [ ] Loop timing jitter: add random ±2ms variation to `control_dt` each tick
- [ ] Obstacle sensor false negatives: small probability of a raycast returning max-range even when obstacle is present

## Priority 6: Hardware Bring-Up (when Pi arrives)

- [ ] `hardware/pi_runner.py` — main loop on Raspberry Pi, same structure as `mujoco_sim/runner.py`
- [ ] `hardware/motor_driver.py` — PWM output to motor controller (L298N / DRV8833)
- [ ] `hardware/encoders.py` — GPIO interrupt-based tick counting
- [ ] `hardware/imu.py` — I2C read from MPU6050 / BNO055
- [ ] `hardware/ble_scanner.py` — BLE RSSI + bearing from beacon
- [ ] `hardware/range_sensors.py` — HC-SR04 ultrasonic or VL53L0X ToF
- [ ] `hardware/battery_monitor.py` — ADC voltage divider reading
- [ ] `hardware/watchdog.py` — hardware watchdog timer integration
- [ ] Implement `HardwareRobotIO(RobotIO)` wiring all the above

## Priority 7: Calibration Tooling

- [ ] `tools/calibrate_wheels.py` — drive straight for N seconds, measure actual vs expected distance, compute wheel radius correction
- [ ] `tools/calibrate_imu.py` — stationary IMU capture, compute gyro bias offset
- [ ] `tools/tune_controller.py` — automated parameter sweep of steering_gain, base_speed, avoid_weight with fitness metric
