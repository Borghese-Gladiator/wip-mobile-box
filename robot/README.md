# Robot

## Connectivity
- Bluetooth Low Energy (BLE)
- Sends telemetry:
  - Battery level
  - Position estimate
  - IMU data
  - Motor state
- BLE RSSI — rough distance to phone

## Movement
- Differential drive (tank style)
  - Left motor, right motor
  - Speed control via PWM
  - Turning via differential velocity

---

# Equipment
Tires
* 8–10 inch pneumatic tires
* Rubber, air-filled
* Deep tread
* Benefits:
  * Absorb shock
  * Handle cracks
  * Climb small curbs
  * Less slip
  * Lower energy use
* Add:
  * Slightly wide wheel base for stability

Chip
- ESP32 (does BLE + motor control)
  - Cheap
  - Integrated Bluetooth
  - Real-time capable
* Raspberry Pi 5 (high-level logic)
* Arduino Nano (motor control)

Motor
- 2x 12V DC gear motors
- two large wheels
- Motor driver:
  - L298N (cheap)
  - TB6612FNG (light robot)
  - Cytron MD10C (more power)


Sensors
- Wheel encoders - Odometry (distance + heading)
  - Hall effect encoders?
  - optical encoders?
- IMU (gyro + accel) - Orientation + smoothing
  - MPU6050? (cheap)
  - BNO055? (better — does onboard fusion)
- Ultrasonic or LiDAR - Obstacle detection
  - HC-SR04 ultrasonic (cheap) -> prob this one!
  - RPLidar A1 (real robot-level)
  - (upgrade) UWB

Power
* 3S or 4S Li-ion pack
* Separate logic regulator
* Fuse + cutoff switch

Safety
* Emergency stop button
* Watchdog timer


#### (Robot) Non-Functional Requirements
- Max speed: 1.5 m/s
- Follow distance: 1–2 meters
- Latency: <200ms control loop
- Battery life: 1–2 hours
- Outdoor and indoor capable


# Responsibilites
### 1️⃣ Wheel Encoder Reading

* Measure actual wheel velocity
* Calculate odometry

### 2️⃣ PID Motor Control

Closed-loop control:

```
error = target_velocity - measured_velocity
output = PID(error)
```

This compensates for:

* slope
* battery drop
* wheel slip

### 3️⃣ IMU Stabilization

* Fuse gyro + accelerometer
* Smooth heading estimate
* Reduce drift

### 4️⃣ Obstacle Detection

Ultrasonic or LiDAR:

If obstacle distance < threshold:

* Override phone
* Slow or stop

### 5️⃣ Dead Reckoning

Compute position from:

* Encoders
* IMU

Send estimated pose back to phone.

### 6️⃣ Bluetooth Timeout Safety

If no command received for 500 ms:

* Stop motors
* Enter safe state

### 7️⃣ Acceleration Limiting

Robot must clamp:

```
dv/dt <= max_accel
dω/dt <= max_angular_accel
```

Phone should not directly control acceleration.