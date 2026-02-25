# 🤖 Follow Algorithm

You have two main approaches:

---

## Option 1: BLE RSSI Following (Simple)

Robot measures signal strength:

```
if RSSI < threshold_far:
    move forward
if RSSI > threshold_close:
    stop
```

Problem:

* No direction.
* Only distance approximation.
* Very noisy.

Not great.

---

## Option 2 (Better): Phone Sends GPS + Heading

Phone calculates:

```
phone_position = (lat, lon)
robot_position = estimate_from_odometry()

vector = phone_position - robot_position
desired_heading = atan2(dy, dx)
```

Robot turns toward heading and moves forward.

---

## Core Control Loop (Pseudo Code)

```
loop every 100ms:

    read encoders
    read IMU
    update position estimate

    if follow_mode:
        target = phone_position
        error_vector = target - current_position

        desired_angle = atan2(error_y, error_x)
        angle_error = desired_angle - current_heading

        left_speed  = base_speed - Kp * angle_error
        right_speed = base_speed + Kp * angle_error

        set_motors(left_speed, right_speed)

    check obstacles
    if obstacle_detected:
        stop or reroute
```
