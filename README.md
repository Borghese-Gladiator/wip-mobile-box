# TODO
- think: name for repo
- develop: mobile app
- develop: marketing page
- develop: robot

# WIP Mobile Box
(Temporary Name and a WIP Project.)

Idea is to have a robot box that follows your phone. This enables luggage w/o carrying things explicitly

## Requirements
- Phone App to manage robot
- Robot
  - robot drives with a differential drive system (two-wheeled for V1)
  - obstacle avoidance
  - can hold items
    - eg: beers as an icebox
    - eg: beach equipment
    - eg: picnic equipment

QUESTIONS
- how well should obstacle avoidance be written? Is it okay if it gets stuck in a local minima?

## Setup
```
[Phone App]
    |
 Bluetooth BLE
    |
[Robot MCU]
    |
Sensors + Motors
```

**Phone App** — [details](app/README.md)
- Map with live GPS (user) and robot position
- "Follow Me" and "Stop" buttons
- Bluetooth connection status indicator
- Robot status icons: battery, motor state, IMU data, position estimate

**Robot** — [details](robot/README.md)
- Bluetooth Low Energy (BLE), sends telemetry: battery, position, IMU, motor state
- Differential drive (tank style): left/right motors, PWM speed control
- Obstacle avoidance




---
```
Excellent. Let’s turn this into an actual build plan you could execute.

I’ll assume:
* Outdoor + pavement use
* Walking speed (~1.4 m/s max)
* 2–3 hour runtime
* UWB-based following
* ESP32 control brain

This is a **concrete V1 → V2 roadmap with real algorithms and pseudocode.**

---

# 🧊 SYSTEM ARCHITECTURE (V1)

**Phone (UWB tag + app)**
⬇
**UWB Anchor (on cooler)**
⬇
**ESP32**
⬇
**Motor Driver**
⬇
**Left + Right Motors**

Optional:

* 2x ultrasonic sensors (front)
* Emergency stop button

---

# PHASE 1 — Build a Driveable Robot (No Following Yet)

## Step 1: Mechanical Assembly

1. Buy a 4-wheel differential drive chassis rated for 50–80 lbs.
2. Mount:

   * 2 geared DC motors (left/right)
   * Motor driver (dual H-bridge)
   * 12V–24V battery
3. Secure cooler to chassis (low center of gravity).

---

## Step 2: Wire Electronics

### Wiring Layout

Battery → Motor Driver
Battery → Buck Converter → ESP32
ESP32 GPIO → Motor Driver inputs

Motor driver typical pins:

* IN1 / IN2 = Left motor
* IN3 / IN4 = Right motor
* PWM pins for speed

---

## Step 3: Basic Motor Control Code

Goal: Make it drive forward, backward, left, right.

### Pseudocode

```pseudo
function set_motor(left_speed, right_speed):
    set_pwm(LEFT_PWM, abs(left_speed))
    set_pwm(RIGHT_PWM, abs(right_speed))

    if left_speed > 0:
        set_pin(IN1, HIGH)
        set_pin(IN2, LOW)
    else:
        set_pin(IN1, LOW)
        set_pin(IN2, HIGH)

    if right_speed > 0:
        set_pin(IN3, HIGH)
        set_pin(IN4, LOW)
    else:
        set_pin(IN3, LOW)
        set_pin(IN4, HIGH)
```

Test:

* set_motor(100, 100) → forward
* set_motor(-100, -100) → backward
* set_motor(100, -100) → rotate

---

# PHASE 2 — Add UWB Tracking

## Step 4: UWB Setup

You need:

* 1 UWB module on cooler (anchor)
* 1 UWB module carried by you (tag)

Modules give:

* Distance (meters)
* Sometimes angle (if using multi-antenna)

If angle not available:
You must compute direction using motion.

---

# FOLLOWING ALGORITHM (CORE LOGIC)

We define:

* `target_distance = 1.5m`
* `deadband = 0.2m`
* `max_speed = 150 PWM`

---

## Simple Distance Control (Proportional Control)

This keeps the cooler 1.5m behind you.

### Pseudocode

```pseudo
loop:
    distance = uwb_get_distance()

    error = distance - target_distance

    if abs(error) < deadband:
        stop()
    else:
        speed = Kp_distance * error
        speed = clamp(speed, -max_speed, max_speed)
        set_motor(speed, speed)
```

Where:

```
Kp_distance ≈ 100
```

This is a proportional controller (P-controller).

---

# DIRECTION CONTROL (TURNING TOWARD USER)

## If UWB Gives Angle

Assume:

* angle = 0° means straight ahead
* positive = target to right
* negative = target to left

### Combined Control

```pseudo
loop:
    distance = uwb_get_distance()
    angle = uwb_get_angle()

    distance_error = distance - target_distance
    angle_error = angle

    forward_speed = Kp_distance * distance_error
    turn_speed = Kp_angle * angle_error

    left_speed = forward_speed - turn_speed
    right_speed = forward_speed + turn_speed

    left_speed = clamp(left_speed, -max_speed, max_speed)
    right_speed = clamp(right_speed, -max_speed, max_speed)

    set_motor(left_speed, right_speed)
```

Typical values:

```
Kp_distance = 80
Kp_angle = 3–5
```

---

# If UWB Does NOT Provide Angle

You must estimate direction.

## Direction Estimation Algorithm

1. Measure distance.
2. Rotate slightly left.
3. Measure distance.
4. If distance decreases → target is left.
5. Else → target is right.

### Pseudocode

```pseudo
function find_direction():
    d1 = uwb_get_distance()

    rotate_left_small()
    wait(200ms)

    d2 = uwb_get_distance()

    if d2 < d1:
        return LEFT
    else:
        return RIGHT
```

Then steer in that direction until distance decreases.

This is slower but works.

---

# PHASE 3 — Add Obstacle Avoidance

Add 2 ultrasonic sensors (left + right front).

## Obstacle Logic

```pseudo
front_left = ultrasonic_left()
front_right = ultrasonic_right()

if front_left < 0.5m OR front_right < 0.5m:
    stop()
    rotate_away_from_closest_obstacle()
```

More advanced version blends obstacle force with follow force:

```pseudo
avoidance = K_avoid / obstacle_distance
turn_speed += avoidance
```

---

# PHASE 4 — BLE App Control

## App Commands

Define protocol:

```
CMD_START
CMD_STOP
CMD_SET_DISTANCE:1.5
CMD_MAX_SPEED:120
```

---

### ESP32 BLE Loop

```pseudo
if ble_received("CMD_START"):
    follow_enabled = true

if ble_received("CMD_STOP"):
    follow_enabled = false
    stop()

if follow_enabled:
    run_follow_algorithm()
```

---

# PHASE 5 — Safety Logic (Critical)

## Signal Loss Protection

```pseudo
if uwb_signal_lost_for > 1 second:
    stop()
```

## Speed Ramp (Prevent Jerky Motion)

```pseudo
current_speed += (target_speed - current_speed) * 0.1
```

This smooths acceleration.

---

# CONTROL ARCHITECTURE SUMMARY

Main Loop:

```pseudo
loop:
    read_ble()
    read_uwb()
    read_obstacles()

    if emergency_pressed:
        stop()
        continue

    if follow_enabled:
        compute_follow_control()
        compute_obstacle_adjustment()
        apply_motor_output()
    else:
        stop()
```

---

# PHASE 6 — Tuning Process

1. Tune distance Kp first.
2. Then tune angle Kp.
3. Then tune obstacle gain.
4. Then tune acceleration smoothing.

Test with:

* No load
* Half load
* Full beer load

---

# REALISTIC BUILD TIMELINE

Week 1:

* Assemble chassis
* Basic motor control

Week 2:

* UWB distance reading

Week 3:

* Basic follow logic

Week 4:

* App + BLE control

Week 5:

* Obstacle avoidance

Week 6:

* Testing + tuning

---

# MVP CHECKLIST

✔ Drives manually
✔ Maintains 1.5m distance
✔ Turns toward you
✔ Stops if too close
✔ Stops if signal lost
✔ Avoids obstacles
✔ App start/stop

---

If you'd like, I can next:

* Write actual ESP32-style C++ code
* Draw a block diagram with real signal flow
* Design a V2 with full PID control
* Or design a “commercializable” version

How serious is this project — fun hack or something you’d actually invest $1–2k into?


---
Excellent. This is exactly how real robotics teams work.

You want a **software-in-the-loop (SIL) simulation** so you can:

* Validate the follow algorithm
* Tune gains (Kp values)
* Test edge cases (signal loss, obstacles)
* Confirm stability
* Avoid smashing a $1,000 beer robot into a wall

Below is a concrete, realistic way to do this.

---

# ✅ Best Option: Python 2D Physics Simulation

We simulate:

* You (target)
* Icebox robot
* Distance sensor (simulated UWB)
* Optional angle
* Obstacles
* Noise
* Latency

Then run your exact control algorithm against it.

---

# 🧠 System Model

We model:

### State Variables

Robot:

```
x, y
heading (theta)
left_speed
right_speed
```

User:

```
x, y
```

---

# 📐 Robot Motion Model (Differential Drive)

Standard equations:

```
v = (left + right) / 2
omega = (right - left) / wheel_base
```

Update:

```
x += v * cos(theta) * dt
y += v * sin(theta) * dt
theta += omega * dt
```

This mirrors real robot physics.

---

# 🧪 Step 1 — Minimal Simulation Framework (Python)

Install:

```
pip install pygame numpy
```

---

## Basic Simulation Code (Concrete)

This simulates following with proportional control.

```python
import pygame
import numpy as np
import math
import random

# --- Constants ---
WIDTH, HEIGHT = 800, 600
DT = 0.05
WHEEL_BASE = 40
TARGET_DISTANCE = 80  # pixels
KP_DISTANCE = 1.5
KP_ANGLE = 2.0
MAX_SPEED = 150

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# Robot state
robot_x, robot_y = 200, 300
robot_theta = 0

# User state
user_x, user_y = 500, 300

def clamp(val, minv, maxv):
    return max(min(val, maxv), minv)

running = True
while running:
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Move user with arrow keys
    keys = pygame.key.get_pressed()
    if keys[pygame.K_UP]: user_y -= 3
    if keys[pygame.K_DOWN]: user_y += 3
    if keys[pygame.K_LEFT]: user_x -= 3
    if keys[pygame.K_RIGHT]: user_x += 3

    # --- Simulated UWB ---
    dx = user_x - robot_x
    dy = user_y - robot_y

    distance = math.sqrt(dx**2 + dy**2)
    angle_to_user = math.atan2(dy, dx)

    angle_error = angle_to_user - robot_theta
    angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

    # Add simulated noise
    distance += random.uniform(-2, 2)
    angle_error += random.uniform(-0.02, 0.02)

    # --- Control Algorithm ---
    distance_error = distance - TARGET_DISTANCE

    forward_speed = KP_DISTANCE * distance_error
    turn_speed = KP_ANGLE * angle_error

    left_speed = forward_speed - turn_speed
    right_speed = forward_speed + turn_speed

    left_speed = clamp(left_speed, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed, -MAX_SPEED, MAX_SPEED)

    # --- Physics Update ---
    v = (left_speed + right_speed) / 2
    omega = (right_speed - left_speed) / WHEEL_BASE

    robot_x += v * math.cos(robot_theta) * DT
    robot_y += v * math.sin(robot_theta) * DT
    robot_theta += omega * DT

    # --- Draw ---
    screen.fill((30, 30, 30))

    pygame.draw.circle(screen, (0, 255, 0), (int(user_x), int(user_y)), 10)
    pygame.draw.circle(screen, (0, 0, 255), (int(robot_x), int(robot_y)), 15)

    pygame.display.flip()

pygame.quit()
```

---

# 🧪 What This Lets You Test

✔ Does it oscillate?
✔ Does it overshoot?
✔ Does it spiral?
✔ Does it lose control at high speed?
✔ Is KP_DISTANCE too high?

You tune in simulation until it behaves cleanly.

---

# 🧠 Add Realistic Failure Modes

To make it useful:

## 1️⃣ Add Signal Drop

```python
if random.random() < 0.01:
    distance = None
```

Then test your stop logic.

---

## 2️⃣ Add Latency

Simulate UWB delay:

```python
measurement_queue.append((distance, angle_error))
if len(measurement_queue) > 5:
    distance, angle_error = measurement_queue.pop(0)
```

This simulates 250ms delay.

---

## 3️⃣ Add Obstacles

Define circular obstacles and simulate ultrasonic distance:

```python
obstacles = [(400, 300, 40)]

for ox, oy, r in obstacles:
    pygame.draw.circle(screen, (255, 0, 0), (ox, oy), r)
```

Then add avoidance force.

---

# 🔥 Even Better: Test Arduino Code Directly

Instead of rewriting control logic twice:

### Option A (Best Practice)

Write control logic as:

```
control.py
```

Then write the same logic in C++.

Or…

---

### Option B (Professional Method)

Write control logic in pure C (no Arduino functions).

Then:

* Compile for PC (for simulation)
* Compile for Arduino

This is how real robotics stacks work.

---

# 🧪 Advanced Option — Use a Real Robotics Simulator

If you want professional-grade testing:

Use:

* Gazebo
* Webots
* ROS

But that’s much heavier.

For your project, Python 2D simulation is perfect.

---

# 🎯 What You Should Validate Before Uploading to Arduino

1. Stable distance hold
2. No oscillation
3. No spin-in-place bug
4. Smooth acceleration
5. Signal-loss stop
6. Obstacle avoidance does not conflict with follow logic
7. Gain values stable under noise + latency

---

# 🧊 Final Recommendation

Build this progression:

1. Python sim
2. Hardware with wheels lifted off ground
3. Low-speed real-world test
4. Gradually increase speed

---

If you want next, I can:

* Upgrade this to full PID simulation
* Add obstacle force blending math
* Design a tuning workflow
* Or simulate a full “park party” scenario

How realistic do you want this demo to feel — quick validation or near-real physics?

(https://chatgpt.com/c/6993727c-17d8-8326-8480-e9a3c99d3012)
```
