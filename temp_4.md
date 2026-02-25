

----
Great idea. Let’s turn **Mobile Box** into something structured and buildable.

---

# 📄 Product Requirements Document (PRD)

**Project Name:** Mobile Box
**Version:** v1.0
**Owner:** Tim

## 1. Product Overview

Mobile Box is a small autonomous robot that:

* Connects to a smartphone via Bluetooth
* Displays both phone and robot location on a map
* Follows the phone when “Follow Me” is pressed
* Uses tank-style differential drive for movement

Primary use case: a robot that follows the user and carries items.

---

## 2. Functional Requirements

## 2.1 Mobile App

### Map Screen

* Display map (Google Maps / Apple Maps)
* Show:

  * Phone position (live GPS)
  * Robot position (live estimate)
* Auto-refresh at 1 Hz minimum

### Controls

* Button: **“Follow Me”**

  * Sends follow command to robot
* Button: **“Stop”**

  * Immediately halts movement
* Bluetooth connection status indicator

---

## 2.2 Robot

### Connectivity

* Bluetooth Low Energy (BLE)
* Reconnect automatically
* Send telemetry:

  * Battery level
  * Position estimate
  * IMU data
  * Motor state

### Movement

* Differential drive (tank style)

  * Left motor
  * Right motor
* Speed control via PWM
* Turning via differential velocity

### Sensors

Minimum viable sensor set:

| Sensor              | Purpose                       |
| ------------------- | ----------------------------- |
| Wheel encoders      | Odometry (distance + heading) |
| IMU (gyro + accel)  | Orientation + smoothing       |
| Ultrasonic or LiDAR | Obstacle detection            |
| BLE RSSI            | Rough distance to phone       |

---

## 3. Non-Functional Requirements

* Max speed: 1.5 m/s
* Follow distance: 1–2 meters
* Latency: <200ms control loop
* Battery life: 1–2 hours
* Outdoor and indoor capable

---

## 4. System Architecture

### High-Level

```
[Phone App]
    |
 Bluetooth BLE
    |
[Robot MCU]
    |
Sensors + Motors
```

Phone does:

* UI
* GPS tracking
* High-level control

Robot does:

* Motion control
* Sensor fusion
* Obstacle avoidance
* Low-level autonomy

---

# 🔧 Implementation Plan

Now let’s make it real.

---

# 🧠 Hardware Selection

## Robot Brain

**Option A (Recommended):**

* Raspberry Pi 5 (high-level logic)
* Arduino Nano (motor control)

OR

**Option B (Simpler):**

* ESP32 (does BLE + motor control)

  * Cheap
  * Integrated Bluetooth
  * Real-time capable

I recommend **ESP32** for v1.

---

## Motors

* 2x 12V DC gear motors
* Tank tread or two large wheels
* Motor driver:

  * L298N (cheap)
  * Or better: TB6612FNG

---

## Sensors

### 1. Wheel Encoders

* Hall effect encoders or optical encoders
* Needed for odometry

### 2. IMU

* MPU6050 (cheap)
* Or BNO055 (better — does onboard fusion)

### 3. Obstacle Detection

* HC-SR04 ultrasonic (cheap)
* OR
* RPLidar A1 (real robot-level)

---

# 📱 Mobile App Stack

## Option A: Flutter (Cross Platform)

* Google Maps SDK
* BLE plugin

## Option B: React Native

## Option C: Native Swift / Kotlin

I’d recommend:

> Flutter + BLE + Google Maps SDK

---

# 🧠 Core Algorithms

Now the interesting part.

---

# 📍 How Does the Robot Know Its Position?

### 1. Outdoor:

Use phone GPS as truth.
Robot estimates relative position.

### 2. Indoor:

Use:

* Wheel odometry
* IMU
* BLE signal strength
* Possibly UWB for precision

---

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

---

# 🗺️ Map Implementation

Phone:

* Displays:

  * Blue dot = phone GPS
  * Red dot = robot position (from robot telemetry)
* Robot sends position updates every 500ms via BLE

---

# ❓ Your Z-Position Question

You asked:

> How does a Roomba do Z position?

Great question.

### Short answer:

It doesn’t track absolute Z like GPS.

### Example: iRobot Roomba

Modern Roombas use:

1. Wheel encoders
2. IMU
3. Optical flow sensors (looking at floor)
4. SLAM (Simultaneous Localization and Mapping)
5. Camera (in higher-end models)

They estimate position relative to a starting point.

No GPS.
No global coordinates.
Pure sensor fusion.

---

# 🧠 What is SLAM?

SLAM = build a map while tracking your location inside it.

Uses:

* LiDAR or camera
* IMU
* Wheel encoders

If you want Mobile Box v2:
→ Add LiDAR
→ Run SLAM on Raspberry Pi
→ Use ROS2

---

# 🚀 Phase-Based Development Plan

---

## Phase 1 — Remote Control Only

* BLE connection
* Manual joystick control
* Motor driver working

Goal: Robot moves reliably.

---

## Phase 2 — Odometry

* Add encoders
* Track distance traveled
* Display robot position relative to start

Goal: Position estimate on map.

---

## Phase 3 — Follow Me (Outdoor GPS)

* Phone sends GPS
* Robot moves toward GPS target
* Add basic obstacle avoidance

Goal: Works in open areas.

---

## Phase 4 — Indoor Following

* Add LiDAR
* Implement SLAM
* Add obstacle navigation

Goal: Works in house.

---

# ⚡ If You Want “Real Robotics Level”

Use:

* Raspberry Pi 5
* RPLidar
* ROS2
* Navigation stack

That puts you in Roomba-tier architecture.

---

# 🧠 Hard Problems You’ll Hit

1. BLE latency
2. RSSI instability
3. Wheel slip
4. GPS drift
5. Indoor localization
6. Power management
7. Obstacle avoidance edge cases

---

# 🔥 If I Were Building This

I’d do:

ESP32 + encoders + IMU + ultrasonic
Flutter app
Outdoor GPS following first
Then upgrade to SLAM later

---

If you want, I can next:

* Design full hardware wiring diagram
* Write ESP32 firmware skeleton
* Write Flutter BLE + map starter code
* Or design the full SLAM-based version

Where do you want to go next?


---
Perfect. This is exactly how real robotics teams do it: **simulate first, break it in software, then touch hardware**.

We’ll build:

* A physics simulation
* Differential drive robot
* Phone moving target
* Follow algorithm
* Obstacle avoidance
* Real-time visualization

All in pure Python.

---

# 🧠 What We’ll Simulate

* 2D world (meters)
* Robot with:

  * (x, y)
  * heading θ
  * left/right wheel speeds
* Phone moving along path
* Follow controller
* Optional obstacle
* Real-time animation

---

# 🧰 Stack

* Python 3
* numpy
* matplotlib (animation)
* no external physics engine required

---

# 🚗 Differential Drive Physics

Robot state:

```
x, y        # position
theta       # heading (radians)
v_l, v_r    # wheel velocities
```

Physics:

```
v = (v_r + v_l) / 2
omega = (v_r - v_l) / wheel_base
```

Update:

```
x += v * cos(theta) * dt
y += v * sin(theta) * dt
theta += omega * dt
```

---

# 🎯 Follow Algorithm

```
error_vector = phone_pos - robot_pos
desired_heading = atan2(dy, dx)
angle_error = wrap(desired_heading - theta)

v_l = base_speed - Kp * angle_error
v_r = base_speed + Kp * angle_error
```

If close → stop.

---

# 🧪 Full Working Simulation Code

Copy this into a file: `mobile_box_sim.py`

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# =============================
# Simulation Parameters
# =============================
dt = 0.05
wheel_base = 0.4
base_speed = 1.0
Kp = 2.0
follow_distance = 0.8

world_size = 10

# =============================
# Robot State
# =============================
robot = {
    "x": 0.0,
    "y": 0.0,
    "theta": 0.0,
    "v_l": 0.0,
    "v_r": 0.0
}

# =============================
# Phone Path (Target)
# =============================
def phone_position(t):
    # Circular motion
    radius = 3
    speed = 0.4
    angle = speed * t
    return np.array([radius * np.cos(angle),
                     radius * np.sin(angle)])

# =============================
# Helper Functions
# =============================
def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def update_robot():
    v_l = robot["v_l"]
    v_r = robot["v_r"]

    v = (v_r + v_l) / 2
    omega = (v_r - v_l) / wheel_base

    robot["x"] += v * np.cos(robot["theta"]) * dt
    robot["y"] += v * np.sin(robot["theta"]) * dt
    robot["theta"] += omega * dt

# =============================
# Control Logic
# =============================
def follow_controller(target):
    dx = target[0] - robot["x"]
    dy = target[1] - robot["y"]

    distance = np.sqrt(dx**2 + dy**2)
    desired_heading = np.arctan2(dy, dx)
    angle_error = wrap_angle(desired_heading - robot["theta"])

    if distance < follow_distance:
        robot["v_l"] = 0
        robot["v_r"] = 0
        return

    robot["v_l"] = base_speed - Kp * angle_error
    robot["v_r"] = base_speed + Kp * angle_error

# =============================
# Visualization Setup
# =============================
fig, ax = plt.subplots()
ax.set_xlim(-world_size, world_size)
ax.set_ylim(-world_size, world_size)

robot_dot, = ax.plot([], [], 'ro')
phone_dot, = ax.plot([], [], 'bo')
heading_line, = ax.plot([], [], 'r-')

trail_x = []
trail_y = []

time = 0

# =============================
# Animation Loop
# =============================
def animate(frame):
    global time
    time += dt

    target = phone_position(time)

    follow_controller(target)
    update_robot()

    trail_x.append(robot["x"])
    trail_y.append(robot["y"])

    # Update visuals
    robot_dot.set_data(robot["x"], robot["y"])
    phone_dot.set_data(target[0], target[1])
    heading_line.set_data(
        [robot["x"], robot["x"] + np.cos(robot["theta"])],
        [robot["y"], robot["y"] + np.sin(robot["theta"])]
    )

    ax.plot(trail_x, trail_y, 'r--', linewidth=0.5)

    return robot_dot, phone_dot, heading_line

ani = FuncAnimation(fig, animate, interval=dt*1000)
plt.title("Mobile Box Follow Simulation")
plt.show()
```

---

# 🧪 What You’ll See

* Blue dot = Phone
* Red dot = Robot
* Red arrow = heading
* Dashed trail = robot path

Robot will:

* Turn toward phone
* Follow circular path
* Maintain distance

---

# 🔥 Want To Make It More Real?

Add noise:

Replace:

```python
desired_heading = np.arctan2(dy, dx)
```

With:

```python
noise = np.random.normal(0, 0.02)
desired_heading = np.arctan2(dy, dx) + noise
```

Now it behaves like real sensors.

---

# 🧠 Add Obstacle Simulation

Add this inside controller:

```python
obstacle = np.array([1.5, 1.5])
obs_dist = np.linalg.norm(
    obstacle - np.array([robot["x"], robot["y"]])
)

if obs_dist < 1.0:
    robot["v_l"] = -0.5
    robot["v_r"] = 0.5
    return
```

Now it avoids a fixed obstacle.

---

# 🚀 Next Level (If You Want)

We can extend this into:

* SLAM-like mapping simulation
* Kalman filter for sensor fusion
* BLE latency simulation
* Full ROS-style architecture
* Web-based UI dashboard with live map

---

# 🧠 Why This Is Powerful

Once this simulation behaves correctly:

* You already validated:

  * Controller math
  * Physics model
  * Follow logic
* Hardware becomes “just sensors + motors”

You’ve eliminated 70% of risk.

---

If you want, next we can:

* Turn this into a real UI dashboard with a clickable “Follow Me” button
* Add virtual BLE latency
* Convert this into a browser-based simulation
* Add real path planning (A*)

What direction do you want to evolve it?


----
run gpt-oss-20b on my server for some passive data processing. It handles things like data extraction and summarization throughout the day, and the same server powers my home voice assistant. When I ask it to turn off a light or check tomorrow's weather, the GPU wakes up, processes the request through the LLM, and goes right back to sleep. 



- washing machine gasket - wipe with paper towel
- rice puff 
- ask Dad to bring cutter for security lock
  - tell SW how it goes 
- clean up glove box

We realized that we no longer needed such complex orchestration to build agents, because models now have native tool-use and reasoning capabilities.” What used to take months to make production-ready now takes days with a model-driven approach. Today, with Claude Sonnet 4.6, Gemini 3 Pro, and Llama 4 Scout with its 10-million-token context window, models handle planning, tool selection, and self-correction on their own. The need for heavy framework orchestration shrinks with every model generation.
- context engineering - Google adk
- evaluation and observability 
- agent security 
- Interoperability (protocols)

Interoperability protocols might be the most important shift of all. MCP is becoming the universal standard for tool integration. A2A handles agent-to-agent communication. Google’s ADK and Amazon’s Strands ship with support for both. If MCP and A2A harden into industry standards, the framework above them becomes a thinner commodity layer.

First, bet on protocols, not frameworks. Design your tool integrations around MCP from day one. Adopt A2A for inter-agent communication. This is your hedge against framework churn, and there will be churn.

Second, invest in evaluation and observability independently of your framework choice. This layer outlasts any framework. Pick tooling that works across frameworks, not tooling that locks you into one.

Third, if you are already deep in a hyperscaler, use their SDK. The co-design between Strands and Bedrock, or ADK and Vertex, is a real engineering advantage. Do not fight that gravity. It is not worth the friction.

https://thenewstack.io/agent-framework-container-wars/
(Smart dude)

ガラケー