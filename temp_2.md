Excellent. Now we’re designing this like a real robotics system instead of a demo toy.

I’ll give you:

1. ✅ A fully modular class-based simulation (same animation, cleaner architecture)
2. 🧠 A clear system architecture explanation
3. 📱 What the phone should truly handle
4. 🤖 What the robot must handle locally
5. 🚗 Movement model recommendations
6. 🛞 Tire + drivetrain design recommendations
7. ⚙ Expanded “bare minimum” hardware list

---

# 🧠 SYSTEM ARCHITECTURE (REAL-WORLD DESIGN)

We are building a **hierarchical control system**.

There are two brains:

```
Phone  →  High-level autonomy
Robot  →  Real-time safety + motion control
```

This is important.

The phone is powerful but unreliable (latency, dropouts).
The robot must be self-stabilizing and safe even if the phone dies.

---

# 📱 PHONE RESPONSIBILITIES (High-Level Control)

The phone should handle:

### 1️⃣ Localization (Global Awareness)

* GPS processing
* Optional map services
* Geofencing
* Logging

### 2️⃣ Compute Relative Vector

Phone knows:

* Its own GPS position
* Robot’s estimated GPS position (from robot telemetry)

It computes:

```
relative_vector = phone_pos - robot_pos
```

### 3️⃣ High-Level Path Planning

* Choose target speed
* Avoid large obstacles if map-aware
* Smooth velocity commands
* Maintain follow distance

### 4️⃣ Send Commands (Not Wheel Speeds)

Send to robot:

```
target_linear_velocity  (m/s)
target_angular_velocity (rad/s)
```

NOT raw wheel speeds.

The robot should decide how to achieve those velocities.

---

# 🤖 ROBOT RESPONSIBILITIES (Low-Level Autonomy)

Robot must be safe and autonomous locally.

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

---

# 🚗 WHAT TYPE OF MOVEMENT SHOULD THE ROBOT DO?

You are using tank-style differential drive.

That’s correct for this project.

Movement model:

```
v = linear velocity
ω = angular velocity

v_l = v - (ω * wheel_base / 2)
v_r = v + (ω * wheel_base / 2)
```

This gives:

* Forward/backward
* Spin in place
* Smooth curves

This is exactly how:

* iRobot robots operate
* NASA rovers operate
* Most warehouse robots operate

---

# 🛞 WHAT KIND OF TIRES?

For your use case (sidewalk + curb capability):

### Best Choice: Large Rubber Wheels (NOT tank treads)

Why?

Tank treads:

* Heavy
* Inefficient
* Complex tension system
* Hard to maintain

Instead:

### Recommended:

* 8–10 inch pneumatic tires
* Rubber, air-filled
* Deep tread

Benefits:

* Absorb shock
* Handle cracks
* Climb small curbs
* Less slip
* Lower energy use

Add:

* Slightly wide wheel base for stability

---

# ⚙ Expanded Bare Minimum Hardware List

### 🔹 Compute

* ESP32 (v1)
* OR Raspberry Pi 5 + motor MCU (v2)

### 🔹 Motors

* 2x 12V 100–200 RPM gear motors
* Metal gearbox
* Integrated encoder

### 🔹 Motor Driver

* TB6612FNG (light robot)
* OR Cytron MD10C (more power)

### 🔹 Sensors

Minimum:

* 2x wheel encoders
* IMU (BNO055 recommended)
* 2–4 ultrasonic sensors front-facing

Upgrade path:

* RPLidar A1
* UWB localization

### 🔹 Power

* 3S or 4S Li-ion pack
* Separate logic regulator
* Fuse + cutoff switch

### 🔹 Safety

* Emergency stop button
* Watchdog timer

---

# 🧱 Now: Modular Simulation (Class-Based)

Below is the same simulation — but cleanly modular.

You’ll see:

* Phone class
* Robot class
* MotorModel
* ObstacleField
* World
* Cleaner animation

---

# 📦 Full Modular Simulation Code

Save as:

`mobile_box_modular_sim.py`

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# =============================
# Utility
# =============================

def wrap_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi


# =============================
# Phone
# =============================

class Phone:
    def __init__(self):
        self.time = 0

    def true_position(self):
        r = 4
        speed = 0.3
        angle = speed * self.time
        return np.array([r*np.cos(angle), r*np.sin(angle)])

    def update(self, dt):
        self.time += dt

    def compute_command(self, robot_pose):
        phone_pos = self.true_position()
        error = phone_pos - robot_pose[:2]

        dist = np.linalg.norm(error)
        desired_angle = np.arctan2(error[1], error[0])
        angle_error = wrap_angle(desired_angle - robot_pose[2])

        target_v = 1.2 * dist
        target_w = 3.0 * angle_error

        return target_v, target_w


# =============================
# Robot
# =============================

class Robot:
    def __init__(self):
        self.x = -5
        self.y = 0
        self.theta = 0

        self.v = 0
        self.w = 0

        self.v_cmd = 0
        self.w_cmd = 0

        self.last_command_time = 0

        self.wheel_base = 0.5
        self.max_accel = 3
        self.max_speed = 2

    def receive_command(self, v, w, current_time):
        self.v_cmd = v
        self.w_cmd = w
        self.last_command_time = current_time

    def safety_check(self, current_time):
        if current_time - self.last_command_time > 0.5:
            self.v_cmd = 0
            self.w_cmd = 0

    def apply_limits(self, dt):
        dv = np.clip(self.v_cmd - self.v,
                     -self.max_accel*dt,
                     self.max_accel*dt)

        dw = np.clip(self.w_cmd - self.w,
                     -5*dt,
                     5*dt)

        self.v += dv
        self.w += dw

        self.v = np.clip(self.v, -self.max_speed, self.max_speed)

    def update(self, dt):
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt
        self.theta += self.w * dt
        self.theta = wrap_angle(self.theta)

    def pose(self):
        return np.array([self.x, self.y, self.theta])


# =============================
# Obstacle Field
# =============================

class ObstacleField:
    def __init__(self):
        self.obstacles = [
            np.array([2, 1]),
            np.array([-3, -2]),
            np.array([0, 4])
        ]
        self.radius = 1.0

    def avoidance(self, robot):
        force = 0

        for obs in self.obstacles:
            vec = np.array([robot.x, robot.y]) - obs
            dist = np.linalg.norm(vec)

            if dist < 2*self.radius:
                angle = np.arctan2(vec[1], vec[0])
                angle_error = wrap_angle(angle - robot.theta)
                force += 2 * angle_error

        return force


# =============================
# World
# =============================

class World:
    def __init__(self):
        self.phone = Phone()
        self.robot = Robot()
        self.obstacles = ObstacleField()
        self.time = 0
        self.dt = 0.05

        self.trail = []

    def step(self):
        self.time += self.dt
        self.phone.update(self.dt)

        # Phone computes command
        v_cmd, w_cmd = self.phone.compute_command(self.robot.pose())

        # Robot receives command
        self.robot.receive_command(v_cmd, w_cmd, self.time)

        # Obstacle override
        avoidance_w = self.obstacles.avoidance(self.robot)
        self.robot.w_cmd += avoidance_w

        # Safety + limits
        self.robot.safety_check(self.time)
        self.robot.apply_limits(self.dt)

        # Physics
        self.robot.update(self.dt)

        self.trail.append((self.robot.x, self.robot.y))
        if len(self.trail) > 200:
            self.trail.pop(0)


# =============================
# Visualization
# =============================

plt.style.use("dark_background")
fig, ax = plt.subplots(figsize=(8,8))
ax.set_xlim(-10,10)
ax.set_ylim(-10,10)
ax.set_aspect("equal")

world = World()

robot_body = plt.Circle((0,0), 0.4, color='orange')
ax.add_patch(robot_body)

heading_line, = ax.plot([], [], linewidth=3)
phone_dot, = ax.plot([], [], marker='o', markersize=8)

for obs in world.obstacles.obstacles:
    ax.add_patch(plt.Circle(obs, world.obstacles.radius,
                            color='red', alpha=0.3))

def animate(frame):
    world.step()

    robot_body.center = (world.robot.x, world.robot.y)

    heading_line.set_data(
        [world.robot.x,
         world.robot.x + np.cos(world.robot.theta)],
        [world.robot.y,
         world.robot.y + np.sin(world.robot.theta)]
    )

    phone_pos = world.phone.true_position()
    phone_dot.set_data(phone_pos[0], phone_pos[1])

    ax.collections.clear()
    for i, point in enumerate(world.trail):
        alpha = i / len(world.trail)
        ax.scatter(point[0], point[1], s=5, alpha=alpha)

    return robot_body, heading_line, phone_dot

ani = FuncAnimation(fig, animate, interval=50)
plt.show()
```

---

# 🔥 Why This Architecture Is Correct

* Phone handles global logic
* Robot handles safety and physics
* Robot can survive Bluetooth failure
* Robot overrides phone for obstacles
* Robot limits acceleration
* Control structure mirrors real robotics systems

This is no longer a toy simulation.

---

If you'd like, next we can:

* Add a Kalman filter
* Add realistic GPS drift
* Add wheel encoder simulation
* Add real PID motor loop
* Or convert this into a ROS-like architecture

Where do you want to take Mobile Box next?
