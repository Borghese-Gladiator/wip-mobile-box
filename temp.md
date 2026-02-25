SIMULATION for Mobile Box


Love this. Now we move from “toy demo” → “this feels like a robotics lab.”

We’re going to simulate:

✅ Differential drive physics
✅ Wheel slip
✅ IMU noise
✅ GPS noise
✅ BLE latency
✅ Packet drop
✅ Obstacle avoidance (repulsive field)
✅ Acceleration limits (real motors)
✅ Pretty animation (clean UI, robot body, heading arrow, trail fade)

---

# What This Version Adds

### Real-world issues modeled

| Problem       | How we simulate it       |
| ------------- | ------------------------ |
| Wheel slip    | Random velocity scaling  |
| IMU noise     | Heading noise            |
| GPS drift     | Noisy phone position     |
| BLE latency   | Delayed command buffer   |
| Packet drop   | Random message loss      |
| Motor inertia | Acceleration limits      |
| Obstacles     | Repulsive force field    |
| Sensor noise  | Gaussian noise injection |

---

# 🧪 Full Advanced Simulation

Save as:

`mobile_box_realistic_sim.py`

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# ======================================
# WORLD PARAMETERS
# ======================================
dt = 0.05
world_size = 10
wheel_base = 0.5
max_speed = 2.0
max_accel = 3.0
follow_distance = 1.0

# Control gains
Kp_angle = 3.0
Kp_dist = 1.2

# Noise parameters
imu_noise_std = 0.02
gps_noise_std = 0.1
wheel_slip_std = 0.1

# BLE simulation
latency_seconds = 0.4
packet_drop_rate = 0.1
latency_buffer = deque(maxlen=int(latency_seconds/dt))

# Obstacles
obstacles = [
    np.array([2.5, 1.5]),
    np.array([-3.0, -2.0]),
    np.array([0.0, 4.0])
]
obstacle_radius = 1.0
repulsion_strength = 3.0

# ======================================
# ROBOT STATE
# ======================================
robot = {
    "x": -5.0,
    "y": 0.0,
    "theta": 0.0,
    "v_l": 0.0,
    "v_r": 0.0,
    "v_l_cmd": 0.0,
    "v_r_cmd": 0.0
}

# Trail
trail = []

# ======================================
# PHONE PATH
# ======================================
def phone_position(t):
    r = 4
    speed = 0.3
    angle = speed * t
    return np.array([r*np.cos(angle), r*np.sin(angle)])

# ======================================
# HELPERS
# ======================================
def wrap_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def add_noise(value, std):
    return value + np.random.normal(0, std)

# ======================================
# SENSOR MODELS
# ======================================
def get_noisy_phone_position(true_pos):
    return add_noise(true_pos, gps_noise_std)

def get_noisy_heading(true_theta):
    return add_noise(true_theta, imu_noise_std)

# ======================================
# MOTOR MODEL (Acceleration Limits)
# ======================================
def apply_motor_physics():
    for side in ["v_l", "v_r"]:
        cmd = robot[f"{side}_cmd"]
        current = robot[side]
        diff = cmd - current

        max_delta = max_accel * dt
        diff = np.clip(diff, -max_delta, max_delta)

        robot[side] += diff
        robot[side] = np.clip(robot[side], -max_speed, max_speed)

# ======================================
# PHYSICS UPDATE
# ======================================
def update_robot():
    slip_l = np.random.normal(1.0, wheel_slip_std)
    slip_r = np.random.normal(1.0, wheel_slip_std)

    v_l = robot["v_l"] * slip_l
    v_r = robot["v_r"] * slip_r

    v = (v_l + v_r) / 2
    omega = (v_r - v_l) / wheel_base

    robot["x"] += v * np.cos(robot["theta"]) * dt
    robot["y"] += v * np.sin(robot["theta"]) * dt
    robot["theta"] += omega * dt
    robot["theta"] = wrap_angle(robot["theta"])

# ======================================
# OBSTACLE AVOIDANCE (Potential Field)
# ======================================
def obstacle_repulsion():
    force = np.array([0.0, 0.0])
    pos = np.array([robot["x"], robot["y"]])

    for obs in obstacles:
        vec = pos - obs
        dist = np.linalg.norm(vec)

        if dist < obstacle_radius * 2:
            direction = vec / (dist + 1e-5)
            strength = repulsion_strength / (dist**2 + 0.1)
            force += direction * strength

    return force

# ======================================
# FOLLOW CONTROLLER
# ======================================
def follow_controller(phone_pos_noisy):
    pos = np.array([robot["x"], robot["y"]])
    heading = get_noisy_heading(robot["theta"])

    error_vec = phone_pos_noisy - pos
    distance = np.linalg.norm(error_vec)

    if distance < follow_distance:
        return 0, 0

    desired_angle = np.arctan2(error_vec[1], error_vec[0])
    angle_error = wrap_angle(desired_angle - heading)

    forward_speed = Kp_dist * distance
    turn = Kp_angle * angle_error

    v_l = forward_speed - turn
    v_r = forward_speed + turn

    return v_l, v_r

# ======================================
# ANIMATION SETUP (Pretty)
# ======================================
plt.style.use("dark_background")
fig, ax = plt.subplots(figsize=(8,8))
ax.set_xlim(-world_size, world_size)
ax.set_ylim(-world_size, world_size)
ax.set_aspect('equal')
ax.set_title("Mobile Box Realistic Simulation", fontsize=16)

robot_body = plt.Circle((0,0), 0.4, color='orange')
ax.add_patch(robot_body)

heading_line, = ax.plot([], [], linewidth=3)
phone_dot, = ax.plot([], [], marker='o', markersize=8)

# Draw obstacles
for obs in obstacles:
    circle = plt.Circle(obs, obstacle_radius, color='red', alpha=0.3)
    ax.add_patch(circle)

time = 0

# ======================================
# MAIN LOOP
# ======================================
def animate(frame):
    global time
    time += dt

    true_phone = phone_position(time)
    noisy_phone = get_noisy_phone_position(true_phone)

    # Simulate BLE packet drop
    if np.random.rand() > packet_drop_rate:
        latency_buffer.append(noisy_phone)
    else:
        latency_buffer.append(None)

    if latency_buffer and latency_buffer[0] is not None:
        delayed_phone = latency_buffer[0]
    else:
        delayed_phone = noisy_phone

    v_l_cmd, v_r_cmd = follow_controller(delayed_phone)

    # Add obstacle avoidance
    repulse = obstacle_repulsion()
    if np.linalg.norm(repulse) > 0:
        desired_angle = np.arctan2(repulse[1], repulse[0])
        angle_error = wrap_angle(desired_angle - robot["theta"])
        v_l_cmd -= 2 * angle_error
        v_r_cmd += 2 * angle_error

    robot["v_l_cmd"] = v_l_cmd
    robot["v_r_cmd"] = v_r_cmd

    apply_motor_physics()
    update_robot()

    trail.append((robot["x"], robot["y"]))
    if len(trail) > 200:
        trail.pop(0)

    ax.collections.clear()

    # Trail with fade
    for i, point in enumerate(trail):
        alpha = i / len(trail)
        ax.scatter(point[0], point[1], s=5, alpha=alpha)

    # Update robot
    robot_body.center = (robot["x"], robot["y"])

    heading_line.set_data(
        [robot["x"], robot["x"] + np.cos(robot["theta"])],
        [robot["y"], robot["y"] + np.sin(robot["theta"])]
    )

    # Phone
    phone_dot.set_data(true_phone[0], true_phone[1])

    return robot_body, heading_line, phone_dot

ani = FuncAnimation(fig, animate, interval=dt*1000)
plt.show()
```

---

# 🎥 What You’ll See Now

* Smooth dark theme
* Real robot body circle
* Fading trail
* Soft red obstacle zones
* Robot wobbling slightly from noise
* Occasional jitter from BLE drop
* Wide arcs due to slip
* Natural movement inertia

It will feel much more like a physical robot.

---

# 🔬 What This Simulation Now Proves

You can observe:

* Stability under latency
* Robustness to noisy GPS
* Control tuning
* Obstacle interaction
* Motor lag effects
* Slip-induced path deviation

This is legitimately what robotics teams do before hardware.

---

# 🚀 If You Want To Go Even Further

We could next add:

* Kalman filter for sensor fusion
* Full SLAM grid mapping
* A* path planning instead of naive avoidance
* Real BLE timing model
* A clickable “Follow” UI panel
* 3D simulation (PyBullet)
* Export logs for controller tuning

If you want, next step we can:

👉 Turn this into a browser-based interactive simulator
👉 Or build a full robotics-grade architecture simulation

What level are we going to?
