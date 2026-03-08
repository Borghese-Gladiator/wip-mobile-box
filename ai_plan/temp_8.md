Perfect. Hybrid slowdown + reactive turn is exactly what you’d ship in a V1 physical robot.

I’ll break this into:

1. 🧠 Exact behavior model (what we’re implementing)
2. 🏗 Where it fits in the stack
3. 📡 Sensor model design
4. ⚠ Safety layers
5. 📦 Implementation plan (before code)

Then next step we can wire it into the EKF/PID simulation cleanly.

---

# 🧠 Hybrid Slowdown + Reactive Turn — What It Means

We are not doing full path planning.

We are not building a map.

We are doing **local reflex-based collision avoidance layered over phone commands**.

The robot behaves like this:

```text
Phone:      "Drive forward at 1.2 m/s"
Robot:      "Okay — unless I see something in front of me"
```

And if it sees something:

```text
Far obstacle → slow down
Closer obstacle → override angular velocity
Too close → stop
```

That’s it.

Simple. Fast. Very realistic.

---

# 🏗 Where It Lives In The Stack

Current stack:

```
Phone
  ↓
(v, ω)
  ↓
Robot PID
  ↓
Physics
  ↓
Sensors
  ↓
EKF
```

New stack:

```
Phone
  ↓
(v, ω)
  ↓
🚨 Obstacle Safety Layer (robot-side)
  ↓
Modified (v, ω)
  ↓
Wheel PID
  ↓
Physics
  ↓
Sensors
  ↓
EKF
```

Obstacle detection must sit:

* After phone commands
* Before motor control
* Before PID

So it can override.

---

# 📡 Sensor Model (Simulation Design)

We’ll simulate 3 ultrasonic-like sensors:

* Left (30° left)
* Center (forward)
* Right (30° right)

Each sensor:

* Has max range (e.g. 3m)
* Has cone angle (e.g. ±15°)
* Returns nearest obstacle in cone
* Has noise
* Has dropout probability

### In simulation:

We compute intersection between:

* Sensor ray
* Obstacle circles

We return:

```python
distance + gaussian_noise
```

---

# ⚠ Safety Zones (Important)

We’ll define three zones:

## 1️⃣ Soft Slowdown Zone (1.5m)

If center sensor distance < 1.5m:

```
slowdown_factor = distance / slowdown_threshold
v_cmd *= slowdown_factor
```

Robot slows smoothly.

---

## 2️⃣ Reactive Turn Zone (1.0m)

If obstacle closer:

Robot injects angular correction:

If obstacle more left → turn right
If obstacle more right → turn left

We compute:

```
turn_bias = (left_distance - right_distance)
ω_cmd += gain * turn_bias
```

This causes smooth steering away.

---

## 3️⃣ Emergency Stop (0.4m)

If center distance < 0.4m:

```
v_cmd = 0
ω_cmd = 0
```

Hard override.

---

# 🎯 Behavioral Goal

We want:

* Smooth slowdown
* Smooth steering
* No oscillation
* No spinning
* No jitter

This requires tuning.

---

# 📐 How It Feels Physically

This system mimics:

* Warehouse robots
* Lawn robots
* Early Roombas
* Sidewalk delivery bots

It does not globally replan.
It reflexively avoids and resumes.

---

# 🧪 Failure Cases You’ll See (Realistic)

With this method:

* Robot can get stuck in U-shaped obstacles
* Robot can oscillate between left/right
* Robot may “hug” obstacles

That’s expected for reactive systems.

---

# 🔌 Integration Plan

We’ll add:

### New Class: `RangeSensorArray`

* Simulates 3 cones
* Handles noise + dropout

### Update Robot:

Add:

```
apply_obstacle_safety(v_cmd, w_cmd)
```

This function:

* Reads sensors
* Applies layered override
* Returns safe (v, w)

---

# 📊 Visualization Upgrades

We’ll also visualize:

* Sensor cones
* Measured distances
* Obstacle detection zones
* Emergency stop state

So you can SEE when override triggers.

---

# 🧠 Important Design Choice

We must decide:

Should obstacle avoidance use:

A) True pose
B) EKF estimated pose

Real robots use estimated pose.

We will use:

EKF pose for navigation
True geometry for collision physics

This is realistic.

---

# 📦 After Hybrid Is In

Once this works, we can:

* Add Bluetooth latency model
* Add command dropout
* Add watchdog timeout
* Add motor current limiting
* Add curb detection
* Add slip detection

But hybrid first.

---

# 🚀 Next Step

I will now:

* Integrate hybrid obstacle detection
* Keep EKF + PID intact
* Keep GPS drift
* Keep encoder quantization
* Improve visualization
* Show sensor cones live
* Show slowdown region visually

Before I write it:

Do you want obstacles to be:

1. Static circles only
2. Static + moving obstacles
3. Irregular polygon shapes

For first pass, I recommend static circles.
