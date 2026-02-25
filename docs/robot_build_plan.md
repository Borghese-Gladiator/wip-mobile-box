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

## Hard Problems

| Problem | Addressed? |
| --- | --- |
| BLE latency | Simulated (latency buffer + packet drop model) |
| RSSI instability | Not addressed — GPS+heading approach avoids RSSI for follow logic |
| Wheel slip | Simulated (random velocity scaling) |
| GPS drift | Simulated (Gaussian noise on phone position) |
| Indoor localization | Not addressed — Phase 4 (SLAM) |
| Power management | Not addressed — hardware TBD |
| Obstacle avoidance edge cases | Partially — repulsive field works but local minima not solved |
