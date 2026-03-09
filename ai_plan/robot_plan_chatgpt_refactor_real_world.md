Yes — keep **MuJoCo**, but change how you use it.

For your project, the best move is **not** “MuJoCo 3D vs separate 2D simulator.” It is:

* use **MuJoCo as the only simulator**
* make your robot/world **planar**
* add **real-world noise, delays, and actuator limits**
* refactor the codebase so the same controller can run in:

  * `sim_planar` MuJoCo mode
  * later, `hardware` Raspberry Pi mode

So the next-stage plan becomes: **MuJoCo as a fast planar robotics testbed plus a hardware-ready software architecture**.

# Adjusted direction

## Why this is the right change

Your task is mostly planar:

* differential drive
* person following
* obstacle avoidance
* BLE-like bearing/strength estimation
* wheel control
* IMU + encoder fusion later

MuJoCo can absolutely support this by making the environment and dynamics effectively 2D:

* flat floor
* yaw rotation only as the meaningful heading DOF
* no need for complex 3D body motion
* simpler collision geometry
* top-down camera/debug views
* faster stepping and easier tuning

That gives you the benefits of:

* one simulator
* one physics engine
* easier debugging
* less code duplication
* cleaner path to hardware

# What to change in your current single `src/` project

Right now your biggest problem is not MuJoCo. It is the **single `src/` blob**.

Before buying hardware, refactor into a structure that separates:

* reusable robot logic
* MuJoCo-specific plumbing
* future Raspberry Pi hardware drivers

# Recommended directory structure

```text
robot_follower/
├── README.md
├── requirements.txt
├── pyproject.toml
├── .gitignore
├── config/
│   ├── common.yaml
│   ├── control.yaml
│   ├── sensors.yaml
│   ├── mujoco_planar.yaml
│   ├── hardware.yaml
│   └── telemetry.yaml
├── mujoco/
│   ├── assets/
│   │   ├── robot_planar.xml
│   │   ├── target.xml
│   │   ├── world_planar.xml
│   │   └── scene_planar.xml
│   ├── runner.py
│   ├── mujoco_robot.py
│   ├── mujoco_sensors.py
│   ├── mujoco_target.py
│   ├── mujoco_noise.py
│   └── visualize.py
├── core/
│   ├── __init__.py
│   ├── types.py
│   ├── config.py
│   ├── math_utils.py
│   ├── geometry.py
│   ├── filters.py
│   ├── kinematics.py
│   ├── odometry.py
│   ├── estimator.py
│   ├── behavior_manager.py
│   ├── follower_controller.py
│   ├── obstacle_avoidance.py
│   ├── wheel_controller.py
│   ├── safety_manager.py
│   └── telemetry_spec.py
├── interfaces/
│   ├── __init__.py
│   ├── robot_io.py
│   ├── motor_interface.py
│   ├── encoder_interface.py
│   ├── imu_interface.py
│   ├── ble_interface.py
│   └── range_interface.py
├── hardware/
│   ├── __init__.py
│   ├── pi_runner.py
│   ├── motor_driver.py
│   ├── encoders.py
│   ├── imu.py
│   ├── ble_scanner.py
│   ├── range_sensors.py
│   ├── battery_monitor.py
│   └── watchdog.py
├── tools/
│   ├── refactor_check.py
│   ├── tune_controller.py
│   ├── replay_logs.py
│   ├── calibrate_wheels.py
│   ├── calibrate_imu.py
│   └── plot_logs.py
├── tests/
│   ├── test_filters.py
│   ├── test_kinematics.py
│   ├── test_behavior_manager.py
│   ├── test_wheel_controller.py
│   └── test_estimator.py
└── logs/
```

# How your existing single `src/` maps into this

## Move into `core/`

Anything that is not MuJoCo-specific should move here:

* follow controller
* obstacle avoidance logic
* behavior state machine
* geometry helpers
* filtering
* telemetry schema
* wheel command generation
* future state estimation

## Move into `mujoco/`

Anything that touches MuJoCo APIs directly:

* XML loading
* stepping the simulation
* reading simulated sensors
* setting simulated actuators
* moving the target
* debug rendering

## Keep `hardware/` mostly empty for now

You said not to look at hardware shopping yet, so do not implement concrete drivers now. But create the folder and interfaces so the architecture is ready.

# How to use MuJoCo in a planar way

This is the key design change.

## 1. Simplify the world

Use:

* flat plane
* simple box obstacles
* simple capsule/box target
* top-down or angled-follow camera

Avoid:

* ramps
* uneven terrain
* complex body articulation
* dynamics that matter only in full 3D

## 2. Simplify the robot body

Use:

* simple chassis
* two drive wheels
* optional support caster approximation or stable base geometry
* low center of mass

The point is to keep motion effectively planar even though MuJoCo is still 3D underneath.

## 3. Treat only these state variables as meaningful

Your controller should operate on:

* `x`
* `y`
* `yaw`
* linear velocity
* angular velocity

Ignore roll/pitch for the controller unless you later decide to use IMU simulation details.

## 4. Use top-down debugging

Add:

* top-down camera
* drawn sensor rays or debug markers
* target bearing visualization
* obstacle zone visualization

That gets you most of the “2D simulator” benefit while staying in MuJoCo.

# Parameters to change in MuJoCo

Here is what I would adjust.

## Robot modeling parameters

Tune for a stable planar robot:

* modest wheel friction
* reduced bouncing
* stable mass/inertia
* low center of mass
* wheel actuator smoothing
* velocity or torque limits matching realistic small robots

## Environment parameters

Make them simpler and faster:

* fewer meshes, more primitive geoms
* simple wall boxes
* no unnecessary lighting complexity
* no decorative assets

## Simulation parameters

Use settings that help control tuning:

* fixed timestep
* deterministic stepping
* faster-than-real-time support for batch tuning
* configurable noise injection layer in Python rather than trying to encode all noise in XML

# What to add now for real-world readiness

The real world will break three things first:

* sensor readings
* wheel actuation
* timing

So your next work should focus on those.

## 1. Add a sensor abstraction layer immediately

Even in MuJoCo, stop reading raw model/data everywhere.

Create interfaces like:

* `read_imu()`
* `read_wheel_encoder()`
* `read_ble_target()`
* `read_obstacle_ranges()`

At first these return simulated values. Later the hardware version returns real values.

## 2. Change controller inputs from “truth” to “measured values”

Do not feed the controller:

* exact target global position
* perfect robot pose from MuJoCo
* perfect obstacle distances unless those match planned real sensors

Feed it only what the physical robot would know:

* wheel speed estimate
* heading estimate
* BLE strength + relative direction estimate
* obstacle range measurements
* stale/invalid flags

This is one of the most important refactors.

## 3. Add realistic wheel behavior in MuJoCo plus software

Your simulated wheel/motor path should include:

* PWM deadband
* saturation
* unequal left/right gain
* response lag
* static friction threshold
* battery scaling factor

Then in software, separate:

* desired wheel speed
* low-level wheel controller output
* applied actuator command

## 4. Add encoder and IMU simulation now

Even if you do not own hardware yet, add these abstractions now.

### Simulated encoder output

Should include:

* ticks or angular displacement
* quantization
* small noise
* occasional missed increments if you want harsher tests

### Simulated IMU output

Should include:

* gyro z-rate
* accel values if useful
* gyro bias
* noise
* drift over time

Your estimator can begin very simple:

* heading from gyro integrated and corrected by wheel odometry
* wheel odometry from left/right encoders
* maybe no absolute position correction at first

## 5. Make BLE unstable on purpose

For real-world BLE-following, simulate:

* noisy RSSI
* delayed packets
* missing packets
* target direction jitter
* confidence decay when signal is stale

And add logic states:

* `TRACK`
* `SEARCH`
* `AVOID`
* `STOP`

# Suggested software architecture inside the new layout

## `interfaces/robot_io.py`

Defines one main robot I/O contract:

* read all sensors
* send wheel commands
* report timing/status

## `core/types.py`

Shared dataclasses:

* `RobotStateEstimate`
* `TargetMeasurement`
* `ObstacleMeasurement`
* `WheelCommand`
* `TelemetryPacket`
* `SafetyStatus`

## `core/filters.py`

Implement:

* exponential moving average
* median filter
* deadband helper
* stale-data handling

## `core/odometry.py`

Differential drive odometry from:

* left encoder
* right encoder

## `core/estimator.py`

Fuses:

* wheel odometry
* IMU yaw rate

Keep it lightweight.

## `core/wheel_controller.py`

Low-level control:

* desired wheel speed in
* PWM-like control out
* later becomes the exact thing that talks to the Pi motor driver

## `mujoco/mujoco_noise.py`

Central place for sim realism:

* IMU noise
* BLE noise
* encoder quantization
* motor mismatch
* packet dropouts

This is better than scattering random noise across the codebase.

# Concrete roadmap from your current codebase

## Phase 1: refactor the single `src/`

Do this first.

Split your current code into:

* `core/`
* `mujoco/`
* `interfaces/`

You do not need to make it perfect. Just separate reusable logic from MuJoCo plumbing.

## Phase 2: make MuJoCo planar-oriented

Update your XML and runtime:

* simpler planar world
* simpler robot geometry
* top-down camera/debugging
* remove unnecessary 3D complexity

## Phase 3: sensor realism

Add simulated:

* BLE fluctuations
* encoder quantization
* IMU drift/noise
* obstacle sensor noise
* stale packets

## Phase 4: wheel realism

Add:

* deadband
* lag
* mismatch
* saturation
* low-level wheel speed controller abstraction

## Phase 5: estimator and safety

Add:

* odometry
* IMU-assisted heading estimate
* stale-data detection
* watchdog-style stop conditions

## Phase 6: hardware-ready interface layer

Even before hardware exists, define:

* motor interface
* encoder interface
* IMU interface
* BLE interface
* range sensor interface

Then your eventual Pi bring-up is “write adapters,” not “rewrite the robot.”

# BLE telemetry note

Since you said you likely do not need to handle robot-sent BLE telemetry here, I would still keep a spec file in `core/telemetry_spec.py` documenting:

* timestamp
* battery
* motor commands
* wheel speeds
* IMU
* controller mode
* obstacle flags
* target confidence

That will help later when you connect the real robot stack.

# Direct answer on MuJoCo vs 2D

So the adjusted recommendation is:

* **Do not switch away from MuJoCo**
* **Do not build a separate 2D simulator**
* **Use MuJoCo in a planar configuration**
* **Push realism into sensor/actuator modeling and software architecture**

That gets you the speed and simplicity you wanted from “2D,” while preserving continuity with your current implementation.

# Best immediate next steps for you

1. Refactor the single `src/` into `core/`, `mujoco/`, and `interfaces/`
2. Replace all perfect-state controller inputs with simulated measured sensors
3. Add one realism module for BLE, IMU, encoders, and motor imperfections
4. Convert the MuJoCo scene into a simplified planar test world

## What I need from you

1. Do you want me to propose an exact file-by-file migration from your current single `src/` into this new structure?
2. Do you want the planar MuJoCo setup optimized for **fast controller iteration** or **closer hardware realism** first?

## Suggested next step

I can write a concrete **refactor blueprint** that maps your likely current files/functions into the new directory structure and defines the interfaces you should implement first.
