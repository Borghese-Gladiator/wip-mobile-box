Absolutely. Here is the **final implementation blueprint** for a **MuJoCo differential-drive robot** that follows a **person-like moving target** using **signal strength + relative direction**, with **obstacle avoidance**, and with a note on **robot BLE telemetry** to expose later.

# Final Implementation Blueprint

## 1. Project objective

Build a MuJoCo simulation of a **2-wheeled differential-drive robot** that:

* moves using **left/right motor control**
* follows a **person-like moving target**
* estimates target location using:

  * **simulated Bluetooth signal strength**
  * **relative direction/bearing**
* avoids walls and obstacles while tracking
* keeps code organized into:

  * `src/` for Python logic
  * `config/` for MuJoCo XML and runtime config

This version will **not implement real BLE comms**, but it will include a clear telemetry interface note so the robot state can later be exported over BLE.

---

## 2. System scope

### Included in scope

* MuJoCo robot model
* differential drive kinematics
* PWM-like motor command interface
* person-like moving target
* simulated target sensing:

  * RSSI-like strength
  * relative bearing
* obstacle avoidance
* behavior arbitration between follow and avoid
* simulation loop and viewer
* clear config structure
* note/spec for robot BLE telemetry output

### Not included in v1

* real BLE stack
* real radio propagation
* camera vision
* SLAM or mapping
* learned policy / RL
* multi-robot behavior

---

## 3. Directory structure

```text
ble_person_follower/
├── src/
│   ├── main.py
│   ├── controller.py
│   ├── behavior_manager.py
│   ├── robot_interface.py
│   ├── target_tracker.py
│   ├── obstacle_sensors.py
│   ├── telemetry_spec.py
│   ├── sim_env.py
│   ├── motion_profiles.py
│   ├── utils.py
│   └── __init__.py
├── config/
│   ├── robot.xml
│   ├── target.xml
│   ├── world.xml
│   ├── scene.xml
│   └── sim_config.yaml
├── assets/
│   └── textures/
├── requirements.txt
├── README.md
└── .gitignore
```

---

## 4. High-level architecture

The system has five layers.

### A. Physics layer

MuJoCo XML defines:

* robot body and wheels
* floor
* walls and obstacles
* person-like moving target
* lighting and camera

### B. Robot actuation layer

Python converts controller output into:

* left motor PWM-like command
* right motor PWM-like command

### C. Perception layer

Python estimates:

* target signal strength
* target relative bearing
* obstacle proximity

### D. Behavior layer

Combines:

* target-follow behavior
* obstacle avoidance behavior
* escape behavior if blocked

### E. Simulation/runtime layer

Runs:

* sensor updates
* controller decisions
* MuJoCo stepping
* success/failure checks
* rendering

---

## 5. File-by-file blueprint

## `config/robot.xml`

Defines the differential-drive robot.

### Contents

* chassis body
* left wheel
* right wheel
* wheel joints
* actuators for left/right wheels
* inertial properties
* collision geoms
* optional sensor mount sites

### Design intent

Keep geometry simple and stable:

* box chassis
* cylinder wheels
* low center of mass
* stable contact friction

---

## `config/target.xml`

Defines the moving target.

### Contents

* simplified person-like body
* upright capsule/cylinder stack or simple humanoid-like marker
* visible geometry so movement is easy to track in the viewer
* target reference site for sensing

### Design intent

This is not a full humanoid simulation. It is a **person-like moving object** with smooth motion and a clear target point.

---

## `config/world.xml`

Defines the environment.

### Contents

* floor plane
* room boundaries
* static obstacles
* corridors / open spaces
* optional clutter objects

### Design intent

The world should include:

* clear pursuit zones
* narrow passages
* corners
* obstacles that require steering corrections

---

## `config/scene.xml`

Top-level MuJoCo scene.

### Contents

* includes robot, target, world
* lighting
* camera
* simulation defaults
* optional tracking camera

---

## `config/sim_config.yaml`

Holds runtime parameters.

### Contents

* simulation timestep
* wheel radius and wheelbase
* PWM limits
* max linear/angular speed
* target sensing gains
* obstacle thresholds
* follow/avoid blending weights
* target motion profile parameters
* timeout and success radius
* optional sensor noise

---

## `src/main.py`

Simulation entry point.

### Responsibilities

* load YAML config
* load MuJoCo model and data
* initialize robot interface
* initialize target tracker
* initialize controller and behavior manager
* initialize environment
* run simulation loop
* render viewer
* stop on timeout or completion

---

## `src/robot_interface.py`

Low-level robot control and state access.

### Responsibilities

* expose robot pose
* read wheel joint states
* set motor commands
* provide differential drive helpers

### Main interface

* `get_pose()`
* `get_heading()`
* `get_wheel_speeds()`
* `set_motor_pwm(left_pwm, right_pwm)`
* `stop()`

### Design note

PWM is modeled as a normalized control input mapped to actuator commands. This gives the right control architecture without simulating literal electrical PWM timing.

---

## `src/target_tracker.py`

Simulated Bluetooth-like target sensing.

### Responsibilities

* compute target position relative to robot
* estimate signal strength from distance
* compute relative bearing from robot heading
* optionally apply noise and update-rate limits

### Outputs

* target distance estimate
* RSSI-like strength
* bearing angle
* visibility/validity flag

### Design note

This is where the “Bluetooth following” abstraction lives. We simulate the usable information a robot would infer, not RF physics.

---

## `src/obstacle_sensors.py`

Obstacle proximity sensing.

### Responsibilities

* compute front distance
* compute front-left distance
* compute front-right distance
* detect blocked states
* return normalized danger values

### Implementation approach

Use MuJoCo-compatible geometry/range-style logic or simple world-relative ray checks based on robot pose and known obstacle geometry.

### Outputs

* `front_clear`
* `left_clear`
* `right_clear`
* `danger_front`
* `danger_left`
* `danger_right`

---

## `src/controller.py`

Core motion controller.

### Responsibilities

Convert follow and avoid demands into:

* desired forward velocity
* desired turn rate
* left/right PWM commands

### Internal logic

* steering term from target bearing
* speed term from target alignment and obstacle risk
* wheel mixing for differential drive

### Outputs

* `left_pwm`
* `right_pwm`

---

## `src/behavior_manager.py`

Behavior arbitration layer.

### Responsibilities

Blend or switch between:

* target pursuit
* obstacle avoidance
* trap escape

### Suggested states

* `FOLLOW_TARGET`
* `AVOID_OBSTACLE`
* `ESCAPE_TRAP`
* `TARGET_REACHED`

### Priority logic

* if obstacle danger is high, avoidance overrides follow
* if partially constrained, blend follow with avoid
* if fully trapped, rotate until a safe path appears

This keeps the robot from driving directly into objects while still trying to maintain pursuit.

---

## `src/sim_env.py`

Environment state manager.

### Responsibilities

* reset initial state
* update target motion
* check target reached conditions
* check timeout
* provide episode status

### Main functions

* `reset()`
* `step_target_motion(t)`
* `is_target_reached()`
* `is_done()`

---

## `src/motion_profiles.py`

Person-like target movement patterns.

### Responsibilities

Provide scripted motion for the target.

### Initial motion styles

* waypoint walking
* curved patrol
* gentle wandering
* stop-and-go movement

### Recommended v1 profile

A **waypoint-based walking profile** with smooth heading transitions and moderate speed variation. This feels person-like while remaining deterministic and easy to debug.

---

## `src/telemetry_spec.py`

Telemetry note/spec only.

### Responsibilities

* define what robot BLE telemetry should include
* provide packet/data schema notes
* optionally provide a helper to assemble a telemetry dict for debugging

### Important note

Per your request, this will not implement BLE transport. It will document the data the robot should send.

---

## `src/utils.py`

Shared helpers.

### Responsibilities

* angle normalization
* clamp/interpolation functions
* config loading
* coordinate transforms
* debug print helpers

---

## 6. Robot design

## Mechanical model

The robot will use:

* one rigid chassis
* two side wheels
* differential drive
* stable contact parameters

### Why this design

It is the simplest robust base for:

* turning in place
* curved pursuit
* obstacle avoidance
* repeatable behavior in MuJoCo

---

## Kinematic model

The controller works through:

* left motor command
* right motor command

### Motion mapping

* same wheel speeds: straight
* left slower than right: turn left
* right slower than left: turn right
* opposite signs: rotate in place

---

## Motor control model

We will treat motor control as:

* normalized PWM-like input in a fixed range
* mapped to actuator target velocity or effort

This is enough to preserve the “tank-style left/right motor control” concept while keeping the simulation manageable.

---

## 7. Person-like target model

The target will behave like a simplified walking person.

### Representation

* upright visual body
* smooth translational movement
* heading aligned with travel direction

### Behavior

The target does not need full leg animation. It only needs believable motion:

* moderate speed
* smooth turns
* non-robotic pathing
* occasional direction changes

### Recommended motion behavior

Use:

* waypoint route through the environment
* easing between turns
* small speed variation
* optional pause points

This makes the robot-following task meaningful and visually intuitive.

---

## 8. Sensing model

## Target sensing

The robot receives two target-related signals.

### A. Signal strength

A function of distance to target:

* stronger when near
* weaker when far
* optionally noisy

This simulates an RSSI-like quantity.

### B. Relative direction

The bearing from robot heading to target:

* zero when directly ahead
* positive/negative when off-center

This allows steering toward the target rather than only chasing stronger signal blindly.

### Result

The controller can:

* steer toward the target
* slow down when turning sharply
* continue pursuit even when alignment is imperfect

---

## Obstacle sensing

The robot also gets local obstacle awareness.

### Sensor zones

* front
* front-left
* front-right

### Why these are enough

They support:

* emergency front blocking
* choosing safer turn direction
* basic corridor navigation
* smooth avoidance without full mapping

---

## 9. Control and behavior design

## Follow behavior

Inputs:

* target bearing
* target signal strength

Outputs:

* forward speed request
* turn request toward target

### Intended effect

* strong forward motion when target is centered and path is clear
* stronger turning when target is off-axis
* reduced speed when the robot is misaligned

---

## Avoid behavior

Inputs:

* front/left/right obstacle danger

Outputs:

* turn-away request
* speed suppression

### Intended effect

* stop driving into walls
* steer around clutter
* bias toward open space

---

## Escape behavior

Triggered when:

* front blocked
* both sides constrained
* progress stalls

### Intended effect

* rotate in place or arc-turn
* recover a free path
* return to follow mode when safe

---

## Arbitration strategy

Use a hybrid of blending and state logic.

### Recommended rule

* normal case: blend follow + avoid
* high danger: avoidance dominates
* stuck case: enter escape state

This will look smoother than a pure finite-state switch but still be robust in clutter.

---

## 10. Simulation loop

Every simulation step will do the following:

1. read robot pose and wheel state
2. update moving target position
3. estimate target signal strength and bearing
4. read obstacle proximity
5. compute follow and avoid behavior
6. mix into left/right PWM commands
7. apply actuator controls
8. step MuJoCo
9. render frame
10. check for target reached or timeout

---

## 11. Success condition

The main success condition should be:

* robot reaches within a configurable radius of the target and maintains safe motion

You may also optionally define a “soft following” condition later:

* robot remains within a desired following distance band for a minimum duration

For v1, simple proximity is enough.

---

## 12. BLE telemetry note to include

You said BLE telemetry will come from the robot and does not need handling here. Agreed. But the blueprint should define what the robot ought to send.

## Recommended telemetry fields

The robot should send:

### Core state

* timestamp
* robot position `(x, y)`
* heading / yaw
* linear velocity
* angular velocity

### IMU-like data

* accelerometer `(ax, ay, az)`
* gyroscope `(gx, gy, gz)`
* orientation or yaw estimate

### Motor state

* left motor PWM command
* right motor PWM command
* left wheel speed
* right wheel speed
* motor enabled/fault flag

### Power

* battery percentage
* battery voltage
* current draw estimate if available

### Follow/task state

* target signal strength
* target bearing
* obstacle danger flags
* controller mode:

  * follow
  * avoid
  * escape
  * reached

### Diagnostics

* packet counter
* error flags
* sensor validity flags

## Telemetry format suggestion

A compact packet or JSON-like structure would be enough for development. For a production robot, use a small binary message with fixed fields.

---

## 13. Implementation phases

## Phase 1: Scene and robot

* create MuJoCo XML files
* build stable differential-drive robot
* load world and viewer

## Phase 2: Target motion

* add person-like target
* implement waypoint-based motion

## Phase 3: Sensing

* add target strength and bearing estimation
* add obstacle sensing

## Phase 4: Controller

* implement follow controller
* implement avoidance controller
* implement wheel mixing

## Phase 5: Behavior management

* blend follow and avoid
* add escape state
* tune thresholds and gains

## Phase 6: Telemetry spec and documentation

* define robot telemetry payload
* document run instructions and tuning options

---

## 14. Deliverables

The final code implementation should produce:

* complete project folder structure
* MuJoCo XML scene files
* Python simulation runner
* differential-drive control interface
* moving person-like target
* target tracker using strength + bearing
* obstacle avoidance
* behavior arbitration
* telemetry specification note
* README with setup and execution steps

---

## 15. Recommended v1 design decisions

These are the choices I recommend locking in:

* **Actuation:** velocity-based wheel control with PWM-like abstraction
* **Target motion:** waypoint-driven person-like path
* **Target sensing:** distance-to-strength plus relative bearing
* **Obstacle sensing:** front, front-left, front-right
* **Behavior:** blended pursuit + avoidance with escape fallback
* **Telemetry:** documented only, not transported

These choices give you a strong first implementation without overcomplicating the build.

---

## 16. Final blueprint summary

You will implement a **MuJoCo simulation** where a **differential-drive robot** follows a **person-like moving target** using **simulated Bluetooth-style signal strength and relative direction**, while **avoiding obstacles** through local proximity sensing and behavior arbitration. The system will be modular, file-based, and ready to extend later with real telemetry transport, richer sensing, or more advanced planning.

## What I need from you

1. Do you want the first code version to use a **simple box/cylinder robot** or a **slightly more polished robot model** with nicer visual structure?
2. Do you want the target-follow success condition to be **reach the target once** or **maintain a safe following distance continuously**?

## Suggested next step

I can now write the actual project skeleton and begin with the exact directory structure plus the first-pass contents for every file.
