# Robot Follower

MuJoCo simulation of a differential-drive robot that follows a person-like moving target using simulated Bluetooth signal strength (RSSI) and relative bearing, with obstacle avoidance. Architected for real-world hardware portability.

## Overview

A two-wheeled robot navigates a 10m x 10m room with walls and obstacles, pursuing a target that walks along waypoints. The robot uses:

- **RSSI-like signal strength** — distance estimate from simulated Bluetooth (with noise, delay, dropouts)
- **Relative bearing** — direction to target from robot heading
- **Ray-cast obstacle sensors** — front, front-left, front-right proximity
- **Behavior arbitration** — follow / avoid / escape / search states
- **Simulated realism** — motor mismatch, encoder quantization, IMU drift, BLE packet loss

## Project Structure

```
robot-follower/
├── config/
│   ├── common.yaml              # Shared runtime parameters
│   └── scene_planar.xml         # MuJoCo scene (robot, target, world)
├── robot_follower/
│   ├── __main__.py              # `python -m robot_follower` entry point
│   ├── core/                    # Pure logic — no MuJoCo dependency
│   │   ├── types.py             # Shared dataclasses (Pose2D, WheelCommand, etc.)
│   │   ├── math_utils.py        # Angle normalization, bearing, distance
│   │   ├── config.py            # YAML loading + merging
│   │   ├── filters.py           # EMA, median filter, deadband, stale tracker
│   │   ├── kinematics.py        # Differential-drive wheel ↔ twist
│   │   ├── odometry.py          # Dead-reckoning from wheel speeds
│   │   ├── estimator.py         # Fused state estimate (odometry + IMU)
│   │   ├── follower_controller.py  # Bearing + RSSI → body twist
│   │   ├── obstacle_avoidance.py   # Danger readings → avoidance twist
│   │   ├── wheel_controller.py  # Twist → wheel commands (deadband, gain)
│   │   ├── behavior_manager.py  # State machine (follow/avoid/escape/search)
│   │   ├── safety_manager.py    # Watchdog, e-stop, stall detection
│   │   ├── motion_profiles.py   # Waypoint-based target movement
│   │   └── telemetry_spec.py    # BLE telemetry schema (spec only)
│   ├── interfaces/
│   │   └── robot_io.py          # Abstract RobotIO contract
│   ├── mujoco_sim/              # MuJoCo-specific backend
│   │   ├── runner.py            # Simulation entry point + main loop
│   │   ├── sim_robot_io.py      # RobotIO implementation for MuJoCo
│   │   ├── mujoco_robot.py      # MuJoCo robot state + actuator access
│   │   ├── mujoco_sensors.py    # Raycast + BLE sensing from ground truth
│   │   ├── mujoco_target.py     # Target body motion via qpos
│   │   └── mujoco_noise.py      # Motor, IMU, encoder, BLE noise injection
│   └── hardware/                # Future Raspberry Pi drivers (stubs)
├── tests/
│   ├── test_utils.py
│   ├── test_controller.py
│   ├── test_behavior_manager.py
│   ├── test_target_tracker.py
│   ├── test_filters.py
│   ├── test_kinematics.py
│   └── test_estimator.py
├── pyproject.toml
└── poetry.lock
```

## Key Design Principle

**`core/` has zero MuJoCo imports.** All robot logic (controllers, state estimation, behavior) depends only on abstract types. The same code can run against:

- `mujoco_sim/` — MuJoCo simulation with noise injection
- `hardware/` — future Raspberry Pi with real sensors (implement `RobotIO`)

## Requirements

- Python 3.11
- Poetry 2.x

## Setup

```bash
cd robot-follower
poetry install
```

## Running

### Activate the Poetry virtualenv

```bash
eval "$(poetry env activate)"
```

### Headless (no GUI)

```bash
poetry run python -m robot_follower --headless

# Override max simulation steps
poetry run python -m robot_follower --headless --max-steps 200000

# Custom config
poetry run python -m robot_follower --headless --config path/to/config.yaml
```

### With MuJoCo viewer (GUI)

On **macOS**, MuJoCo's `launch_passive` requires the `mjpython` launcher (ships with the `mujoco` package). Standard `python` will fail with a `RuntimeError`.

```bash
# macOS — must use mjpython for GUI
eval "$(poetry env activate)"
mjpython -m robot_follower --gui

# Linux / Windows — standard python works
poetry run python -m robot_follower --gui
```

### CLI Options

| Flag | Description |
|------|-------------|
| `--gui` | Launch MuJoCo interactive viewer |
| `--headless` | Run without GUI (default behavior) |
| `--config PATH` | Path to config YAML (default: `config/common.yaml`) |
| `--max-steps N` | Override max physics steps per episode |

## Running Tests

```bash
poetry run pytest -v
```

## Configuration

All tunable parameters are in `config/common.yaml`:

| Section | Key Parameters |
|---------|---------------|
| `simulation` | `timestep` (0.001s), `max_steps` (120000) |
| `robot` | `wheel_radius` (0.04m), `wheelbase` (0.22m), `max_wheel_speed` (10 rad/s) |
| `target` | `success_radius` (0.5m), waypoint list |
| `sensing.rssi` | `ref_power`, `path_loss_exp`, `noise_std` |
| `sensing.obstacle` | `ray_length` (1.0m), `num_rays` (5) |
| `behavior` | `avoid_weight`, `escape_stall_time`, `escape_turn_speed` |

## Robot Specs

| Property | Value |
|----------|-------|
| Chassis | 0.30 x 0.20 x 0.05 m, 1.0 kg |
| Wheels | 0.04 m radius, 0.1 kg each |
| Wheelbase | 0.22 m |
| Caster | 0.02 m radius, low-friction sphere |
| Max wheel speed | 10 rad/s (~0.4 m/s linear) |
| Actuators | Velocity controllers, kv=0.5 |
| Physics | 1 kHz, implicitfast integrator |

## Architecture

```
  RobotIO (abstract)
     │
     ├── SimRobotIO (MuJoCo)          ← current
     └── HardwareRobotIO (Pi)         ← future
          │
          ▼
  ┌─────────────────────────────────────────┐
  │  core/ — hardware-agnostic logic        │
  │                                         │
  │  Sensors ──► StateEstimator             │
  │                  │                      │
  │                  ▼                      │
  │  TargetMeas ──► BehaviorManager         │
  │  ObstacleMeas    │                      │
  │                  ├─ FOLLOW_TARGET        │
  │                  ├─ AVOID_OBSTACLE       │
  │                  ├─ ESCAPE_TRAP          │
  │                  ├─ SEARCH_TARGET        │
  │                  └─ TARGET_REACHED       │
  │                  │                      │
  │                  ▼                      │
  │              WheelController            │
  │                  │                      │
  │                  ▼                      │
  │              SafetyManager ──► WheelCmd │
  └─────────────────────────────────────────┘
```

## Simulated Realism

The MuJoCo backend injects real-world imperfections:

- **Motor noise**: left/right gain mismatch (97%), response lag
- **Encoder noise**: tick quantization (1440 tpr), occasional missed ticks
- **IMU noise**: gyro bias drift, Gaussian noise
- **BLE noise**: RSSI jitter, bearing noise, packet delay (2 steps), 5% drop rate
- **Wheel controller**: deadband threshold, per-motor gain scaling
