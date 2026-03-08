# Robot Follower

MuJoCo simulation of a differential-drive robot that follows a person-like moving target using simulated Bluetooth signal strength (RSSI) and relative bearing, with obstacle avoidance.

## Overview

A two-wheeled robot navigates a 10m x 10m room with walls and obstacles, pursuing a target that walks along waypoints. The robot uses:

- **RSSI-like signal strength** — distance estimate from simulated Bluetooth
- **Relative bearing** — direction to target from robot heading
- **Ray-cast obstacle sensors** — front, front-left, front-right proximity
- **Behavior arbitration** — blends follow/avoid with escape fallback

## Project Structure

```
robot-follower/
├── config/
│   ├── scene.xml           # MuJoCo scene (robot, target, world)
│   └── sim_config.yaml     # Runtime parameters
├── robot_follower/
│   ├── main.py             # Entry point & simulation loop
│   ├── robot_interface.py  # Differential-drive control & pose
│   ├── target_tracker.py   # RSSI + bearing sensing
│   ├── obstacle_sensors.py # Ray-cast proximity detection
│   ├── controller.py       # Follow/avoid → wheel velocities
│   ├── behavior_manager.py # State machine (follow/avoid/escape)
│   ├── sim_env.py          # Target motion & episode lifecycle
│   ├── motion_profiles.py  # Waypoint-based target movement
│   ├── telemetry_spec.py   # BLE telemetry schema (spec only)
│   └── utils.py            # Angle math, config loading
├── tests/
│   ├── test_utils.py
│   ├── test_controller.py
│   ├── test_target_tracker.py
│   └── test_behavior_manager.py
├── pyproject.toml
└── poetry.lock
```

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
# Activate the venv in your current shell
eval "$(poetry env activate)"
```

### Headless (no GUI)

```bash
# Works on all platforms
poetry run python -m robot_follower.main --headless

# Override max simulation steps
poetry run python -m robot_follower.main --headless --max-steps 200000

# Custom config
poetry run python -m robot_follower.main --headless --config path/to/config.yaml
```

### With MuJoCo viewer (GUI)

On **macOS**, MuJoCo's `launch_passive` requires the `mjpython` launcher (ships with the `mujoco` package). Standard `python` will fail with a `RuntimeError`.

```bash
# macOS — must use mjpython for GUI
eval "$(poetry env activate)"
mjpython -m robot_follower.main --gui

# Linux / Windows — standard python works
poetry run python -m robot_follower.main --gui
```

### CLI Options

| Flag | Description |
|------|-------------|
| `--gui` | Launch MuJoCo interactive viewer |
| `--headless` | Run without GUI (default behavior) |
| `--config PATH` | Path to config YAML (default: `config/sim_config.yaml`) |
| `--max-steps N` | Override max physics steps per episode |

## Running Tests

```bash
poetry run pytest -v
```

## Configuration

All tunable parameters are in `config/sim_config.yaml`:

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
Sensors ──► Behavior Manager ──► Wheel Velocities ──► MuJoCo Actuators
              │
              ├─ FOLLOW_TARGET:  steer toward bearing, speed from RSSI
              ├─ AVOID_OBSTACLE: blend avoidance when rays detect danger
              └─ ESCAPE_TRAP:    spin in place when stalled
```

The controller runs at 50 Hz (every 20 physics steps). Target motion is updated at the same rate via waypoint interpolation.
