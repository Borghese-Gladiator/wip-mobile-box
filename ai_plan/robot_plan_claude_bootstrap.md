# Robot Follower Implementation Plan

## Brief
Implement MuJoCo differential-drive robot that follows a person-like target using simulated BLE (RSSI + bearing) with obstacle avoidance. All code inside `robot_follower/`.

## Changes

### Phase 1: Project structure + MuJoCo XML
- `config/scene.xml` — top-level scene including robot, target, world
- `config/robot.xml` — diff-drive robot (box chassis + 2 cylinder wheels)
- `config/target.xml` — person-like capsule target
- `config/world.xml` — floor, walls, obstacles
- `config/sim_config.yaml` — runtime parameters

### Phase 2: Python modules (src/)
- `src/__init__.py`
- `src/utils.py` — angle normalization, config loading, helpers
- `src/robot_interface.py` — pose, heading, set_motor_pwm
- `src/target_tracker.py` — RSSI + bearing from distance
- `src/obstacle_sensors.py` — raycast-like front/left/right proximity
- `src/motion_profiles.py` — waypoint-based target movement
- `src/controller.py` — follow steering → left/right PWM
- `src/behavior_manager.py` — FOLLOW/AVOID/ESCAPE arbitration
- `src/sim_env.py` — environment manager, target motion, done checks
- `src/telemetry_spec.py` — telemetry schema documentation
- `src/main.py` — entry point: load, loop, render

### Phase 3: Dependencies
- Add `pyyaml` to pyproject.toml

## Tests

### Unit
- `tests/test_utils.py` — angle_normalize, clamp
- `tests/test_target_tracker.py` — RSSI model, bearing calc
- `tests/test_controller.py` — wheel mixing, steering
- `tests/test_behavior_manager.py` — state transitions

### Manual
1. Run `poetry run python -m robot_follower.src.main`
2. Observe robot in MuJoCo viewer following the moving target
3. Verify robot avoids walls and obstacles
4. Verify robot reaches target proximity


# RESULTS
All 35 tests pass. Here's a summary of everything updated based on the maze-runner reference:

Changes made

scene.xml (physics & robot model):
- integrator="implicit" → "implicitfast" (better stability)
- timestep="0.002" → "0.001" (1 kHz like maze-runner)
- Floor: added condim="4", friction 1.5 (higher ground grip)
- Removed default <joint damping="0.01"/> and <flag contact="enable"/>
- Robot chassis: mass 2.0 → 1.0 kg, explicit <inertial> with diaginertia
- Wheels: radius 0.06 → 0.04 m, mass 0.3 → 0.1 kg, damping 0.02 → 0.001, condim="6" → condim="4", added priority="1" and
explicit <inertial>
- Caster: removed hinge joints (fixed like maze-runner), condim="3" → condim="1", friction 0.1 → 0.01 (smooth pivoting),
explicit <inertial>
- Actuators: kv="5.0" → kv="0.5", removed forcerange (let MuJoCo handle it)
- Front marker: made non-colliding (contype="0" conaffinity="0")

sim_config.yaml:
- timestep: 0.001, replaced max_time/control_dt with max_steps: 120000
- Robot: wheel_radius: 0.04, wheelbase: 0.22, max_wheel_speed: 10.0 (rad/s directly)
- Obstacle ray_length: 1.0 (was 1.5)
- escape_turn_speed: 4.0 (was 1.5, now in rad/s matching actuator range)

robot_interface.py:
- Pose reading via qpos (freejoint address) instead of xpos/xquat
- Added quat_to_yaw() helper
- Replaced set_motor_pwm() with set_wheel_velocities() — direct rad/s, no PWM abstraction

controller.py:
- Outputs wheel velocities (rad/s) instead of normalized PWM
- mix_to_pwm() → mix_to_wheels() with proper clamping to [-10, 10]
- Tuned gains for the new actuator scale

behavior_manager.py:
- Escape uses direct wheel velocities (spin in place)
- Removed unused import time

sim_env.py:
- Removed is_done()/is_timeout() — lifecycle managed by step count in main loop
- Moved import math to module level

main.py:
- Added argparse with --config, --max-steps, --headless, --gui (like maze-runner)
- Step-count based loop instead of time-based
- mj_forward() call after model creation to settle initial state
- Proper viewer lifecycle with try/finally
- Progress logging with robot/target positions