# Plan: Fix Person Wall-Corner Sticking + Robot Follow Oscillation

## Brief
Two bugs in the robot-follower simulation:
1. **Person gets stuck on wall corners** — kinematic qpos updates push into geometry; MuJoCo pushes back; person vibrates in place.
2. **Robot oscillates at fixed distance** — proportional-only speed control with a `max(0.3, proximity)` floor means the robot never stops, creating an underdamped back-and-forth.

## Root Cause Analysis

### Bug 1: Person stuck on corners
- `MuJoCoTarget.step()` adds `dx/dy` directly to `qpos` (line 42-43 of `mujoco_target.py`)
- MuJoCo physics resolves contact after `mj_step()`, pushing the target back
- Next control tick adds `dx/dy` again → vibration loop
- The `turn_smoothing=0.3` makes heading changes sluggish, so near corners the person keeps pushing into the wall for many ticks before the heading catches up

**Fix:** After applying `dx/dy` to qpos, run `mj_forward()` to detect contacts, then check if the target body has active contacts. If so, project the displacement along the wall surface (slide along the wall normal) instead of pushing through it. Simpler approach: after writing qpos, call `mj_forward` and read back the contact-corrected position.

Actually, even simpler: MuJoCo already resolves contacts in `mj_step`. The problem is we set qpos BEFORE `mj_step` and then `mj_step` fights the displacement. The real fix is to **read the post-step position** and use that as the source of truth, rather than accumulating deltas on top of possibly-corrected positions. But we already do `get_position()` which reads `xpos` (post-step). The issue is we also zero out qvel, preventing MuJoCo from building up momentum to slide.

**Chosen fix:** Use `mj_forward` after setting qpos to detect penetration, then use MuJoCo's contact solver to resolve it. More robustly: add a simple wall-collision check in the motion profile or target mover — raycast in the movement direction and clamp displacement.

### Bug 2: Robot oscillation
- `follower_controller.py` line 24-25:
  ```python
  proximity = 1.0 - max(0.0, (target.signal_strength - 0.8) / 0.2)
  forward = self.base_speed * alignment * max(0.3, proximity)
  ```
- `max(0.3, proximity)` means minimum forward speed is `0.3 * 3.0 = 0.9 m/s` — **the robot never stops**
- No integral or derivative term — purely proportional to signal strength
- The robot overshoots the desired distance, slows, falls behind, speeds up → oscillation

**Fix:** Replace the RSSI-proximity speed modulation with a proper distance-error controller:
- Use `distance_estimate` from `TargetMeasurement` (already available from the BLE tracker)
- Compute `error = distance - follow_distance`
- Apply PD control: `forward = kp * error - kd * d_error/dt`
- Add dead zone around follow distance where forward = 0
- Allow forward speed to be zero (remove `max(0.3, ...)` floor)

## Changes

### 1. `robot_follower/mujoco_sim/mujoco_target.py`
- After setting qpos, raycast in the movement direction
- If ray hits a wall within the step distance, clamp displacement to stop short of the wall
- Use MuJoCo `mj_ray` to detect walls ahead of the target

### 2. `robot_follower/core/follower_controller.py`
- Add `follow_distance` parameter (default 1.5m, from config `target.follow_band`)
- Replace RSSI-proximity speed modulation with distance-error PD controller
- Add dead zone (±0.2m around follow_distance) where forward = 0
- Remove `max(0.3, proximity)` floor — allow the robot to stop
- Keep bearing-proportional steering (it works fine)

### 3. `robot_follower/mujoco_sim/runner.py`
- Pass `follow_distance` to `FollowerController` constructor (from config)

### 4. Config: `config/sim_config.yaml` + `config/common.yaml`
- Add `follow_distance: 1.5` under `target` section (or derive from `follow_band` midpoint)

## Tests

### Unit
- `test_controller.py`: Update existing tests + add new:
  - `test_stops_at_follow_distance` — zero forward speed within dead zone
  - `test_approaches_when_far` — positive forward when distance > follow_distance
  - `test_retreats_when_too_close` — negative forward when distance < follow_distance (or zero)
  - `test_derivative_dampens_approach` — less forward speed when closing fast

### Manual
1. Run sim headless: `python -m robot_follower.mujoco_sim.runner --headless`
2. Observe logs: robot distance to target should converge to ~1.5m and stabilize (no oscillation)
3. Run sim with GUI: `python -m robot_follower.mujoco_sim.runner --gui`
4. Watch person navigate past wall corners — should slide along walls, not get stuck
5. Watch robot follow — should maintain steady distance without back-and-forth
