# Plan: Separate simulation package from robot_follower

## Brief

Restructure `demo/` into a standalone `simulation/` package that lives as a
sibling to `robot_follower/` under the monorepo root. The simulation imports
`robot_follower` as a dependency (absolute imports, not relative). Tests for
simulation live in `simulation/tests/`, separate from sim source code.

## Changes

### 1. Create `simulation/` project (sibling to `robot_follower/`)

```
wip-mobile-box/
├── robot_follower/          # existing package (unchanged)
│   ├── src/robot_follower/
│   ├── tests/
│   └── pyproject.toml
└── simulation/              # NEW standalone project
    ├── src/simulation/
    │   ├── __init__.py
    │   ├── __main__.py      # python -m simulation
    │   ├── config.py
    │   ├── sensors.py
    │   ├── scenarios.py
    │   ├── controller.py
    │   ├── metrics.py
    │   ├── world.py
    │   └── render.py
    ├── tests/
    │   ├── __init__.py
    │   ├── test_sensors.py
    │   ├── test_scenarios.py
    │   ├── test_controller.py
    │   └── test_metrics.py
    └── pyproject.toml        # depends on robot_follower + pybullet
```

### 2. Rewrite all imports from relative to absolute

`from ..behaviors.follow import FollowBehavior`
→ `from robot_follower.behaviors.follow import FollowBehavior`

`from ..types.messages import Twist`
→ `from robot_follower.types.messages import Twist`

### 3. Remove `demo/` from robot_follower

Delete `robot_follower/demo/` and the `[tool.poetry.group.demo.dependencies]`
section from `robot_follower/pyproject.toml`.

### 4. Clean up robot_follower

Remove pybullet from robot_follower deps. It's now only in simulation.

## Tests

### Unit (simulation/tests/ — no PyBullet dependency)
- test_sensors.py — BLE path-loss, packet drop, ultrasonic stub
- test_scenarios.py — person policies return correct positions
- test_controller.py — state machine transitions
- test_metrics.py — collision count, follow band %, reacquire time

### Manual
```bash
cd simulation
poetry install
poetry run python -m simulation --save-figs
```
