# robot_follower

Person-following differential-drive robot package.

## Structure

```
robot_follower/
  config/           YAML tunables (robot, follow, avoid)
  src/robot_follower/
    types/          Shared dataclasses (Twist, PersonTarget, …)
    drivers/        Hardware I/O stubs (motor, encoder, lidar, camera)
    estimation/     Diff-drive odometry from encoders
    perception/     Person tracker + LiDAR obstacle sectoriser
    behaviors/      Follow, Avoid, Arbiter
    control/        Diff-drive kinematics, accel limiter
    app/            Main control loop (World) + structured logging
  tests/            Unit tests for behaviors
```

## Install

```bash
cd robot_follower
poetry install
```

## Run simulation

```python
from robot_follower import World
import matplotlib.pyplot as plt

world = World()
for _ in range(500):
    world.step()
```

Or run the full animated simulation:

```bash
python -m robot_follower.sim.simulation_full
```

## Run tests

```bash
pytest tests/
```

## Configuration

All tunables live in `config/`:

| File | Contents |
|------|----------|
| `robot.yaml` | Wheel geometry, motor PID, sensor noise |
| `follow.yaml` | Follow gains, BLE simulation, target confidence |
| `avoid.yaml` | Sector definitions, stop distances, turn bias |
