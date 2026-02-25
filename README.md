# TODO
- think: name for repo
- develop: mobile app
- develop: marketing page
- develop: robot

# WIP Mobile Box
(Temporary Name and a WIP Project.)

Idea is to have a robot box that follows your phone. This enables luggage w/o carrying things explicitly

## Requirements
- Phone App to manage robot
- Robot
  - robot drives with a differential drive system (two-wheeled for V1)
  - obstacle avoidance
  - can hold items
    - eg: beers as an icebox
    - eg: beach equipment
    - eg: picnic equipment

QUESTIONS
- how well should obstacle avoidance be written? Is it okay if it gets stuck in a local minima?

## Setup
```
[Phone App]
    |
 Bluetooth BLE
    |
[Robot MCU]
    |
Sensors + Motors
```

**Phone App** — [details](app/README.md)
- Map with live GPS (user) and robot position
- "Follow Me" and "Stop" buttons
- Bluetooth connection status indicator
- Robot status icons: battery, motor state, IMU data, position estimate

**Robot** — [details](robot/README.md)
- Bluetooth Low Energy (BLE), sends telemetry: battery, position, IMU, motor state
- Differential drive (tank style): left/right motors, PWM speed control
- Obstacle avoidance

