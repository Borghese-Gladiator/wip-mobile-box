# Log
Log for Learning + Issues encountered

**02/24**
What to build:
* A physics simulation
* Differential drive robot
* Phone moving target
* Follow algorithm
* Obstacle avoidance
* Real-time visualization

IDEALLY, if the simulation behaves correctly, I've eliminated 70% of the risk
* You already validated:
  * Controller math
  * Physics model
  * Follow logic
* Hardware becomes “just sensors + motors”

NEXT STEPS
* Kalman filter for sensor fusion
* Full SLAM grid mapping
* A* path planning instead of naive avoidance
* Real BLE timing model
* A clickable “Follow” UI panel
* 3D simulation (PyBullet)
* Export logs for controller tuning

NEXT STEPS (x2)
* Add a Kalman filter
* Add realistic GPS drift
* Add wheel encoder simulation
* Add real PID motor loop
* Or convert this into a ROS-like architecture
* 3D simulation (PyBullet)
* Export logs for controller tuning

ChatGPT Chat over Flutter for Web - https://chatgpt.com/c/699e7929-5124-8330-9869-c58da13b08ed


**02/21**

Initial Build Plan from this Chat with ChatGPT: https://chatgpt.com/c/6998e5e8-2b50-8330-b9b2-44acc944cc48

# Glossary
- Bleak - Python library to communicate with BLE devices
- BLE (Bluetooth Low Energy)
  - low-duty cycle, packat-based timing model for power efficiency where device remains in sleep and wake briefly to exchange data
- ROS (Robot Operating Systm) - simplify interacting with hardware, but very hard to customize and an opt-in framework (with a learning curve) -> I'm against for now unless I know a reason we should use it
- Kalman Filter - optimal, recursive, linear algorithm to estimate state of dynamic system (eg: position, velocity) from noisy measurements over time
- PID Motor Loop - feedback mechanism to measure motor's speed or position. It continuously calculates difference between desired target (Setpoint) and motor's actual state (Process Variable).
  - Proportional (P) - muscle -> corrective foce proportional to current error
  - Integral (I) - persistence -> ensures motor reaches exact target with steady "nudge" if small error persists
  - Deriviate (D) - brake -> predicts future error by measuring rate at which error is changing
- Sensor Fusion - combine data from multiple sensors (cameras, LiDAR, radar, IMUs) to produce more accurate, reliable than what any single sensor alone can do.
  - Camera
  - LiDAR
  - Radar
  - IMU - inertial measurement unit is an electronic sensor that measures and reports objects specific force (acceleration), angular rate (rotation), and sometimes magnetic field with accelerometers, gyroscopes, and magnetometers
- Roombas work by estimating position relative to a start point and pure sensor fusion (NO GPS)
- SLAM (Simultaneous Localization and Mapping) - create maps in realtime from sensors
  - SLAM enables robots, autonomous vehicles, and drones to map unknown environments while simultaneously tracking their own position within them. It uses sensors like LiDAR, cameras, or IMUs to create 3D maps in real-time, allowing for navigation without pre-existing maps

Hardware
- Wheel Encoder - electromechanical device that measures rotational speed, position, or distance traveled by wheel
  - works by converting rotation into electrical pulses via optical or magnetic sensors