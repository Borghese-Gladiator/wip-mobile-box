# TODO
- [ ] bootstrap Flutter
- [ ] implement Google Maps SDK to display user + robot (AND have them move)
- [ ] implement BLE
- [ ] investigate Flutter Web

# App
Flutter (cross platform) + BLE (bluetooth low energy) + Google Maps SDK

## UI

### Map Screen
- Auto-refreshing map (Google Maps)
- Show user position (GPS — absolute)
- Show robot position (telemetry — relative estimate)

### Controls
- Button: **"Follow Me"** — sends follow command to robot
- Button: **"Stop"** — immediately halts movement
- Bluetooth connection status indicator

### Robot Status Icons
- Battery level
- Motor state
- IMU data
- Position estimate (the numbers behind the relative position on the map)

---

Phone Responsibilities
* Localization (global awareness)
  * GPS processing
  * Optional map services
  * Geofencing
  * Logging
* Compute Relative Vector
  * Phone tracks its own GPS position
  * Phone tracks Robot’s estimated GPS position (from robot telemetry)
* High-Level Path Planning
  * Choose target speed
  * Avoid large obstacles if map-aware
  * Smooth velocity commands
  * Maintain follow distance
* Send Commands (Not Wheel Speeds)
  * Send to robot:
    ```
    target_linear_velocity  (m/s)
    target_angular_velocity (rad/s)
    ```
    * The robot should decide how to achieve those velocities.
