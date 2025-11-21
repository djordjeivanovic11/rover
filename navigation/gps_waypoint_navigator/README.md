# GPS Waypoint Navigator

**Bridge between GPS, Nav2, and `drive_control` for URC-style waypoint navigation.**

This package lets you send GPS waypoints (lat/lon) to the rover and have the full stack:

> GPS → localization → Nav2 planning → `drive_control` → motors

handle the motion, with retries and clean mission-level interfaces.

---

## What This Package Provides

- **Actions** (in `urc_msgs`):
  - `NavigateToGPS` – go to a single GPS coordinate.
  - `FollowGPSWaypoints` – follow a sequence of GPS coordinates.
  - `GoToNamedWaypoint` – go to a named waypoint from a YAML file.

- **Nodes**:
  - `gps_navigator_node` – Hosts the `NavigateToGPS` action server:
    - Converts GPS targets → `geometry_msgs/PoseStamped` in `map` frame.
    - Calls Nav2's `NavigateToPose` action.
    - Monitors result and retries on failure.
  - `waypoint_sequencer` – Hosts the `FollowGPSWaypoints` action server:
    - Navigates through a sequence of GPS waypoints.
    - Handles skip-on-failure and abort-on-first-failure logic.
  - `named_waypoint_node` – Hosts the `GoToNamedWaypoint` action server:
    - Looks up named waypoints from YAML configuration.
    - Calls `NavigateToGPS` with the looked-up coordinates.

- **Utilities**:
  - `gps_converter.py` – small helper to convert (lat, lon) offsets into map-frame positions.
  - `waypoint_manager.py` – loads named waypoints from `config/waypoints.yaml`.
  - `record_waypoint.py` – CLI tool to record the **current GPS fix** as a named waypoint.

---

## High-Level Data Flow

### 1. Full Stack Overview

```text
GPS (/gps/fix)  ZED + IMU
       │           │
       ├──> loc_fusion (robot_localization + navsat_transform)
       │           │
       │           └──> TF: map ↔ odom ↔ base_link
       │
Mission / BT  ──>  GPS Waypoint Navigator  ──>  Nav2 (NavigateToPose)
 (GoToNamed)        (this package)              (planner + controller)
                                              │
                                         /cmd_vel
                                              │
                                          drive_control
                                              │
                                            Motors
```

### 2. Inside `gps_waypoint_navigator`

```text
Action: NavigateToGPS (lat, lon)
       ↓
gps_navigator_node:
  - reads current GPS fix (/gps/fix)
  - reads current pose (TF map→base_link)
  - sets a local reference (lat0, lon0) ↔ (x0, y0)
  - converts target (lat, lon) → (x, y) in map frame
       ↓
Nav2 NavigateToPose:
  - plans a path in map frame
  - drives robot using /cmd_vel
       ↓
drive_control:
  - converts /cmd_vel → wheel speeds → /drive/left_rpm, /drive/right_rpm
       ↓
drive_firmware:
  - sends RPM to VESCs → motors turn
```

---

## Files and Responsibilities

- `gps_navigator_node.py`
  - Node name: `gps_navigator`
  - Action server: `navigate_to_gps` (`urc_msgs/NavigateToGPS`)
  - Subscribes: `/gps/fix` (for current lat/lon + quality).
  - Uses TF: `map` → `base_link` to know where the rover is.
  - Uses `GPSConverter` to turn (lat, lon) into a `PoseStamped` goal in `map`.
  - Uses `NavigateToPose` action client to send that goal to Nav2.
  - Exposes basic retry logic and publishes status on `/gps_navigator/status`.

- `waypoint_sequencer.py`
  - Node name: `waypoint_sequencer`
  - Action server: `follow_gps_waypoints` (`urc_msgs/FollowGPSWaypoints`)
  - Accepts a list of GPS coordinates and navigates to each in sequence.
  - Configurable behavior for handling failures (skip, abort, retry).
  - Uses `NavigateToGPS` action client for individual waypoint navigation.

- `named_waypoint_node.py`
  - Node name: `named_waypoint_node`
  - Action server: `go_to_named_waypoint` (`urc_msgs/GoToNamedWaypoint`)
  - Loads waypoints from `config/waypoints.yaml`.
  - Looks up waypoint coordinates by name and calls `NavigateToGPS`.
  - Provides mission-friendly interface for named waypoints.

- `gps_converter.py`
  - Stores a reference GPS position `(lat0, lon0)` and the corresponding map pose `(x0, y0)`.
  - For a target `(lat, lon)`:
    - Computes local East/North offsets relative to `(lat0, lon0)`.
    - Adds those to `(x0, y0)` to get the target `(x, y)` in `map`.
  - Returns a `PoseStamped` with:
    - `header.frame_id = "map"`
    - `pose.position.{x,y}` set to target.

- `waypoint_manager.py`
  - Loads YAML from `config/waypoints.yaml`:
    ```yaml
    waypoints:
      start_zone:
        latitude: 38.4063
        longitude: -110.7918
      science_1:
        latitude: 38.4070
        longitude: -110.7920
    ```
  - Provides `lookup(name) -> (lat, lon)`.

- `record_waypoint.py`
  - Small CLI:
    ```bash
    ros2 run gps_waypoint_navigator record_waypoint --name my_point
    ```
  - Subscribes to `/gps/fix`, waits for a valid fix, then writes:
    ```yaml
    waypoints:
      my_point:
        latitude: <current_lat>
        longitude: <current_lon>
    ```
  - Useful in the field to build a named waypoint list.

---

## How It Integrates with Nav2

### Launch

Nav2 bringup is extended in `nav2_launch/launch/nav2_bringup.launch.py`:

- It now launches:
  - the standard Nav2 stack (map server, planner, controller, BT, etc.)
  - `drive_control` (`wheel_drive.launch.py`) – converts `/cmd_vel` to wheel RPM.
  - `gps_navigator` (`gps_navigator.launch.py`) – this package’s node.

### Runtime Behavior

1. **Localization (`loc_fusion`)** runs:
   - ZED odometry + IMU + GPS fused.
   - TF tree: `map ↔ odom ↔ base_link` is stable.

2. **Nav2**:
   - Listens for `NavigateToPose` goals in `map` frame.
   - Computes path, publishes `/cmd_vel`.

3. **GPS Navigator**:
   - Receives `NavigateToGPS` goals (lat/lon).
   - Converts them into `NavigateToPose` goals for Nav2.
   - Tracks Nav2 goal status; retries or aborts as needed.

4. **drive_control**:
   - Already wired into Nav2:
     - `/cmd_vel` → `twist_to_wheels` → `/cmd_wheels` → `wheel_bridge`.
     - `wheel_bridge` sends:
       - either `/drive/left_rpm` + `/drive/right_rpm` (micro-ROS mode), or
       - ASCII commands over Serial/UDP (Arduino firmware).

So **any Nav2 goal in `map` frame** – whether from RViz, a mission BT, or `gps_navigator` – uses the exact same drive pipeline you already tested.

---

## How GPS Goals Translate to Motor Commands

End-to-end:

```text
1) Mission sends:
   action: /navigate_to_gps (lat, lon)

2) gps_navigator:
   - reads current GPS + map pose
   - computes target (x, y) in 'map'
   - calls Nav2 NavigateToPose with that target

3) Nav2:
   - plans path, generates /cmd_vel

4) drive_control:
   - /cmd_vel → /cmd_wheels (v_left/v_right in m/s)
   - /cmd_wheels → /drive/left_rpm, /drive/right_rpm (or serial L/R)

5) drive_firmware (Teensy):
   - applies RPM to VESCs → motors spin.
```

If Nav2 decides “turn left, then move 15m south-west”, that logic is **unchanged** – the only new piece is that you can now express the goal in GPS coordinates instead of map coordinates.

---

## Usage Summary

### Launch Full Stack (Nav2 + drive_control + GPS navigator)

```bash
cd ~/workspaces/rover
source install/setup.bash

ros2 launch nav2_launch nav2_bringup.launch.py \
  track_width:=0.42 \
  wheel_radius:=0.105
```

### Record a Waypoint

```bash
source install/setup.bash
ros2 run gps_waypoint_navigator record_waypoint --name science_1
```

### Navigate to a Raw GPS Coordinate

```bash
ros2 action send_goal /navigate_to_gps urc_msgs/action/NavigateToGPS \
  "{latitude: 38.4063, longitude: -110.7918, waypoint_name: 'test', max_retries: 1}"
```

### Navigate to a Named Waypoint

```bash
ros2 action send_goal /go_to_named_waypoint urc_msgs/action/GoToNamedWaypoint \
  "{waypoint_name: 'science_1', max_retries: 2}"
```

Internally that will:

1. Look up `science_1` in `waypoints.yaml`.
2. Convert its lat/lon into a `map`-frame goal.
3. Call Nav2 `NavigateToPose`.
4. Let `drive_control` and `drive_firmware` execute the motion.

### Follow a Sequence of GPS Waypoints

```bash
ros2 action send_goal /follow_gps_waypoints urc_msgs/action/FollowGPSWaypoints \
  "{latitudes: [38.4063, 38.4070, 38.4075], longitudes: [-110.7918, -110.7920, -110.7925], \
    waypoint_names: ['start', 'science_1', 'science_2'], max_retries: 2, \
    skip_on_failure: true, abort_on_first_failure: false}"
```

This will:
1. Navigate to each waypoint in sequence.
2. Skip failed waypoints if `skip_on_failure: true`.
3. Continue until all waypoints are processed or an error occurs.

---

## Integration with Mission/Behavior Trees

From a mission behavior tree or mission node, you can now:

```python
from rclpy.action import ActionClient
from urc_msgs.action import GoToNamedWaypoint

# In your mission node
self.waypoint_client = ActionClient(self, GoToNamedWaypoint, '/go_to_named_waypoint')

# Usage
goal = GoToNamedWaypoint.Goal()
goal.waypoint_name = "science_1"
goal.max_retries = 2
future = self.waypoint_client.send_goal_async(goal)
```

## Next Steps (Optional Enhancements)

- Add more robust GPS quality checks (covariance / HDOP thresholds).
- Add diagnostics publishing for GPS fix quality and navigation status.
- Add backup/recovery behaviors on navigation failure.
- Create behavior tree nodes that wrap these action servers for mission integration.

The core plumbing – **GPS → Nav2 → drive_control → motors** – is now fully implemented. This package is the adapter layer that makes GPS waypoints usable by your existing navigation stack.


