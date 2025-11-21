# Mission Behavior Tree Package

Behavior tree-based mission execution system for URC rover autonomous operations, integrated with GPS waypoint navigation.

## Overview

This package provides a behavior tree (BT) framework using `py_trees` library to create and execute complex autonomous missions. It integrates seamlessly with the `gps_waypoint_navigator` package to enable GPS-based waypoint navigation missions.

## Architecture

```
Mission BT Executor
    ↓
Behavior Tree (py_trees)
    ├── Condition Nodes (GPS check, battery check, etc.)
    ├── Action Nodes (navigate to waypoints, collect samples, etc.)
    └── Utility Nodes (wait, log, counters, etc.)
    ↓
GPS Waypoint Navigator Actions
    ↓
Nav2 → drive_control → Motors
```

## Features

### Action Nodes
- **GoToNamedWaypointAction**: Navigate to a named GPS waypoint
- **NavigateToGPSAction**: Navigate to raw GPS coordinates
- **FollowWaypointSequenceAction**: Follow a sequence of waypoints

### Condition Nodes
- **HasGPSFix**: Check if GPS has valid fix
- **NavigationReady**: Check if navigation system is ready
- **BatteryOK**: Check battery level (placeholder for actual battery monitoring)
- **IsNavigating**: Check if currently navigating

### Utility Nodes
- **Wait**: Wait for specified duration
- **Log**: Log messages at various levels
- **Counter**: Count iterations
- **SetBlackboard**: Set behavior tree blackboard variables
- **CheckBlackboard**: Check blackboard variable values

## Usage

### Launch Mission Executor

```bash
cd ~/workspaces/rover
source install/setup.bash

# Launch with default settings
ros2 launch mission_bt mission_bt.launch.py

# Launch with auto-start
ros2 launch mission_bt mission_bt.launch.py auto_start:=true

# Adjust tick rate
ros2 launch mission_bt mission_bt.launch.py tick_rate_hz:=20.0
```

### Creating Custom Missions in Python

```python
from mission_bt.src.bt_executor import MissionBTExecutor
from mission_bt.src.nodes.action_nodes import GoToNamedWaypointAction
from mission_bt.src.nodes.utility_nodes import Wait, Log
import py_trees
from py_trees.composites import Sequence

# Create executor
executor = MissionBTExecutor()

# Create custom mission
mission = Sequence(name="Custom Science Mission", memory=True)

# Add waypoint navigation
mission.add_child(
    Log("Mission Start", executor, "Starting custom mission")
)

mission.add_child(
    GoToNamedWaypointAction(
        "Navigate to Target",
        executor,
        waypoint_name="science_site_alpha",
        max_retries=2
    )
)

mission.add_child(
    Wait("Sample Collection", executor, duration_sec=10.0)
)

mission.add_child(
    GoToNamedWaypointAction(
        "Return Home",
        executor,
        waypoint_name="start_zone",
        max_retries=2
    )
)

# Set as active mission
executor._root = mission
executor._root.setup_with_descendants()

# Start mission
executor.start_mission()
```

### Simple Sequential Mission

```python
# Create a simple waypoint sequence
executor = MissionBTExecutor()
executor.create_simple_nav_mission([
    "start_zone",
    "science_1",
    "science_2",
    "return_point"
])
executor.start_mission()
```

## Example Missions

### Default Mission
The executor comes with a default mission that:
1. Checks GPS fix availability
2. Navigates to start zone
3. Visits science sites 1 and 2 (with sample collection waits)
4. Returns to start point

This mission runs automatically if launched with `auto_start:=true`.

### Science Collection Mission
```python
from py_trees.composites import Sequence, Selector
from py_trees.decorators import Retry

root = Sequence(name="Science Collection", memory=True)

# Pre-flight checks
preflight = Selector(name="Preflight", memory=False)
preflight.add_children([
    HasGPSFix("GPS Check", node),
    Log("No GPS", node, "GPS unavailable, aborting", "error")
])

# Navigation with retry
nav_to_science = Retry(
    name="Navigate with Retry",
    child=GoToNamedWaypointAction("Go Science", node, "science_1"),
    num_failures=3
)

# Build mission
root.add_children([
    preflight,
    nav_to_science,
    Wait("Collect", node, 15.0),
    GoToNamedWaypointAction("Return", node, "start_zone"),
])
```

### Patrol Mission
```python
# Continuous patrol between waypoints
from py_trees.composites import Sequence
from py_trees.decorators import Repeat

patrol_sequence = Sequence(name="Patrol Route", memory=True)
patrol_sequence.add_children([
    GoToNamedWaypointAction("Point A", node, "patrol_a"),
    Wait("Observe", node, 5.0),
    GoToNamedWaypointAction("Point B", node, "patrol_b"),
    Wait("Observe", node, 5.0),
])

# Repeat patrol indefinitely
patrol_mission = Repeat(
    name="Infinite Patrol",
    child=patrol_sequence,
    num_success=py_trees.common.MAX_INT
)
```

## Integration with GPS Waypoint Navigator

This package requires `gps_waypoint_navigator` to be running. The action nodes communicate with:
- `/navigate_to_gps` - NavigateToGPS action
- `/go_to_named_waypoint` - GoToNamedWaypoint action
- `/follow_gps_waypoints` - FollowGPSWaypoints action

Make sure to launch the GPS navigator before starting missions:

```bash
# Terminal 1: Launch navigation stack
ros2 launch nav2_launch nav2_bringup.launch.py

# Terminal 2: Launch GPS navigator
ros2 launch gps_waypoint_navigator gps_navigator.launch.py

# Terminal 3: Launch mission executor
ros2 launch mission_bt mission_bt.launch.py auto_start:=true
```

## Behavior Tree Visualization

py_trees provides built-in visualization:

```python
import py_trees

# Display tree structure
print(py_trees.display.unicode_tree(root, show_status=True))
```

Output example:
```
[RUNNING] URC Science Mission
    [SUCCESS] Pre-flight Checks
        [SUCCESS] Check GPS Fix
    [RUNNING] Mission Waypoints
        [SUCCESS] Starting Mission
        [SUCCESS] Go to Start Zone
        [RUNNING] Go to Science 1
```

## Blackboard System

The blackboard is a shared data structure for communication between nodes:

```python
# Set a value
SetBlackboard("Set Target", node, key="target_wp", value="science_1")

# Check a value
CheckBlackboard("Check Target", node, key="target_wp", expected_value="science_1")

# Use in custom nodes
class CustomNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key("my_key", access=py_trees.common.Access.READ)
    
    def update(self):
        value = self.blackboard.get("my_key")
        # Use value...
        return py_trees.common.Status.SUCCESS
```

## Configuration

Edit `config/mission_config.yaml` to set mission parameters:

```yaml
mission_bt:
  ros__parameters:
    tick_rate_hz: 10.0          # BT update rate
    mission_timeout: 300.0      # Max mission duration (seconds)
    auto_start: false           # Auto-start on launch
    goal_tolerance: 0.5         # Navigation goal tolerance (meters)
    retry_attempts: 3           # Default retry count
```

## Debugging

### Enable verbose logging

```bash
ros2 launch mission_bt mission_bt.launch.py --log-level debug
```

### Check BT status

```bash
# Monitor mission executor logs
ros2 topic echo /rosout | grep mission_bt

# Check navigation status
ros2 topic echo /gps_navigator/status
```

### Common Issues

**Issue: "No mission tree loaded!"**
- Solution: Ensure mission tree is created before calling `start_mission()`

**Issue: Action nodes return FAILURE immediately**
- Solution: Check GPS waypoint navigator is running
- Solution: Verify waypoint names exist in `waypoints.yaml`

**Issue: GPS fix check fails**
- Solution: Ensure GPS hardware is connected and publishing to `/gps/fix`
- Solution: Wait for GPS to acquire satellites (30-60s)

## Advanced: Custom Action Nodes

Create domain-specific action nodes:

```python
import py_trees
from rclpy.action import ActionClient

class CollectSampleAction(py_trees.behaviour.Behaviour):
    """Custom action to collect a science sample."""
    
    def __init__(self, name, node):
        super().__init__(name)
        self._node = node
        # Initialize arm action client, etc.
    
    def initialise(self):
        # Send sample collection goal
        pass
    
    def update(self):
        # Check collection status
        # Return RUNNING, SUCCESS, or FAILURE
        pass
    
    def terminate(self, new_status):
        # Cleanup if cancelled
        pass
```

## Competition Integration

For URC autonomous mission:

1. **Pre-mission setup**:
   - Record all waypoints using `record_waypoint` tool
   - Test navigation to each waypoint individually
   - Verify GPS quality and localization

2. **Mission creation**:
   - Create mission BT in Python or XML
   - Include retry logic for critical waypoints
   - Add timeout decorators for safety

3. **During competition**:
   - Launch full stack (nav2 + gps_navigator + mission_bt)
   - Monitor mission progress via logs
   - Ready to cancel with Ctrl+C if needed

## Next Steps

- Add more domain-specific action nodes (arm control, sample collection, etc.)
- Create XML-based mission definitions for easier editing
- Add mission monitoring and telemetry publishing
- Integrate with competition scoring system
- Add recovery behaviors for common failure modes

## Dependencies

- `rclpy` - ROS2 Python client library
- `py_trees` - Behavior tree library
- `gps_waypoint_navigator` - GPS waypoint navigation
- `urc_msgs` - Custom action definitions
- `nav2_msgs` - Navigation messages

## References

- py_trees documentation: https://py-trees.readthedocs.io/
- Behavior trees overview: https://www.behaviortree.dev/
- GPS Waypoint Navigator: `../gps_waypoint_navigator/README.md`

