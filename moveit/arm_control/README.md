# ARM CONTROL - URC Mission Layer

**Mission-focused control layer for the URC rover arm - lean and purpose-built for competition.**

## What This Package Does

Provides **essential** high-level control for URC missions:

✅ **Gripper Control** - Simple open/close/position control  
✅ **Mission Actions** - GoToNamedPose and PickAndPlace action servers  
✅ **Safety Monitoring** - E-stop and joint limit checking  

That's it. No bloat. No over-engineering. Just what you need for URC.

## Package Contents

```
arm_control/
├── arm_control/
│   ├── gripper_controller.py      # Simple gripper control
│   ├── action_servers.py          # GoToNamedPose & PickAndPlace
│   └── safety_monitor.py          # E-stop & fault monitoring
├── action/
│   ├── GoToNamedPose.action       # Named pose action definition
│   └── PickAndPlace.action        # Pick and place action definition
├── msg/
│   └── Fault.msg                  # Fault message definition
├── old_arm/                       # Legacy system (reference only)
└── launch/
    └── arm_control.launch.py      # Launch all nodes
```

## Usage

### Launch with MoveIt

```bash
# Terminal 1: Launch MoveIt (main arm control)
ros2 launch moveit_config_rover demo.launch.py

# Terminal 2: Launch mission control layer
ros2 launch arm_control arm_control.launch.py
```

### Gripper Control

**Simple topics:**
```bash
# Open gripper (0.0 = closed, 1.0 = fully open)
ros2 topic pub --once /gripper/open std_msgs/Float32 "data: 1.0"

# Close gripper (0.0 = no force, 1.0 = maximum grip)
ros2 topic pub --once /gripper/close std_msgs/Float32 "data: 0.8"

# Set specific position (0-1000)
ros2 topic pub --once /gripper/set_position std_msgs/Int32 "data: 500"
```

**In Python:**
```python
from std_msgs.msg import Float32

# Open gripper
self.gripper_open_pub = self.create_publisher(Float32, '/gripper/open', 10)
msg = Float32()
msg.data = 1.0  # Fully open
self.gripper_open_pub.publish(msg)

# Close gripper
self.gripper_close_pub = self.create_publisher(Float32, '/gripper/close', 10)
msg = Float32()
msg.data = 0.8  # 80% grip force
self.gripper_close_pub.publish(msg)
```

### Action Servers

#### GoToNamedPose Action

```python
from rclpy.action import ActionClient
from arm_control.action import GoToNamedPose

# Create action client
client = ActionClient(self, GoToNamedPose, '/arm/go_to_named_pose')
client.wait_for_server()

# Send goal
goal = GoToNamedPose.Goal()
goal.pose_name = 'es_ready'
goal.velocity_scaling = 0.1  # 10% of max speed

future = client.send_goal_async(goal)
```

**Available named poses** (from SRDF):
- `home`, `stow` - Basic positions
- `es_ready`, `es_panel_approach`, `es_fine_position` - Equipment Servicing
- `sc_ready`, `sc_ground_survey`, `sc_sample_collect` - Science Mission
- `dm_ready`, `dm_pickup_approach`, `dm_pickup_grasp` - Delivery Mission

#### PickAndPlace Action

```python
from arm_control.action import PickAndPlace
from geometry_msgs.msg import PoseStamped

# Create action client
client = ActionClient(self, PickAndPlace, '/arm/pick_and_place')
client.wait_for_server()

# Define pick and place poses
pick_pose = PoseStamped()
pick_pose.header.frame_id = 'base_link'
pick_pose.pose.position.x = 0.3
pick_pose.pose.position.y = 0.2
pick_pose.pose.position.z = 0.1
pick_pose.pose.orientation.w = 1.0

place_pose = PoseStamped()
place_pose.header.frame_id = 'base_link'
place_pose.pose.position.x = 0.3
place_pose.pose.position.y = -0.2
place_pose.pose.position.z = 0.1
place_pose.pose.orientation.w = 1.0

# Send goal
goal = PickAndPlace.Goal()
goal.pick_pose = pick_pose
goal.place_pose = place_pose
goal.grasp_effort = 0.8  # 80% grip force

future = client.send_goal_async(goal)
```

The PickAndPlace action automatically:
1. Approaches pick position (10cm above)
2. Moves down to pick
3. Closes gripper
4. Lifts object (15cm)
5. Moves to place approach
6. Places object
7. Opens gripper
8. Retracts

### Safety Monitoring

**Monitor topics:**
```bash
# Check if arm is safe
ros2 topic echo /arm/is_safe

# Watch for faults
ros2 topic echo /arm/faults

# Trigger emergency stop
ros2 topic pub /emergency_stop std_msgs/Bool "data: true"

# Release emergency stop
ros2 topic pub /emergency_stop std_msgs/Bool "data: false"
```

## Integration with Behavior Trees

Perfect for py_trees or BehaviorTree.CPP:

```python
# In your behavior tree node
class PickUpSample(py_trees.behaviour.Behaviour):
    def setup(self):
        self.action_client = ActionClient(
            rclpy.create_node('pick_sample'),
            PickAndPlace,
            '/arm/pick_and_place'
        )
    
    def update(self):
        # Send pick and place goal
        goal = PickAndPlace.Goal()
        goal.pick_pose = self.detect_sample_pose()
        goal.place_pose = self.cache_pose
        
        result = self.action_client.send_goal(goal)
        
        if result.success:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
```

## Configuration

Edit `config/safety_params.yaml` for safety thresholds:

```yaml
safety_monitor:
  joint_limits:
    AB_Rev:
      min: -3.1416
      max: 3.1416
      velocity: 1.0
    # ... etc
```

## URC Mission Examples

### Equipment Servicing

```python
# Move to panel
client.send_goal_async(GoToNamedPose.Goal(pose_name='es_panel_approach'))

# Fine adjust with MoveIt
# (use moveit_config_rover's teleoperation or Cartesian commands)

# Operate switch/button
gripper_pub.publish(Float32(data=0.5))  # Partial close
time.sleep(2.0)
gripper_pub.publish(Float32(data=0.0))  # Release
```

### Science Mission

```python
# Go to sampling position
client.send_goal_async(GoToNamedPose.Goal(pose_name='sc_ground_survey'))

# Pick up sample
pick_goal = PickAndPlace.Goal()
pick_goal.pick_pose = sample_detector.get_sample_pose()
pick_goal.place_pose = cache_location
client.send_goal_async(pick_goal)
```

### Delivery Mission

```python
# Pick up object
pick_goal = PickAndPlace.Goal()
pick_goal.pick_pose = object_detector.get_object_pose()
pick_goal.place_pose = delivery_location
pick_goal.grasp_effort = 0.9  # Firm grip

result = client.send_goal_async(pick_goal)
```

## Architecture

```
┌─────────────────────────────────────────────┐
│         Mission Planning / Behavior Tree     │
└────────────────┬────────────────────────────┘
                 │
      ┌──────────┴──────────┐
      │                     │
┌─────▼──────┐   ┌─────────▼────────┐
│ Action     │   │    Gripper       │
│ Servers    │   │   Controller     │
└─────┬──────┘   └──────────────────┘
      │
      │ Uses MoveIt for planning
      ▼
┌──────────────────────────────────┐
│   moveit_config_rover            │
│   (Motion Planning & Execution)  │
└──────────────────────────────────┘
```

This package sits **on top of** `moveit_config_rover`, providing mission-level abstractions.

## What About old_arm/?

The `old_arm/` folder contains your previous working system. It's kept for:
- Reference for motor control logic
- Fallback if needed
- Understanding gear ratios and encoder mappings

**Don't delete it**, but don't use it - the new MoveIt system is better.

## Dependencies

- `moveit_config_rover` - Core MoveIt setup (required)
- MoveIt2 - Motion planning
- ROS2 Control - Trajectory execution
- Standard ROS2 messages

## Building

```bash
cd /home/rover/workspaces/rover
colcon build --packages-select arm_control
source install/setup.bash
```

## Testing

```bash
# Test gripper (with demo running)
ros2 topic pub --once /gripper/open std_msgs/Float32 "data: 1.0"
ros2 topic pub --once /gripper/close std_msgs/Float32 "data: 0.5"

# Test named pose action
ros2 action send_goal /arm/go_to_named_pose arm_control/action/GoToNamedPose "{pose_name: 'home'}"

# Monitor safety
ros2 topic echo /arm/is_safe
```

---

**This is everything you need for URC. Simple. Focused. Competition-ready.** 🏆
