# MoveIt Configuration for URC Rover Arm

Complete MoveIt2 integration for the 4-DOF rover arm, replacing the old custom IK system with professional-grade motion planning and control.

## Overview

This package provides a complete MoveIt2 configuration for the URC rover arm, including:

- **✅ 4-DOF Arm Support**: Full kinematics for AB_Rev, AS1_Rev, AW_Rev, AM_Rev joints
- **✅ Hardware Bridge**: Interfaces with existing motor control system
- **✅ Backward Compatibility**: Legacy IK service for existing code
- **✅ Named Poses**: Pre-configured poses for URC missions
- **✅ Teleoperation**: Game controller and keyboard control
- **✅ Safety**: Collision checking and joint limit enforcement via MoveIt
- **✅ RViz Integration**: Real-time visualization and interactive planning

## What's New vs Old System

### Old System (`old_arm/`)
- Custom Product of Exponentials (POE) inverse kinematics
- Direct motor tick commands
- Manual joint limit checking
- No collision avoidance
- Single-purpose IK service

### New System (This Package)
- **MoveIt2 Planning**: Collision-free trajectories
- **Multiple Planners**: RRT, RRTConnect, PRM, etc.
- **Named Poses**: Quick mission-specific positioning
- **Better Control**: Smooth trajectory execution
- **Hardware Compatible**: Uses same motor interface
- **Extensible**: Easy to add tools, constraints, obstacles

## Package Structure

```
moveit_config_rover/
├── config/
│   ├── rover_arm.srdf              # Semantic robot description
│   ├── kinematics.yaml             # IK solver config
│   ├── joint_limits.yaml           # Joint limits
│   ├── ompl_planning.yaml          # Motion planning config
│   └── rover_arm_controllers.yaml  # ROS2 Control config
├── launch/
│   ├── demo.launch.py              # Complete demo with RViz
│   └── move_group.launch.py        # MoveIt move_group node
├── urdf/
│   └── rover_arm.urdf.xacro        # Arm URDF (from rover description)
├── scripts/
│   ├── moveit_arm_controller.py    # High-level controller
│   └── teleop_arm.py               # Teleoperation interface
├── hardware/
│   └── rover_arm_hardware.py       # Hardware interface bridge
└── rviz/
    └── moveit.rviz                 # RViz configuration
```

## Installation & Setup

### 1. Dependencies

```bash
# Install MoveIt2 for ROS2 Humble
sudo apt install ros-humble-moveit ros-humble-moveit-visual-tools

# Install ROS2 Control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# Python dependencies
pip3 install numpy scipy
```

### 2. Build the Package

```bash
cd /home/rover/workspaces/rover
colcon build --packages-select moveit_config_rover
source install/setup.bash
```

### 3. Verify Installation

```bash
# Check package is found
ros2 pkg list | grep moveit_config_rover

# Check launch files
ros2 launch moveit_config_rover --show-args demo.launch.py
```

## Usage

### Quick Start: Demo with RViz

```bash
# Launch complete system with mock hardware
ros2 launch moveit_config_rover demo.launch.py
```

This starts:
- MoveIt move_group (motion planning)
- ROS2 Control with mock hardware
- RViz with MoveIt plugin
- Robot state publisher

**In RViz:**
1. Use the "Planning" tab in the Motion Planning panel
2. Select a named pose from the dropdown (e.g., "es_ready", "stow")
3. Click "Plan" to compute a trajectory
4. Click "Execute" to run the motion

### Running with Real Hardware

**⚠️ IMPORTANT: Test thoroughly in simulation first!**

```bash
# Launch with real hardware interface
ros2 launch moveit_config_rover demo.launch.py use_fake_hardware:=false
```

This requires:
- Motor controllers powered and connected
- Serial/CAN communication active
- Emergency stop accessible
- Hardware bridge running (`/arm_target_motor_positions` topic active)

### Teleoperation

#### Start Teleoperation Node

```bash
# With the demo running, start teleop in another terminal
ros2 run moveit_config_rover teleop_arm.py
```

#### Controller Layout

**PowerA / Xbox Controller:**
- **Left Stick**: Move end effector in X/Y plane (or rotate if A held)
- **Right Stick Y**: Move end effector up/down (Z-axis)
- **Right Stick X**: Rotate base
- **Left Trigger**: Open gripper
- **Right Trigger**: Close gripper
- **A Button**: Toggle translation/rotation mode
- **B Button**: Toggle Cartesian/joint mode
- **X Button**: Go to stow position
- **Y Button**: Go to ready position

### Programmatic Control

#### Using the Legacy IK Service (Backward Compatible)

```python
import rclpy
from rclpy.node import Node
from interfaces.srv import IK

class ArmClient(Node):
    def __init__(self):
        super().__init__('arm_client')
        self.client = self.create_client(IK, '/arm_position')
        
    def move_to_pose(self, x, y, z, rx, ry, rz):
        request = IK.Request()
        request.x = x
        request.y = y
        request.z = z
        request.rx = rx
        request.ry = ry
        request.rz = rz
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# Use it
rclpy.init()
client = ArmClient()
result = client.move_to_pose(0.3, 0.2, 0.4, 0.0, 0.0, 0.0)
print(f"Success: {result.valid == 0}")
```

#### Using MoveIt Python API

```python
import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped

# Initialize
rclpy.init()
moveit = MoveItPy(node_name="arm_commander")
arm = moveit.get_planning_component("arm")

# Move to named pose
arm.set_goal_state(configuration_name="es_ready")
plan = arm.plan()
if plan:
    moveit.execute(plan.trajectory)

# Move to Cartesian pose
target = PoseStamped()
target.header.frame_id = "base_link"
target.pose.position.x = 0.3
target.pose.position.y = 0.2
target.pose.position.z = 0.4
target.pose.orientation.w = 1.0

arm.set_goal_state(pose_stamped_msg=target)
plan = arm.plan()
if plan:
    moveit.execute(plan.trajectory)
```

## Named Poses for URC Missions

Pre-configured poses defined in `config/rover_arm.srdf`:

### General Poses
- **`home`**: All joints at zero
- **`stow`**: Compact position for driving

### Equipment Servicing Mission
- **`es_ready`**: Ready position for equipment servicing
- **`es_panel_approach`**: Approach control panel
- **`es_fine_position`**: Fine manipulation positioning

### Science Mission
- **`sc_ready`**: Ready for science tasks
- **`sc_ground_survey`**: Ground survey position
- **`sc_sample_collect`**: Sample collection position

### Delivery Mission
- **`dm_ready`**: Ready for object pickup
- **`dm_pickup_approach`**: Approach target object
- **`dm_pickup_grasp`**: Grasp position

## Hardware Interface

The hardware interface bridges MoveIt to your existing motor control system.

### Topics

**Published by Hardware Interface:**
- `/arm_target_motor_positions` (Int32MultiArray): Motor commands in ticks

**Subscribed by Hardware Interface:**
- `/get_arm_position` (Int32MultiArray): Motor encoder feedback in ticks

**Published for Monitoring:**
- `/current_cart` (Float32MultiArray): End effector position [x, y, z]
- `/current_rot` (Float32MultiArray): End effector rotation [rx, ry, rz]
- `/joint_states` (JointState): Current joint positions/velocities

### Gear Ratios (from old system)

```python
GEAR_RATIOS = {
    'AB_Rev': 50,      # Base rotation
    'AS1_Rev': 100,    # Shoulder
    'AW_Rev': 50,      # Wrist
    'AM_Rev': 50*22/9  # End effector
}
```

### Conversion Formulas

**Radians to Motor Ticks:**
```python
ticks = (radians / (2π)) * (TICKS_PER_REV * gear_ratio)
```

**Motor Ticks to Radians:**
```python
radians = (ticks / (TICKS_PER_REV * gear_ratio)) * 2π
```

## Configuration Files

### Modifying Joint Limits

Edit `config/joint_limits.yaml`:

```yaml
joint_limits:
  AB_Rev:
    max_velocity: 1.0        # rad/s
    max_acceleration: 2.0    # rad/s²
    min_position: -3.1416    # rad
    max_position: 3.1416     # rad
```

### Adding New Named Poses

Edit `config/rover_arm.srdf`:

```xml
<group_state name="my_custom_pose" group="arm">
  <joint name="AB_Rev" value="0.5"/>
  <joint name="AS1_Rev" value="-1.0"/>
  <joint name="AW_Rev" value="-0.8"/>
  <joint name="AM_Rev" value="0.0"/>
</group_state>
```

### Changing Planning Algorithm

Edit `config/ompl_planning.yaml`:

```yaml
arm:
  default_planner_config: RRTConnectkConfigDefault  # Change to desired planner
```

Available planners: RRT, RRTConnect, RRTstar, PRM, KPIECE, etc.

## Troubleshooting

### Issue: "Planning failed"
- **Check joint limits**: Ensure target pose is reachable
- **Check collisions**: Verify no self-collisions in target pose
- **Increase planning time**: Edit `ompl_planning.yaml` planning time
- **Try different planner**: Change default planner config

### Issue: "Controller manager not responding"
- **Check ROS2 Control**: Ensure `ros2_control_node` is running
- **Verify controllers**: `ros2 control list_controllers`
- **Respawn controllers**: Use spawner nodes in launch file

### Issue: "Hardware interface not publishing"
- **Check motor feedback**: Monitor `/get_arm_position` topic
- **Verify CAN/Serial**: Ensure motor controllers are connected
- **Test in mock mode**: Use `use_fake_hardware:=true` first

### Issue: "End effector pose incorrect"
- **Check URDF**: Verify joint origins and axes
- **Calibrate encoders**: Ensure zero positions are correct
- **Check gear ratios**: Verify GEAR_RATIOS match hardware

## Integration with Existing Code

### Migrating from Old IK Service

The new system provides a **compatible legacy service** at `/arm_position` (same as old system).

**No code changes needed!** Your existing code using the old IK service will work with the new MoveIt system.

Example (works with both old and new):
```python
# This code works with both old_arm/ and moveit_config_rover!
from interfaces.srv import IK

client = self.create_client(IK, '/arm_position')
request = IK.Request()
request.x, request.y, request.z = 0.3, 0.2, 0.4
request.rx, request.ry, request.rz = 0.0, 0.0, 0.0
future = client.call_async(request)
```

### Advantages of Migrating to Native MoveIt API

While backward compatibility is maintained, migrating to the native MoveIt API provides:

- **Collision avoidance**: Automatic obstacle and self-collision checking
- **Cartesian paths**: Straight-line end effector motion
- **Velocity control**: Smooth, controlled motion
- **Named poses**: Instant recall of mission-specific poses
- **Better debugging**: RViz visualization of planning

## Performance Considerations

### Planning Time
- **Default**: 5 seconds max per plan
- **Tune**: Edit `ompl_planning.yaml` for faster/more accurate planning
- **Trade-off**: Longer time = better paths

### Control Frequency
- **Joint State Broadcaster**: 50 Hz
- **Trajectory Execution**: Variable, depends on trajectory
- **Hardware Interface**: 50 Hz motor commands

### Real-Time Considerations
- MoveIt planning is NOT real-time (runs in background)
- Trajectory execution IS real-time (via ROS2 Control)
- Emergency stops handled by hardware interface

## Future Enhancements

### Planned Features
- [ ] Force/torque sensor integration
- [ ] Cartesian impedance control
- [ ] Vision-guided manipulation
- [ ] Multi-arm coordination
- [ ] Learning-based grasping

### Contributing
To add features:
1. Create branch: `feature/your-feature-name`
2. Test in simulation first
3. Document in README
4. Submit for review

## Resources

### MoveIt2 Documentation
- [MoveIt2 Tutorials](https://moveit.picknik.ai/humble/index.html)
- [MoveIt2 API Docs](https://moveit.picknik.ai/humble/doc/api/html/index.html)
- [OMPL Planners](http://ompl.kavrakilab.org/planners.html)

### ROS2 Control
- [ROS2 Control Docs](https://control.ros.org/humble/index.html)
- [Writing Hardware Interfaces](https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html)

### URC Rules
- [URC Official Rules](http://urc.marssociety.org/home/requirements-guidelines)
- [Equipment Servicing Task](http://urc.marssociety.org/home/requirements-guidelines/equipment-servicing)

## Support

**Questions?** Check:
1. This README
2. MoveIt2 troubleshooting guide
3. ROS2 Control documentation
4. Team Slack #arm-control channel

**Found a bug?** Create an issue with:
- ROS2 version
- Steps to reproduce
- Expected vs actual behavior
- Relevant log output

---

**🎉 Ready for URC! Your arm is now MoveIt-powered with professional-grade planning and control.**

