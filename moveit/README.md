# MoveIt Integration for URC Rover Arm

**Complete MoveIt2-based control system for the URC rover's 4-DOF manipulator arm.**

## 🎯 What This Is

A professional-grade arm control system that replaces custom IK with MoveIt2, providing:
- **Collision-free motion planning**
- **Named poses for URC missions**
- **Hardware-compatible interface**
- **Action servers for behavior trees**
- **Simple gripper control**
- **Safety monitoring**

## 🧠 How It Works

### System Flow:
```
Your Mission Code
    ↓
"Move to es_ready pose"
    ↓
MoveIt Planning (finds collision-free path)
    ↓
ROS2 Control (executes trajectory)
    ↓
Hardware Interface (radians → motor ticks)
    ↓
Motors (TICKS_PER_REV × gear_ratio)
    ↓
Arm Moves!
```

### Key Components:

**1. MoveIt (The Brain)**
- Takes goal: "go to es_ready" or Cartesian pose (x,y,z)
- Plans collision-free trajectory using OMPL
- Outputs: sequence of joint angles over time

**2. ROS2 Control (The Executor)**
- Receives trajectory from MoveIt
- Interpolates smooth motion at 50 Hz
- Sends position commands to hardware

**3. Hardware Interface (The Translator)**
- Converts radians → motor encoder ticks
- Formula: `ticks = (radians / 2π) × TICKS_PER_REV × gear_ratio`
- Publishes to `/arm_target_motor_positions`
- Reads from `/get_arm_position`

**4. Your Motors (The Muscle)**
- Receive tick commands (same format as old system)
- Move to positions
- Report encoder feedback

### What Replaced What:

| Old System | New System | Why Better |
|------------|------------|------------|
| Custom POE IK | MoveIt IK | Better singularity handling |
| Direct motor commands | Trajectory execution | Smooth, collision-free paths |
| Manual limit checking | MoveIt constraints | Automatic enforcement |
| Single pose movements | Multi-waypoint plans | Complex motions possible |

### Named Poses (How They Work):

1. Defined in SRDF: `es_ready = [0, -1.5, 0, -1.5]` (joint angles)
2. You call: `arm.set_goal_state(configuration_name="es_ready")`
3. MoveIt plans path from current position → es_ready
4. Avoids self-collisions and joint limits
5. Hardware executes the plan

### Hardware Compatibility:

**Same as old system:**
- Topic: `/arm_target_motor_positions` (Int32MultiArray)
- Topic: `/get_arm_position` (Int32MultiArray)
- Constants: TICKS_PER_REV = 1600
- Gear ratios: [50, 100, 50, 50×22/9]

**Different:**
- Commands come from MoveIt instead of custom IK
- Smoother trajectories (interpolated)
- Collision checking built-in

## 📦 Package Overview

### `moveit_config_rover/` - Core MoveIt Setup ⭐
The main package you'll use daily. Contains everything for motion planning and execution.

**What's inside:**
- URDF (arm description)
- SRDF (planning groups, named poses)
- MoveIt configurations (kinematics, planning, controllers)
- Hardware interface (bridges to your motors)
- Teleoperation scripts

**Use it for:** All your motion planning and arm control

### `arm_control/` - Mission Control Layer 🎮
High-level mission interface for URC tasks.

**What's inside:**
- Action servers (GoToNamedPose, PickAndPlace)
- Gripper controller
- Safety monitor

**Use it for:** Mission scripts and behavior tree integration

### `old_arm/` - Legacy Reference 📚
Your previous working system. Kept for reference.

**Don't use it** - just keep for comparing motor control logic

## 🚀 Quick Start

### 1. Build Everything

```bash
cd /home/rover/workspaces/rover
colcon build --packages-select rover_description arm_control moveit_config_rover
source install/setup.bash
```

### 2. Launch Demo

```bash
# Complete system with RViz
ros2 launch moveit_config_rover demo.launch.py
```

### 3. Test in RViz

1. Open RViz (auto-launches with demo)
2. Go to "Planning" tab in Motion Planning panel
3. Select "arm" planning group
4. Choose named pose: "home", "stow", "es_ready", etc.
5. Click "Plan"
6. Click "Execute"

### 4. Add Mission Control

```bash
# In another terminal (while demo is running)
ros2 launch arm_control arm_control.launch.py
```

## 📖 Documentation

- **[BUILD_AND_TEST.md](./BUILD_AND_TEST.md)** - Complete build and testing guide
- **[moveit_config_rover/README.md](./moveit_config_rover/README.md)** - Main package documentation
- **[arm_control/README.md](./arm_control/README.md)** - Mission control layer guide

## 🎮 Usage Examples

### Move to Named Pose

```python
from moveit.planning import MoveItPy

moveit = MoveItPy(node_name="arm_commander")
arm = moveit.get_planning_component("arm")

# Move to equipment servicing ready position
arm.set_goal_state(configuration_name="es_ready")
plan = arm.plan()
if plan:
    moveit.execute(plan.trajectory)
```

### Control Gripper

```bash
# Open
ros2 topic pub --once /gripper/open std_msgs/Float32 "data: 1.0"

# Close with 80% force
ros2 topic pub --once /gripper/close std_msgs/Float32 "data: 0.8"
```

### Pick and Place

```python
from rclpy.action import ActionClient
from arm_control.action import PickAndPlace
from geometry_msgs.msg import PoseStamped

# Create client
client = ActionClient(node, PickAndPlace, '/arm/pick_and_place')

# Define poses
pick = PoseStamped()
pick.header.frame_id = 'base_link'
pick.pose.position.x = 0.3
pick.pose.position.y = 0.2
pick.pose.position.z = 0.1
pick.pose.orientation.w = 1.0

place = PoseStamped()
# ... (similar for place pose)

# Execute
goal = PickAndPlace.Goal()
goal.pick_pose = pick
goal.place_pose = place
goal.grasp_effort = 0.8

future = client.send_goal_async(goal)
```

## 🏆 URC Mission Integration

### Equipment Servicing

```python
# Approach panel
arm.set_goal_state(configuration_name="es_panel_approach")
arm.plan_and_execute()

# Fine positioning with teleoperation or Cartesian control
```

### Science Mission

   ```python
# Sample collection sequence
arm.set_goal_state(configuration_name="sc_ground_survey")
arm.plan_and_execute()

# Pick up sample
pick_goal = PickAndPlace.Goal()
pick_goal.pick_pose = sample_location
pick_goal.place_pose = cache_location
pick_place_client.send_goal(pick_goal)
```

### Delivery Mission

```python
# Object pickup and delivery
pick_goal = PickAndPlace.Goal()
pick_goal.pick_pose = object_detector.get_pose()
pick_goal.place_pose = delivery_zone
pick_goal.grasp_effort = 0.9  # Firm grip
pick_place_client.send_goal(pick_goal)
```

## 🔧 Configuration

### Named Poses (SRDF)

Edit `moveit_config_rover/config/rover_arm.srdf` to add new poses:

```xml
<group_state name="my_custom_pose" group="arm">
  <joint name="AB_Rev" value="0.5"/>
  <joint name="AS1_Rev" value="-1.0"/>
  <joint name="AW_Rev" value="-0.8"/>
  <joint name="AM_Rev" value="0.0"/>
</group_state>
```

### Joint Limits

Edit `moveit_config_rover/config/joint_limits.yaml`:

```yaml
joint_limits:
  AB_Rev:
    max_velocity: 1.0
    max_acceleration: 2.0
```

### Hardware Interface

Edit `moveit_config_rover/hardware/rover_arm_hardware.py` to adjust:
- Gear ratios
- Encoder mappings
- Motor communication

## 📊 System Architecture

```
┌─────────────────────────────────────────────┐
│         Mission Planning Layer               │
│    (Your Python scripts / Behavior Tree)     │
└────────────┬────────────────────────────────┘
             │
    ┌────────┴────────┐
    │                 │
┌───▼────────┐  ┌────▼──────────┐
│  Action    │  │    Gripper    │
│  Servers   │  │   Controller  │
└───┬────────┘  └───────────────┘
    │
    │ Uses MoveIt
    ▼
┌──────────────────────────────────────────────┐
│         MoveIt Move Group                     │
│  (Motion Planning + Collision Checking)       │
└────────────┬─────────────────────────────────┘
             │
┌────────────▼─────────────────────────────────┐
│         ROS2 Control                          │
│  (Trajectory Execution + Joint Control)       │
└────────────┬─────────────────────────────────┘
             │
┌────────────▼─────────────────────────────────┐
│      Hardware Interface                       │
│  (Motor Commands <-> Encoder Feedback)        │
└───────────────────────────────────────────────┘
```

## 🎓 Learning Resources

- **MoveIt2 Tutorials:** https://moveit.picknik.ai/humble/
- **ROS2 Control:** https://control.ros.org/humble/
- **OMPL Planners:** http://ompl.kavrakilab.org/

## 🐛 Troubleshooting

### Build Errors?
See [BUILD_AND_TEST.md](./BUILD_AND_TEST.md) for complete troubleshooting guide.

### Planning Failures?
- Check joint limits are reasonable
- Verify target pose is reachable
- Try different planner (edit ompl_planning.yaml)
- Increase planning time

### Hardware Not Responding?
- Check `/arm_target_motor_positions` topic
- Verify encoder feedback on `/get_arm_position`
- Test with `use_fake_hardware:=true` first

### RViz Issues?
- Regenerate configs after URDF changes
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify all transforms publishing

## 🎯 What You Need for URC

**Essential (use daily):**
- ✅ `moveit_config_rover` - Motion planning
- ✅ `arm_control/gripper_controller` - Gripper control
- ✅ Named poses for missions

**Nice to have (mission-specific):**
- 🎮 Action servers for behavior trees
- 📊 Safety monitoring
- 🤖 Pick and place automation

**Don't need:**
- ❌ `arm_template` (deleted - was just a template)
- ❌ `moveit_config_template` (deleted - empty skeleton)
- ❌ Complex force control (unless you have F/T sensors)

## 📝 Next Steps

1. ✅ Build and test (see BUILD_AND_TEST.md)
2. 🎮 Practice in RViz with mock hardware
3. 🔧 Test with real hardware
4. 📝 Write mission-specific code
5. 🏆 Dominate URC!

---

**Built for URC 2025 - Go Rovers! 🚀**



# Build and Test Guide for MoveIt Rover Arm

Complete guide to building and testing the MoveIt-based arm control system.

## 📋 Pre-Build Checklist

### 1. Dependencies Installed?

```bash
# Check MoveIt2
ros2 pkg list | grep moveit

# Check ROS2 Control
ros2 pkg list | grep ros2_control

# If missing, install:
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
```

### 2. Python Dependencies

```bash
pip3 install numpy scipy
```

### 3. Rover Description Package Built?

```bash
# The arm control depends on rover_description for meshes
cd /home/rover/workspaces/rover
colcon build --packages-select rover_description
```

## ⚠️ CRITICAL: Test Sequence

**ALWAYS test in simulation before connecting to real hardware!**

### Correct Testing Order:
1. ✅ **Build** (5 minutes)
2. ✅ **Test in RViz Simulation** (10 minutes) - SAFE, catches bugs
3. ✅ **Verify Hardware Interface** (offline)
4. ⚠️ **Test with Real Hardware** (with E-stop ready!)

### Why This Order?
- Simulation catches 90% of bugs without risk
- Prevents damage to motors/arm
- Faster iteration (no mechanical troubleshooting)
- Verifies planning works before hardware moves

---

## 🔨 Build Process

### Step 1: Build Messages First (arm_control)

```bash
cd /home/rover/workspaces/rover

# Build only arm_control to generate messages/actions
colcon build --packages-select arm_control

# Source the workspace
source install/setup.bash
```

**Expected output:**
- `Starting >>> arm_control`
- `Finished <<< arm_control [Xs]`

**Common errors:**
- `Package 'arm_template' not found` - ✅ Fixed (removed dependency)
- `ToolChange.action not found` - ✅ Fixed (removed from CMakeLists.txt)

### Step 2: Build MoveIt Config

```bash
# Build moveit_config_rover
colcon build --packages-select moveit_config_rover

# Source again
source install/setup.bash
```

**Expected output:**
- `Starting >>> moveit_config_rover`
- `Finished <<< moveit_config_rover [Xs]`

### Step 3: Verify Messages Generated

```bash
# Check that action interfaces were created
ros2 interface list | grep arm_control

# Should see:
# arm_control/action/GoToNamedPose
# arm_control/action/PickAndPlace
# arm_control/msg/ArmStatus
# arm_control/msg/Fault
```

### Step 4: Full Rebuild (Clean Build)

```bash
cd /home/rover/workspaces/rover

# Remove old build artifacts
rm -rf build/ install/ log/

# Build everything
colcon build --packages-select rover_description arm_control moveit_config_rover

# Source workspace
source install/setup.bash
```

## ✅ Verification Tests

### Test 1: Check Package Installation

```bash
# List installed packages
ros2 pkg list | grep -E "arm_control|moveit_config_rover|rover_description"

# Should show all three packages
```

### Test 2: Check Launch Files

```bash
# List launch files for moveit_config_rover
ros2 launch moveit_config_rover --show-args demo.launch.py

# Should show:
# Arguments (pass arguments as '<name>:=<value>'):
#   'use_sim_time': ...
#   'use_fake_hardware': ...
#   'rviz': ...
```

### Test 3: Verify URDF

```bash
# Check if URDF can be parsed
ros2 run xacro xacro /home/rover/workspaces/rover/install/moveit_config_rover/share/moveit_config_rover/urdf/rover_arm.urdf.xacro > /tmp/test_arm.urdf

# Check for errors (should be none)
check_urdf /tmp/test_arm.urdf
```

### Test 4: Launch Demo (Mock Hardware) ⭐ START HERE

```bash
# Terminal 1: Launch MoveIt demo with mock hardware
ros2 launch moveit_config_rover demo.launch.py

# Wait for "You can start planning now!" message
# RViz should open showing the arm

# Terminal 2: Check topics
ros2 topic list | grep -E "arm|joint|gripper"

# Should see topics like:
# /joint_states
# /arm_controller/...
# /move_group/...
```

**In RViz (Motion Planning plugin):**
1. Select "arm" planning group
2. Go to "Planning" tab
3. Select named pose: "home"
4. Click "Plan" → should see orange trajectory
5. Click "Execute" → arm moves in simulation
6. Try other poses: "stow", "es_ready", etc.

**This verifies:**
- ✅ URDF loads correctly
- ✅ MoveIt planning works
- ✅ Joint limits are reasonable
- ✅ Named poses are reachable
- ✅ No code errors

### Test 5: Test Action Servers

```bash
# Terminal 1: Keep demo running

# Terminal 2: Launch arm control layer
ros2 launch arm_control arm_control.launch.py

# Terminal 3: Test action
ros2 action send_goal /arm/go_to_named_pose arm_control/action/GoToNamedPose "{pose_name: 'home'}"
```

## 🐛 Common Build Errors and Solutions

### Error: `Package 'arm_template' not found`
**Solution:** Already fixed - removed from package.xml

### Error: `ToolChange.action` not found
**Solution:** Already fixed - removed from CMakeLists.txt

### Error: `Could not find a package configuration file provided by "moveit"`
**Solution:**
```bash
sudo apt install ros-humble-moveit
```

### Error: `ImportError: cannot import name 'GoToNamedPose'`
**Solution:** Messages not built yet. Run:
```bash
colcon build --packages-select arm_control
source install/setup.bash
```

### Error: `xacro: error: could not find the following packages`
**Solution:** Build rover_description first:
```bash
colcon build --packages-select rover_description
```

### Error: `MoveItPy initialization failed`
**Solution:** Make sure all MoveIt packages are installed:
```bash
sudo apt install ros-humble-moveit-ros-planning-interface \
                 ros-humble-moveit-kinematics \
                 ros-humble-moveit-planners
```

## 🚀 Full System Launch Sequence

### For Development/Testing (Mock Hardware):

```bash
# Terminal 1: MoveIt + RViz + Mock Hardware
cd /home/rover/workspaces/rover
source install/setup.bash
ros2 launch moveit_config_rover demo.launch.py

# Terminal 2: Mission Control Layer
source install/setup.bash
ros2 launch arm_control arm_control.launch.py

# Terminal 3: Teleoperation (optional)
source install/setup.bash
ros2 run moveit_config_rover teleop_arm.py
```

### For Real Hardware:

```bash
# Terminal 1: MoveIt with Real Hardware
cd /home/rover/workspaces/rover
source install/setup.bash
ros2 launch moveit_config_rover demo.launch.py use_fake_hardware:=false

# Terminal 2: Mission Control Layer
source install/setup.bash
ros2 launch arm_control arm_control.launch.py

# Terminal 3: Monitor hardware
ros2 topic echo /arm_target_motor_positions
ros2 topic echo /get_arm_position
```

## 📊 System Health Checks

### Check All Nodes Running:

```bash
ros2 node list

# Should see:
# /move_group
# /robot_state_publisher
# /controller_manager
# /gripper_controller
# /arm_action_servers
# /safety_monitor
# /rviz2
```

### Check Controllers:

```bash
ros2 control list_controllers

# Should see:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# arm_controller[joint_trajectory_controller/JointTrajectoryController] active
```

### Monitor Joint States:

```bash
ros2 topic echo /joint_states --once
```

### Check for Errors:

```bash
# Watch for any errors in the logs
ros2 topic echo /rosout | grep -i error
```

## 🎯 Quick Test Scenarios

### Scenario 1: Move to Named Pose (Command Line)

```bash
ros2 action send_goal /arm/go_to_named_pose arm_control/action/GoToNamedPose \
  "{pose_name: 'home', velocity_scaling: 0.1}"
```

### Scenario 2: Open/Close Gripper

```bash
# Open gripper
ros2 topic pub --once /gripper/open std_msgs/Float32 "data: 1.0"

# Close gripper
ros2 topic pub --once /gripper/close std_msgs/Float32 "data: 0.8"
```

### Scenario 3: Emergency Stop Test

```bash
# Activate E-stop
ros2 topic pub /emergency_stop std_msgs/Bool "data: true"

# Release E-stop
ros2 topic pub /emergency_stop std_msgs/Bool "data: false"
```

## 📝 Build Summary

After successful build, you should have:

```
install/
├── arm_control/
│   ├── lib/arm_control/
│   │   ├── gripper_controller
│   │   ├── action_servers
│   │   └── safety_monitor
│   ├── share/arm_control/
│   │   ├── action/
│   │   ├── msg/
│   │   ├── launch/
│   │   └── config/
├── moveit_config_rover/
│   ├── lib/moveit_config_rover/
│   │   ├── moveit_arm_controller.py
│   │   └── teleop_arm.py
│   ├── share/moveit_config_rover/
│   │   ├── config/
│   │   ├── launch/
│   │   ├── urdf/
│   │   └── rviz/
└── rover_description/
    └── share/rover_description/
        └── meshes/
```

## 🎉 Success Criteria

Your system is ready when:

✅ All packages build without errors  
✅ `ros2 interface list` shows arm_control actions  
✅ Demo launch shows "You can start planning now!"  
✅ RViz displays the arm model correctly  
✅ Named poses work in RViz Motion Planning plugin  
✅ Action servers respond to goals  
✅ Gripper commands work  
✅ Joint states are published  

## 🆘 Still Having Issues?

1. **Clean everything:**
   ```bash
   cd /home/rover/workspaces/rover
   rm -rf build/ install/ log/
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Check ROS2 installation:**
   ```bash
   printenv | grep ROS
   # Should show ROS_DISTRO=humble
   ```

3. **Verify all dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Check for conflicting packages:**
   ```bash
   # Make sure old arm interfaces don't conflict
   ros2 interface list | grep -i "ik"
   ```

## 🎯 Complete Testing Workflow

### Phase 1: Simulation Testing (DO THIS FIRST) ✅

```bash
# Terminal 1: Launch demo
cd /home/rover/workspaces/rover
source install/setup.bash
ros2 launch moveit_config_rover demo.launch.py
```

**Test in RViz:**
1. Verify arm model displays correctly
2. Test all named poses (home, stow, es_ready, etc.)
3. Plan trajectories between poses
4. Check collision detection
5. Verify joint movements are smooth

**Expected time:** 10-15 minutes  
**Risk:** Zero - completely safe  
**Catches:** 90% of bugs (URDF errors, planning failures, limit violations)

---

### Phase 2: Hardware Interface Check (Offline) ✅

**Verify topics without motors running:**

```bash
# Keep demo running from Phase 1

# Terminal 2: Check hardware interface topics exist
ros2 topic list | grep -E "motor|arm_target|get_arm"

# Should see:
# /arm_target_motor_positions  (publishes motor commands)
# /get_arm_position           (expects encoder feedback)

# Check message format
ros2 topic info /arm_target_motor_positions
# Type: std_msgs/msg/Int32MultiArray

# Verify gear ratios in code
cat /home/rover/workspaces/rover/src/moveit/moveit_config_rover/hardware/rover_arm_hardware.py | grep -A5 "GEAR_RATIOS"
```

**This verifies:**
- ✅ Hardware interface configured correctly
- ✅ Topic names match old system
- ✅ Message formats compatible

---

### Phase 3: Real Hardware Testing (⚠️ CAUTION) ⚠️

**Prerequisites:**
- ✅ Phase 1 complete (simulation works)
- ✅ Phase 2 complete (interface verified)
- ✅ E-stop within reach
- ✅ Clear workspace around arm
- ✅ Motors powered ON
- ✅ Someone spotting the arm

**Start with ONE joint:**

```bash
# Terminal 1: Launch with real hardware
cd /home/rover/workspaces/rover
source install/setup.bash
ros2 launch moveit_config_rover demo.launch.py use_fake_hardware:=false

# Terminal 2: Monitor motor commands
ros2 topic echo /arm_target_motor_positions

# Terminal 3: Monitor encoder feedback
ros2 topic echo /get_arm_position
```

**Testing sequence:**
1. **Start small:** Move just base joint first
2. **Verify direction:** Check encoder values increase/decrease correctly
3. **Check limits:** Test joint stops at configured limits
4. **One joint at a time:** Add joints incrementally
5. **Test named pose:** Only after individual joints work

**Emergency procedures:**
- Press E-stop immediately if unexpected movement
- Check encoder feedback matches expected values
- Verify gear ratios if movement is wrong speed/direction

---

### Phase 4: Mission Testing ✅

**Once hardware works:**

```bash
# Terminal 1: MoveIt with real hardware
ros2 launch moveit_config_rover demo.launch.py use_fake_hardware:=false

# Terminal 2: Mission control layer
ros2 launch arm_control arm_control.launch.py

# Terminal 3: Test actions
ros2 action send_goal /arm/go_to_named_pose arm_control/action/GoToNamedPose \
  "{pose_name: 'home', velocity_scaling: 0.1}"
```

---

## 📋 Testing Checklist

Before moving to next phase:

### Phase 1 Complete When:
- [ ] RViz shows arm model
- [ ] All named poses plan successfully
- [ ] Trajectories execute in simulation
- [ ] No console errors
- [ ] Collision detection works

### Phase 2 Complete When:
- [ ] Hardware topics exist
- [ ] Message formats correct
- [ ] Gear ratios verified

### Phase 3 Complete When:
- [ ] Each joint moves correctly
- [ ] Encoder feedback accurate
- [ ] Limits enforced
- [ ] E-stop tested
- [ ] Named poses work on hardware

### Phase 4 Complete When:
- [ ] Action servers respond
- [ ] Gripper control works
- [ ] Pick and place functional
- [ ] Ready for missions!

---

**Next Steps After Successful Build:**
1. ✅ Test in RViz with mock hardware (Phase 1)
2. ✅ Verify interface offline (Phase 2)
3. ⚠️ Calibrate with real hardware (Phase 3)
4. 🚀 Write mission-specific scripts (Phase 4)
5. 🏆 Practice for URC!

