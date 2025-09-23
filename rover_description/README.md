# Rover Description Package

This package contains the URDF description, meshes, and configuration files for the Rover robot with rocker-bogie suspension and 5-DOF manipulator arm.

## Robot Description

The rover features:
- **6-wheel rocker-bogie suspension system** for enhanced mobility over rough terrain
- **5-DOF robotic manipulator arm** for object manipulation  
- **Detailed 3D meshes** exported from SolidWorks
- **Full ROS2 and Ignition Gazebo simulation support** with physics properties and materials

## Package Contents

- `urdf/rover.urdf` - Main robot description file (ROS2 compatible)
- `meshes/` - STL mesh files for all robot components (16 files)
- `launch/` - ROS2 Python launch files for visualization and simulation
- `config/` - Configuration files including RViz setup and joint names

## Installation Requirements

### Required ROS2 Packages
```bash
# Core ROS2 packages (usually included with ros-humble-desktop)
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-xacro

# For Gazebo simulation support
sudo apt install ros-humble-ros-ign-gazebo
sudo apt install ros-humble-ros-ign-gazebo-demos  
sudo apt install ros-humble-ros-ign-bridge
sudo apt install ros-humble-ros-ign-image
```

### System Requirements
- **ROS2 Humble** (tested and working)
- **Ignition Gazebo 6.16.0+** (for simulation)
- **Ubuntu 22.04 Jammy** (recommended)

## Usage

### Visualization in RViz2

To visualize the robot in RViz2 with joint controls:

```bash
ros2 launch rover_description display.launch.py
```

This command will:
- Start the robot state publisher with the rover URDF
- Launch joint state publisher GUI for manual joint control
- Open RViz2 with the pre-configured `rover.rviz` setup

**Alternative manual launch:**
```bash
# If you prefer to launch RViz2 manually with the rover configuration
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix rover_description)/share/rover_description/config/rover.rviz
```

### Gazebo Simulation

To launch the robot in Ignition Gazebo simulation:

```bash
ros2 launch rover_description gazebo.launch.py
```

For headless simulation (no GUI):
```bash
ros2 launch rover_description gazebo.launch.py ign_args:='--headless-rendering -v 4'
```

## Robot Structure

### Suspension System
- **Chassis**: Main body (5.96 kg)
- **Rockers**: Left/Right rocker arms (30-31 kg each)
- **Bogies**: Left/Right bogie assemblies (3.35 kg each)
- **Wheels**: 6 wheels total - Front, Mid, Rear on each side (0.62 kg each)

### Manipulator Arm
- **Arm-Base**: Base joint (1.22 kg)
- **Arm-Stage1**: First arm segment (2.06 kg)
- **Arm-Stage2**: Second arm segment (1.11 kg)
- **Arm-Wrist**: Wrist assembly (0.50 kg)
- **Arm-Manip**: End effector (0.46 kg)

### Joint Configuration
- **15 continuous joints** total
- **Suspension joints**: RR-Rev, RL-Rev, BR-Rev, BL-Rev
- **Wheel joints**: WFR/WFL, WMR/WML, WRR/WRL-Rev
- **Arm joints**: AB-Rev, AS1-Rev, AS2-Rev, AW-Rev, AM-Rev

## Dependencies

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz`
- `gazebo_ros`
- `gazebo_ros_control`
- `xacro`

## Learning Resources

Essential resources for working with robot description files, RViz visualization, and Gazebo simulation:

1. **[ROS2 URDF Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)** - Official ROS2 URDF documentation and robot description fundamentals
2. **[RViz2 User Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)** - Complete RViz2 tutorial for robot visualization
3. **[Ignition Gazebo Tutorials](https://gazebosim.org/docs/garden/tutorials)** - Official Ignition Gazebo documentation for simulation
4. **[ROS2-Gazebo Integration](https://gazebosim.org/docs/dome/tutorials/)** - ROS2 with Ignition Gazebo integration guide

## Package History

This URDF was originally exported from SolidWorks using the SW2URDF plugin and has been enhanced with:
- ✅ **ROS2 Compatibility**: Converted from ROS1 to ROS2 format
- ✅ **Proper Package Naming**: Following ROS conventions
- ✅ **Gazebo Integration**: Full simulation support with physics properties
- ✅ **Material Definitions**: Realistic rendering in simulation
- ✅ **Launch System**: Complete ROS2 Python launch files
- ✅ **Documentation**: Comprehensive usage and learning resources
