# Navigation Stack

Complete Nav2-based navigation for URC rover with GPS, SLAM, and obstacle avoidance.

## What's Inside

**nav2_launch** - Nav2 configuration optimized for outdoor navigation  
**nav2_teensy_bridge** - Motor control interface (cmd_vel → Teensy)  
**~~path_planner~~** - (Disabled) Custom D* Lite planner  
**~~gap-guidance~~** - (Disabled) Custom local planner  

## Architecture

```
PERCEPTION STACK                    NAVIGATION STACK                 HARDWARE
─────────────────────              ──────────────────────           ────────
┌─────────────────┐                ┌──────────────────┐             
│ gnss_launch     │──/gps/fix─────→│                  │             
│ loc_fusion      │──/odometry────→│    Nav2 Stack    │             
│ pointcloud_tools│──/scan────────→│                  │──/cmd_vel──→ Teensy
│ slam_launch     │──/map─────────→│  • Costmaps      │             Motors
└─────────────────┘                │  • Planners      │             
                                   │  • Controllers   │             
                                   └──────────────────┘             
```

## Quick Start

### Prerequisites
```bash
# Install Nav2 (one time)
sudo apt update
sudo apt install ros-humble-navigation2

# Build navigation packages
cd ~/workspaces/rover
colcon build --packages-select nav2_launch nav2_teensy_bridge
source install/setup.bash
```

### Launch Full System

**Terminal 1 - Perception Stack:**
```bash
~/workspaces/rover/src/perception/run_full_stack.sh
```

**Terminal 2 - Navigation Stack:**
```bash
ros2 launch nav2_launch complete_nav.launch.py teensy_port:=/dev/ttyACM1
```

**Terminal 3 - Visualization (optional):**
```bash
rviz2 -d ~/workspaces/rover/src/navigation/rviz/nav2_default_view.rviz
```

## Configuration

### Nav2 Parameters (`nav2_launch/config/nav2_params.yaml`)

Key settings optimized for outdoor rover:
- **Odometry**: Uses `/odometry/filtered` from `loc_fusion`
- **Obstacle detection**: Uses `/scan` from `pointcloud_tools`
- **Max velocities**: 0.6 m/s linear, 1.8 rad/s angular
- **Costmaps**: Rolling windows (no static map required)
- **Transform tolerance**: 1.0s (handles GPS delays)

### Teensy Bridge

Converts Nav2 `/cmd_vel` to Teensy serial protocol:
- **Format**: `x{int16}\n` and `y{int16}\n`
- **Safety**: Automatic timeout stop (1.0s)
- **Port**: Configurable (default `/dev/ttyACM1`)
- **Reconnection**: Automatic retry on disconnect

## Serial Port Configuration

⚠️ **Important:** GPS and Teensy must use different ports!

Check connected devices:
```bash
ls -l /dev/ttyACM*
ls -l /dev/serial/by-id/
```

Typical setup:
- `/dev/ttyACM0` → GPS (ZED-F9P)
- `/dev/ttyACM1` → Teensy (motors)

Override Teensy port:
```bash
ros2 launch nav2_launch complete_nav.launch.py teensy_port:=/dev/ttyACM1
```

## Testing Navigation

### 1. Verify Perception Topics
```bash
ros2 topic hz /odometry/filtered  # Should be ~50 Hz
ros2 topic hz /scan               # Should be ~10 Hz
ros2 topic echo /gps/fix --once   # Should show GPS data
```

### 2. Check TF Tree
```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf - verify: map → odom → base_link
```

### 3. Send Test Goal in RViz
1. Open RViz: `rviz2`
2. Add displays:
   - RobotModel → Set `base_link`
   - LaserScan → Topic: `/scan`
   - Map → Topic: `/global_costmap/costmap`
   - Path → Topic: `/plan`
3. Click "2D Nav Goal" and set destination
4. Rover should plan path and navigate

### 4. Monitor Navigation
```bash
# Watch command velocities
ros2 topic echo /cmd_vel

# Check planner status
ros2 topic echo /plan

# Monitor costmaps
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
```

## Custom Planners (Disabled)

The `path_planner` (D* Lite) and `gap-guidance` packages are disabled in favor of Nav2's built-in planners:

**Why disabled:**
- Nav2 planners are well-tested and robust
- Parallel systems caused integration issues
- D* Lite can be re-enabled later as Nav2 plugin if needed

**To re-enable (advanced):**
1. Convert to Nav2 planner/controller plugins
2. Register in `nav2_params.yaml`
3. Test thoroughly before field use

## Outdoor Navigation Tips

### GPS Waypoint Navigation
```python
# Send GPS waypoint (lat, lon) to Nav2
# TODO: Implement GPS→map coordinate conversion
# Use navsat_transform output: /odom/gps
```

### Terrain Challenges
- **Rough terrain**: Increase `inflation_radius` in costmaps
- **Tall obstacles**: Adjust `max_obstacle_height` in obstacle layers
- **GPS dropouts**: Nav2 continues with local odometry (/odometry/filtered)
- **Steep slopes**: Reduce max velocities for safety

### Performance Tuning
```yaml
# nav2_params.yaml - Controller settings
max_vel_x: 0.6        # Reduce for rough terrain
inflation_radius: 0.55 # Increase for wider clearance
transform_tolerance: 1.0 # Increase for GPS delays
```

## Troubleshooting

**Nav2 won't start:**
```bash
# Check if Nav2 is installed
ros2 pkg list | grep nav2

# If missing, install
sudo apt install ros-humble-navigation2
```

**No cmd_vel output:**
```bash
# Check if planner has a goal
ros2 topic echo /goal_pose

# Check if path is generated
ros2 topic hz /plan

# Verify controller is running
ros2 node list | grep controller_server
```

**Teensy not responding:**
```bash
# Check serial port
ls -l /dev/ttyACM*

# Check Teensy bridge logs
ros2 node info /nav2_teensy_bridge

# Test manual command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

**Costmap errors:**
```bash
# Verify /scan topic
ros2 topic hz /scan

# Check TF transforms
ros2 run tf2_ros tf2_echo map base_link
```

**GPS not fusing:**
```bash
# Check GPS fix
ros2 topic echo /gps/fix --once

# Verify navsat_transform is running
ros2 node list | grep navsat

# Check fusion output
ros2 topic hz /odometry/global
```

## Files Structure

```
navigation/
├── README.md                      # This file
├── global_nav/
│   └── nav2_launch/
│       ├── config/
│       │   ├── nav2_params.yaml   # Main Nav2 configuration
│       │   ├── costmap_global.yaml
│       │   └── costmap_local.yaml
│       └── launch/
│           ├── nav2_bringup.launch.py    # Basic Nav2
│           └── complete_nav.launch.py    # Full stack
└── nav2_teensy_bridge/
    ├── nav2_teensy_bridge/
    │   └── nav2_teensy_bridge.py  # Motor control bridge
    ├── launch/
    │   └── teensy_bridge.launch.py
    ├── package.xml
    └── setup.py
```

## Integration with Perception

Nav2 requires these topics from perception stack:

| Topic | Source | Usage |
|-------|--------|-------|
| `/odometry/filtered` | loc_fusion | Local odometry (odom→base_link) |
| `/scan` | pointcloud_tools | Obstacle detection |
| `/gps/fix` | gnss_launch | GPS waypoints (optional) |
| `/map` | slam_launch | Global map (optional) |
| TF: `map→odom→base_link` | loc_fusion | Coordinate transforms |

Make sure perception stack is running **before** launching Nav2!

## Next Steps

1. ✅ Install Nav2
2. ✅ Configure for perception stack
3. ✅ Fix serial ports
4. ✅ Update costmaps
5. ⏳ Test basic navigation
6. ⏳ Outdoor GPS testing
7. ⏳ Parameter tuning
8. ⏳ Mission integration

