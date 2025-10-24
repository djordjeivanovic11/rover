# GNSS Launch

GPS driver and monitoring system for ZED-F9P RTK GNSS module.

## What's Inside

**GPS Driver** - ublox_gps_node configured for ZED-F9P in rover mode  
**Health Monitor** - Tracks signal quality, satellite count, fix status  
**Validator** - Verifies GPS stability and fusion readiness  

**Topics Published:**
- `/gps/fix` - GPS position (sensor_msgs/NavSatFix)
- `/gps/velocity` - GPS velocity
- `/gnss/health_status` - "excellent", "good", "fair", "poor", "no_fix"
- `/gnss/signal_quality` - Quality score 0.0-1.0
- `/gnss/satellite_count` - Number of satellites
- `/gnss/ready_for_fusion` - Ready for robot_localization fusion

## Quick Start

```bash
# Launch GPS system
~/workspaces/rover/src/perception/gnss_launch/launch_gps.sh

# Or after sourcing workspaces:
source ~/workspaces/ros2-ublox-zedf9p/install/setup.bash
source ~/workspaces/rover/install/setup.bash
ros2 launch ~/workspaces/rover/install/gnss_launch/share/gnss_launch/launch/gnss.launch.py

# Check topics
ros2 topic list | grep gps

# Monitor GPS fix
ros2 topic echo /gps/fix

# Check health
ros2 topic echo /gnss/health_status
```

## What It's Used For

**Navigation** - Provides global position for waypoint navigation with Nav2  
**Localization** - Fuses with robot_localization for drift-free odometry  
**RTK Mode** - Supports centimeter-level accuracy with RTK base station  
**Mapping** - Geo-references maps with GPS coordinates  

## Integration

**With robot_localization:**
```bash
# Full localization stack (GPS + ZED + EKFs)
ros2 launch loc_fusion loc_fusion.launch.py
```

**With Nav2:**
- Provides `/gps/fix` for GPS waypoint navigation
- Converts GPS coordinates to map frame via navsat_transform
- Enables autonomous navigation to lat/lon coordinates

## Requirements

- ZED-F9P GPS module connected to `/dev/ttyACM0`
- Clear outdoor sky view for satellite lock
- ublox_gps driver (installed at `~/workspaces/ros2-ublox-zedf9p`)
- First fix takes 30-60 seconds (cold start)

## Configuration

GPS is configured for:
- **Rover mode** (tmode3=0, not base station)
- **Automotive model** (optimized for ground vehicles)
- **10Hz update rate** with 1Hz navigation rate
- **Auto fix mode** (GPS/GLONASS/Galileo/BeiDou)

## Status Indoors/Underground

✗ No GPS fix (expected - needs outdoor sky view)  
✓ Driver runs and creates topics  
✓ Health monitor shows "no_signal" or "no_fix"  
✓ Go outside for satellite lock  

## RTK Setup

For centimeter accuracy, use second ZED-F9P as base station:
1. Configure base in survey-in mode (u-center software)
2. Stream RTCM corrections to rover via radio or network
3. Rover achieves RTK FIX (1-2cm horizontal accuracy)

## Files

```
gnss_launch/
├── launch/
│   ├── gnss.launch.py         # Main launch file
│   ├── gnss_health_monitor.py # Health monitoring node
│   └── gnss_validator.py      # Signal validation node
├── launch_gps.sh              # Simple launch script
├── CMakeLists.txt
└── package.xml
```

## Troubleshooting

**No topics** - Source both workspaces (ublox + rover)  
**No fix indoors** - Normal, GPS needs outdoor sky view  
**Permission denied** - Add user to dialout group  
**Wrong device** - GPS is on `/dev/ttyACM0`, adjust in launch file if different  

