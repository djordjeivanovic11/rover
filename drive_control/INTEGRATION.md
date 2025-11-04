# Integration Guide

## With Navigation Stack

The `drive_control` package is automatically launched by the navigation stack.

### Usage

```bash
# Full navigation stack (includes drive_control)
ros2 launch nav2_launch complete_nav.launch.py

# With custom parameters
ros2 launch nav2_launch complete_nav.launch.py \
  teensy_port:=/dev/ttyACM0 \
  track_width:=0.45 \
  wheel_radius:=0.110
```

### What Happens

```
Nav2 → /cmd_vel → twist_to_wheels → /cmd_wheels → wheel_bridge → Teensy
```

1. Nav2 publishes desired velocities to `/cmd_vel`
2. `twist_to_wheels` converts using differential drive kinematics
3. `wheel_bridge` converts m/s to RPM and sends to Teensy via Serial
4. Teensy firmware applies RPM to VESC motor controllers

## Standalone Usage

For testing or manual control without Nav2:

```bash
# Launch drive control only
ros2 launch drive_control wheel_drive.launch.py

# Send manual commands
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" -r 10
```

## With Teleoperation

Use `twist_mux` to blend autonomous and manual control:

```bash
# Run twist_mux
ros2 run twist_mux twist_mux --ros-args \
  --params-file src/drive_control/config/twist_mux_config.yaml

# Configure sources to publish to:
# - /cmd_vel/nav  (Nav2, priority 50)
# - /cmd_vel/joy  (Joystick, priority 80)
# - /cmd_vel/stop (E-stop, priority 100)
```

## Monitoring

```bash
# Check nodes running
ros2 node list | grep -E 'twist_to_wheels|wheel_bridge'

# Monitor commands
ros2 topic echo /cmd_vel        # From Nav2
ros2 topic echo /cmd_wheels     # To Teensy

# Check parameters
ros2 param list /twist_to_wheels
ros2 param list /wheel_bridge
```

## Troubleshooting

**No wheel commands:**
```bash
# Is Nav2 publishing?
ros2 topic hz /cmd_vel

# Are nodes running?
ros2 node list

# Check logs
ros2 node info /wheel_bridge
```

**Wrong speeds:**
- Verify `track_width` and `wheel_radius` are accurate
- Check motor inversions (`invert_left`, `invert_right`)
- Test with known distances

**Firmware not responding:**
```bash
# Test directly
screen /dev/ttyACM1 115200
# Type: L1000 R1000

# Check if Teensy is connected
ls -l /dev/ttyACM*
```

