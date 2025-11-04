# Drive Control

Motor control for URC rover with Nav2 integration.

## Overview

Converts Nav2 `/cmd_vel` commands to precise left/right wheel speeds for the Teensy motor controller.

```
Nav2 → /cmd_vel → twist_to_wheels → /cmd_wheels → wheel_bridge → Teensy → Motors
         Twist      (kinematics)     TankDrive      (UDP/Serial)   L/R RPM
```

## Quick Start

### 1. Build

```bash
cd ~/workspaces/rover
colcon build --packages-select drive_control
source install/setup.bash
```

### 2. Flash Firmware

Upload `drive_teensy_wheels.ino` to Teensy. Adds L/R wheel commands while keeping joystick compatibility.

### 3. Measure Robot

```bash
# Wheel radius (meters) - measure diameter and divide by 2
WHEEL_RADIUS=0.105

# Track width (meters) - center-to-center distance between wheels
TRACK_WIDTH=0.42
```

### 4. Launch

```bash
ros2 launch drive_control wheel_drive.launch.py
```

### 5. Test

```bash
# Send forward command
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}" -r 10
```

## Configuration

Edit `launch/wheel_drive.launch.py`:

```python
# Robot geometry (MEASURE THESE!)
TRACK_WIDTH_M = 0.42      # Distance between wheel centers
WHEEL_RADIUS_M = 0.105    # Wheel radius
MAX_WHEEL_SPEED = 2.0     # Max rim speed (m/s)
MAX_RPM = 15000           # VESC limit
```

### Transport

Serial (default - recommended):
```python
# In wheel_drive.launch.py:
TRANSPORT = 'serial'
SERIAL_PORT = '/dev/ttyACM1'  # NOT same as GPS!
```

UDP (if you have network bridge):
```python
# In wheel_drive.launch.py:
TRANSPORT = 'udp'
UDP_HOST = '192.168.0.10'
UDP_PORT = 3000
```

### Motor Direction

If motors spin backwards:
```python
# In wheel_drive.launch.py:
'invert_left': True,   # or
'invert_right': True,
```

## Integration with Navigation

Your Nav2 stack publishes `/cmd_vel` → this package converts it to wheel commands automatically.

Replace `nav2_teensy_bridge` in your navigation launch:

```python
# Instead of nav2_teensy_bridge, use:
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('drive_control'),
            'launch',
            'wheel_drive.launch.py'
        ])
    ])
)
```

## Firmware Protocol

Teensy receives ASCII commands via Serial (default) or UDP:

**Atomic (preferred):**
```
L<rpm> R<rpm>\r\n    # Both wheels in one message
```

**Individual (backwards compatible):**
```
L<rpm>\r\n    # Left wheels RPM
R<rpm>\r\n    # Right wheels RPM
```

Examples:
```
L1500 R1500\r\n   # Both forward 1500 RPM (atomic)
L-800 R800\r\n    # Turn left (atomic)
L0 R0\r\n         # Stop (atomic)
```

Legacy `x`/`y` joystick commands still work.

## Tuning

### Verify Geometry

**Test 1: Forward speed**
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}}" -r 10
# Measure time to travel 5 meters
# Should take 5 seconds if wheel_radius is correct
```

**Test 2: Rotation**
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 1.0}}" -r 10
# Should rotate in place without drifting
# If drifts, adjust track_width
```

### Performance

**Too jerky?**
```python
'max_rpm_step': 300,  # Decrease (slower accel)
```

**Too slow to respond?**
```python
'rate_hz': 50.0,      # Increase update rate
'max_rpm_step': 800,  # Increase (faster accel)
```

## Troubleshooting

**Motors don't move:**
```bash
# Check nodes running
ros2 node list | grep -E 'twist_to_wheels|wheel_bridge'

# Check topics
ros2 topic hz /cmd_vel
ros2 topic hz /cmd_wheels

# Test firmware directly
screen /dev/ttyACM1 115200
# Type: L1000 then R1000
```

**Wrong speed:**
- Remeasure wheel_radius accurately
- Test with known distance

**Wrong direction:**
- Set invert_left or invert_right to true
- Or swap L/R motor IDs in firmware

**Timeout warnings:**
- Check /cmd_vel publishing rate (needs 10+ Hz)
- Check serial connection
- Verify Teensy port is correct

## Files

```
drive_control/
├── README.md
├── package.xml
├── setup.py
├── drive_control/
│   ├── twist_to_wheels.py       # Converts Twist → wheel speeds
│   └── wheel_bridge.py           # Sends wheel speeds to Teensy
├── launch/
│   └── wheel_drive.launch.py    # Main launch file
└── drive_teensy_wheels.ino       # Teensy firmware
```

## How It Works

**twist_to_wheels.py** - Differential drive kinematics:
```python
v_left  = linear_vel - angular_vel * (track_width / 2)
v_right = linear_vel + angular_vel * (track_width / 2)
```

**wheel_bridge.py** - Converts m/s to RPM:
```python
rpm = (wheel_speed_mps / (2 * π * wheel_radius)) * 60
```

Then sends `L{rpm}` and `R{rpm}` to Teensy via UDP or Serial.

**Teensy firmware** - Applies RPM directly to VESC controllers:
```cpp
vesc.setRPM(rpm_left);   // Left motors (IDs 0, 3)
vesc.setRPM(rpm_right);  // Right motors (IDs 4, 6)
```

## Next Steps

1. ✅ Build package
2. ✅ Flash firmware  
3. ✅ Measure robot
4. ✅ Test motor control
5. ⏳ Integrate with Nav2
6. ⏳ Field test and tune
7. ⏳ Add odometry feedback
