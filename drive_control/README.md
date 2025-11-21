# Drive Control

**Converts Nav2 velocity commands to wheel RPM for differential drive rovers.**

## How It Works

```
/cmd_vel  →  twist_to_wheels  →  /cmd_wheels  →  wheel_bridge  →  /drive/left_rpm, /drive/right_rpm
(Twist)      (diff. drive)        (TankDrive)     (m/s→RPM)        (std_msgs/Int32)
                                                                              ↓
                                                                    Teensy micro-ROS firmware
                                                                              ↓
                                                                    VESCs → Motors
```

**Three-step pipeline:**

1. **twist_to_wheels** - Differential drive kinematics
   - Input: `/cmd_vel` (linear.x, angular.z)
   - Math: `v_left = v - ω*(track/2)`, `v_right = v + ω*(track/2)`
   - Output: `/cmd_wheels` (left_speed, right_speed in m/s)

2. **wheel_bridge** - Speed to RPM conversion
   - Input: `/cmd_wheels` (m/s)
   - Math: `rpm = (v / (2π * radius)) * 60`
   - Output: `/drive/left_rpm`, `/drive/right_rpm` (Int32)

3. **Firmware** (drive_firmware on Teensy)
   - Subscribes: `/drive/left_rpm`, `/drive/right_rpm`
   - Sends RPM to VESCs over CAN: IDs 0,2 (left) | 3,4 (right)

## Quick Start

```bash
# 1. Build
cd ~/workspaces/rover
colcon build --packages-select drive_control
source install/setup.bash

# 2. Launch (uses micro-ROS firmware by default)
ros2 launch drive_control wheel_drive.launch.py \
  track_width:=0.42 \
  wheel_radius:=0.105

# 3. Test
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}" -r 10
```

**Requirements:**
- Teensy running `drive_firmware` (micro-ROS) flashed and connected
- Measure your rover's track width (wheel center-to-center distance)
- Measure wheel radius (diameter / 2)

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `transport` | `ros` | Communication mode: `ros` (micro-ROS), `serial` (Arduino), or `udp` |
| `track_width` | `0.42` | Distance between wheel centers (meters) |
| `wheel_radius` | `0.105` | Wheel radius (meters) |
| `max_wheel_speed` | `2.0` | Max wheel rim speed (m/s) |
| `max_rpm` | `15000` | VESC motor RPM limit |
| `invert_left` | `False` | Reverse left motor direction |
| `invert_right` | `False` | Reverse right motor direction |

**Example:**
```bash
ros2 launch drive_control wheel_drive.launch.py \
  transport:=ros \
  track_width:=0.45 \
  wheel_radius:=0.110 \
  invert_left:=True
```

## Firmware Modes

### micro-ROS (Default - `transport:=ros`)
- **Firmware:** `drive_firmware` (micro-ROS on Teensy)
- **Topics:** Publishes to `/drive/left_rpm`, `/drive/right_rpm` (std_msgs/Int32)
- **VESCs:** IDs 0,2 (left) | 3,4 (right)
- **Pros:** Native ROS2, lower latency

### Arduino Serial (`transport:=serial`)
- **Firmware:** `drive_teensy_wheels.ino` (Arduino on Teensy)
- **Protocol:** ASCII over Serial: `L<rpm> R<rpm>\r\n`
- **VESCs:** IDs 0,3 (left) | 4,6 (right)
- **Pros:** Simpler, includes joystick mode

### UDP (`transport:=udp`)
- **Protocol:** Same ASCII as Serial, over network
- **Use case:** Remote testing or networked control

## Tuning & Testing

**Verify wheel_radius:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}}" -r 10
# Should travel 1 m/s. Measure actual speed and adjust radius.
```

**Verify track_width:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 1.0}}" -r 10
# Should rotate in place. If drifts, adjust track_width.
```

**Adjust acceleration:**
- Jerky motion? → Decrease `max_rpm_step` (default: 500)
- Too slow? → Increase `max_rpm_step` or `rate_hz` (default: 30 Hz)

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Motors don't move | Check nodes: `ros2 node list \| grep wheel`<br>Check topics: `ros2 topic hz /cmd_vel /cmd_wheels` |
| Wrong speed | Remeasure `wheel_radius` accurately |
| Wrong direction | Set `invert_left:=True` or `invert_right:=True` |
| Timeout warnings | Ensure `/cmd_vel` publishes at 10+ Hz |
| Connection errors | Check Teensy connected and micro-ROS agent running |

**Debug commands:**
```bash
# Monitor pipeline
ros2 topic echo /cmd_vel        # Nav2 input
ros2 topic echo /cmd_wheels     # Converted to tank drive
ros2 topic echo /drive/left_rpm # Output to firmware

# Check nodes
ros2 node list | grep -E 'twist_to_wheels|wheel_bridge'
```

## Package Structure

```
drive_control/
├── drive_control/
│   ├── twist_to_wheels.py       # Differential drive kinematics
│   └── wheel_bridge.py          # m/s → RPM conversion + transport
├── launch/
│   └── wheel_drive.launch.py   # Main launch file
└── drive_teensy_wheels.ino      # Optional Arduino firmware
```

## Integration with Nav2

Include in your navigation stack:

```python
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

Nav2 will automatically publish `/cmd_vel` → motors will respond.
