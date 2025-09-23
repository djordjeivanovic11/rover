# Arduino Uno Drive System Setup Guide

A minimal, ultra-simple rover drive system using Arduino Uno with PPM servo outputs to VESCs.

## System Overview

```
┌─────────────────┐    ASCII Serial    ┌──────────────────┐
│   Jetson Nano   │◄──────────────────►│   Arduino Uno    │
│  (Drive Bridge) │   "L:0.5 R:-0.3"   │  (PPM Generator) │
└─────────────────┘                    └──────────────────┘
         │                                       │
         ▼                                       ▼
┌─────────────────┐                    ┌──────────────────┐
│   /cmd_vel      │                    │   6x PPM Servo   │
│   Teleop Joy    │                    │   Outputs        │
│   (20 Hz)       │                    │   (1000-2000µs)  │
└─────────────────┘                    └──────────────────┘
                                                │
                                                ▼
                                       ┌──────────────────┐
                                       │   6x VESC ESCs   │
                                       │   (PPM Mode)     │
                                       └──────────────────┘
```

## Hardware Requirements

### Arduino Uno
- Arduino Uno R3 or compatible
- USB cable for Jetson connection
- Breadboard or PCB for connections

### VESCs
- 6x VESC motor controllers
- PPM servo cables (3-wire: signal, +5V, GND)
- Motors and wheels

### Safety
- Status LED (built-in LED 13 used)

## Wiring Diagram

### Arduino Uno Pin Assignments
```
Pin 0  - USB Serial RX (reserved)
Pin 1  - USB Serial TX (reserved)
Pin 2  - Left Front VESC PPM
Pin 3  - Left Middle VESC PPM
Pin 4  - Left Rear VESC PPM
Pin 5  - Right Front VESC PPM
Pin 6  - Right Middle VESC PPM
Pin 7  - Right Rear VESC PPM
Pin 13 - Status LED (built-in)
```

### VESC Connections
Each VESC needs 3 wires from Arduino:
- **Signal**: Arduino digital pin (2-7)
- **+5V**: Arduino 5V pin (shared)
- **GND**: Arduino GND pin (shared)

## VESC Configuration

Each VESC must be configured for PPM input:

### VESC Tool Settings
1. **App Configuration → PPM**
2. **Control Type**: Current No Reverse with Brake
3. **PPM Settings**:
   - **Pulse Width Center**: 1500 µs
   - **Pulse Width Min**: 1000 µs  
   - **Pulse Width Max**: 2000 µs
   - **Median Filter**: 25
   - **Safe Start**: Disabled
   - **Timeout**: 300 ms (VESC internal safety)
   - **Timeout Brake Current**: 5.0 A

### Motor Direction
- **Left side VESCs**: Normal direction
- **Right side VESCs**: Inverted direction (or swap motor wires)

## Software Setup

### 1. Arduino Code Upload

1. Install Arduino IDE
2. Open `/serial_control/serial_control.ino`
3. Verify pin assignments match your wiring
4. Upload to Arduino Uno

### 2. ROS 2 Package Build

```bash
cd ~/ros2_ws/src
# Ensure rover_control package is present
cd ~/ros2_ws
colcon build --packages-select rover_control
source install/setup.bash
```

### 3. Install Dependencies

```bash
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
pip3 install pyserial
```

## Usage

### Basic Teleop Operation

```bash
# Launch the complete Uno drive system
ros2 launch rover_control uno_drive.launch.py

# With custom parameters
ros2 launch rover_control uno_drive.launch.py \
    serial_port:=/dev/ttyUSB0 \
    wheel_separation:=0.5 \
    max_velocity:=1.5
```

### Manual Testing

Test the drive bridge directly:
```bash
# Start drive bridge only
ros2 run rover_control drive_bridge --ros-args -p serial_port:=/dev/ttyACM0

# Send test commands
ros2 topic pub /cmd_vel geometry_msgs/Twist \
    "linear: {x: 0.5, y: 0.0, z: 0.0}
     angular: {x: 0.0, y: 0.0, z: 0.0}"
```

### Monitor Status

```bash
# Check connection status
ros2 topic echo /drive_bridge_connected

# View status messages
ros2 topic echo /drive_bridge_status

# Monitor Arduino serial output
screen /dev/ttyACM0 115200
```

## Safety Features

### VESC Internal Safety
- Each VESC has 300ms timeout - will brake if no PPM signal
- This provides basic safety if Arduino fails

### Software Limits
- Throttle values clamped to [-1.0, 1.0]
- PPM outputs limited to 1000-2000µs range
- Input validation on all serial commands

### Status LED
- Solid LED when system running
- Blinks during startup sequence

## Testing Procedure

### 1. Bench Test (Wheels Off Ground)

```bash
# 1. Upload Arduino code and check serial output
screen /dev/ttyACM0 115200
# Should see: "Rover Drive Bridge Ready"

# 2. Launch ROS system
ros2 launch rover_control uno_drive.launch.py

# 3. Test forward motion
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
    "linear: {x: 0.2, y: 0.0, z: 0.0}"

# 4. Test rotation
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
    "angular: {z: 0.5}"

# 5. Test stop
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{}"
```

### 2. Safety Test

```bash
# 1. Start motion
ros2 topic pub /cmd_vel geometry_msgs/Twist \
    "linear: {x: 0.3, y: 0.0, z: 0.0}"

# 2. Unplug USB cable
# VESCs should stop within 0.3 seconds (VESC timeout)

# 3. Test emergency stop
# Ctrl+C the ROS node - motors should stop
```

### 3. Joystick Test

```bash
# 1. Connect joystick to Jetson
ls /dev/input/js*

# 2. Launch with teleop
ros2 launch rover_control uno_drive.launch.py enable_teleop:=true

# 3. Hold enable button (L1/LB) and drive with left stick
```

## Troubleshooting

### No Serial Connection
```bash
# Check available ports
ls /dev/tty*

# Check permissions
sudo usermod -a -G dialout $USER
# Log out and back in

# Test direct connection
screen /dev/ttyACM0 115200
```

### Motors Not Responding
1. Check VESC PPM configuration
2. Verify wiring connections
3. Test with VESC Tool PPM input display
4. Check motor direction settings

### Jerky Movement
1. Reduce max_velocity parameter
2. Check for loose connections
3. Verify VESC median filter setting (25)

## Performance Specifications

- **Command Rate**: 20 Hz (50ms period)
- **PPM Frequency**: 50 Hz (20ms period)
- **PPM Resolution**: 1µs (1000-2000µs range)
- **Serial Baud**: 115200 bps
- **Latency**: <100ms (Jetson to wheel response)
- **Safety**: VESC 300ms timeout

## Files Used

- `/serial_control/serial_control.ino` - Arduino Uno PPM controller
- `/rover_control/rover_control/drive_bridge.py` - Jetson serial bridge
- `/rover_control/launch/uno_drive.launch.py` - Launch file
- `/rover_control/setup.py` - Package configuration

## Pin Reference Card

Print and keep near rover:

```
┌─────────────────────────────────────┐
│        Arduino Uno Pin Reference    │
├─────────────────────────────────────┤
│ Pin 2: Left Front VESC PPM          │
│ Pin 3: Left Middle VESC PPM         │
│ Pin 4: Left Rear VESC PPM           │
│ Pin 5: Right Front VESC PPM         │
│ Pin 6: Right Middle VESC PPM        │
│ Pin 7: Right Rear VESC PPM          │
│ Pin 13: Status LED (solid=running)  │
│                                     │
│ PPM: 1500µs=stop, 1000-2000µs range│
│ Serial: 115200 baud, "L:x.x R:x.x"  │
│ Safety: VESC 300ms timeout only     │
└─────────────────────────────────────┘
``` 

Here's a simple explanation of each file involved in driving the rover with the Arduino Uno:

## Arduino Side

**`/workspaces/serial_control/serial_control.ino`** - This is the ultra-simple Arduino code that runs on the Uno. It listens for ASCII commands like "L:0.5 R:-0.3" over USB serial, parses the left and right throttle values, and converts them to PPM servo signals (1000-2000 microseconds) that get sent to pins 2-7 to control the 6 VESCs. The status LED on pin 13 stays solid when running and blinks during startup. There's no watchdog, no E-stop - just pure simplicity.

## ROS 2 Side

**`/rover_control/rover_control/drive_bridge.py`** - This Python node runs on the Jetson and acts as the translator between ROS and Arduino. It subscribes to `/cmd_vel` messages (standard ROS velocity commands), uses differential drive math to convert linear/angular velocity into left/right wheel speeds, normalizes them to throttle values (-1.0 to 1.0), and sends the simple ASCII format to Arduino over serial at 20Hz. It also publishes connection status so you know if the Arduino is responding.

**`/rover_control/launch/uno_drive.launch.py`** - This launch file starts everything you need for rover operation. It launches the drive_bridge node with configurable parameters (serial port, wheel separation, max speed), optionally starts the joystick nodes for teleop control, and connects them all together. Just run `ros2 launch rover_control uno_drive.launch.py` and you're driving.

**`/rover_control/setup.py`** - This is the Python package configuration that tells ROS 2 how to build and install the rover_control package. It defines the `drive_bridge` executable and includes the launch files and other package data. When you run `colcon build`, this file tells the system what to compile and where to put everything.

**`/rover_control/package.xml`** - This XML file defines the ROS 2 package dependencies and metadata. It tells the build system that this package needs `pyserial` for Arduino communication, `joy` and `teleop_twist_joy` for joystick support, plus the standard ROS message types. It's like a requirements.txt file but for ROS packages.

That's it! Five files total - the Arduino sketch does PPM output, the Python bridge converts ROS to serial, the launch file starts everything, and the two config files tell ROS how to build it all. Simple and reliable.


This architecture works for a **6-wheel rocker-bogie or differential drive rover** where one Arduino Uno acts as a simple PPM signal generator for multiple VESC motor controllers. The Arduino receives left/right throttle commands from the Jetson over USB serial, then outputs 6 synchronized PPM servo signals (pins 2-7) - three for the left side wheels (front, middle, rear) and three for the right side wheels. Each VESC is configured in PPM mode and controls one motor, with the VESCs handling all the complex motor control, current limiting, and safety timeouts (300ms) internally. This setup is perfect for rovers that need independent wheel control but want to keep the Arduino code minimal - the Uno just acts as a "PPM splitter" that takes two throttle values and fans them out to six identical motor controllers, while all the intelligence stays on the Jetson side with ROS 2.