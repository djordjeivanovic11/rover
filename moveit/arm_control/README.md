# ARM CONTROL

**Runtime control layer for URC rover arm - bridges MoveIt planning to hardware execution**

⚠️ **CRITICAL: DO NOT RUN WITH REAL HARDWARE UNTIL ALL COMPONENTS TESTED** ⚠️

## Overview

Arm Control is the rover's high-reliability execution layer that transforms MoveIt's collision-free plans into real torque, current, and motion on the URC field. It provides a complete manipulation subsystem with mission-critical safety monitoring and structured error reporting.

### Key Features

- **250 Hz Hardware Interface**: Real-time position/velocity control with soft limit enforcement
- **<150ms Safety Response**: Comprehensive monitoring with immediate fault response
- **Latency-Aware Trajectory Execution**: Streams goals with real-time tracking error monitoring
- **Force-Based Gripper Control**: Adaptive grasping with slip detection and grasp verification
- **Hot Tool Swapping**: Dynamic URDF/SRDF reloading without system restart
- **Mission Integration**: Three atomic actions for behavior tree integration

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Behavior Tree  │    │   MoveIt Plan   │    │  Manual Control │
│    Actions      │    │   Execution     │    │    Interface    │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          └──────────────────────┼──────────────────────┘
                                 │
          ┌─────────────────────────────────────────────────┐
          │              ACTION SERVERS                     │
          │  • GoToNamedPose  • PickAndPlace  • ToolChange │
          └─────────────────────┬───────────────────────────┘
                                │
          ┌─────────────────────────────────────────────────┐
          │            TRAJECTORY EXECUTOR                  │
          │     Streams goals + monitors execution          │
          └─────────────────────┬───────────────────────────┘
                                │
  ┌─────────────────┐          │          ┌─────────────────┐
  │ GRIPPER CONTROL │          │          │  TOOL MANAGER   │
  │ Force feedback  │          │          │ Hot URDF reload │
  │ Grasp security  │          │          │ Coupling control│
  └─────────┬───────┘          │          └─────────┬───────┘
            │                  │                    │
            └──────────────────┼────────────────────┘
                               │
          ┌─────────────────────────────────────────────────┐
          │          HARDWARE INTERFACE (250 Hz)            │
          │     Position/Velocity commands + Sensor data    │
          └─────────────────────┬───────────────────────────┘
                                │
          ┌─────────────────────────────────────────────────┐
          │             SAFETY MONITOR (<150ms)             │
          │   Current/Temp/Force/E-stop + Fault Response    │
          └─────────────────────────────────────────────────┘
```

## Components

### Hardware Interface (`hardware_interface.py`)
- **Purpose**: 250 Hz control loop bridging ROS 2 Control to hardware
- **Features**: Mock mode for testing, soft limit enforcement, sensor monitoring
- **Outputs**: Joint commands, current/temperature data, E-stop status

### Safety Monitor (`safety_monitor.py`)  
- **Purpose**: <150ms fault detection and emergency response
- **Monitors**: Joint limits, currents, temperatures, F/T sensors, communication
- **Outputs**: Fault messages, emergency stop signals, diagnostic status

### Trajectory Executor (`trajectory_executor.py`)
- **Purpose**: Latency-aware trajectory streaming with real-time monitoring
- **Features**: Tracking error detection, velocity limiting, immediate halt capability
- **Integration**: Receives from MoveIt, sends to hardware interface

### Gripper Controller (`gripper_controller.py`)
- **Purpose**: Force-based grasping with slip detection
- **Strategies**: Position-based, force-based, adaptive grasping
- **Outputs**: Grasp status (SECURE/SOFT/FAILED), force feedback

### Tool Manager (`tool_manager.py`)
- **Purpose**: Automated tool changes with kinematics updates
- **Features**: Coupling control, URDF/SRDF hot-reload, MoveIt reconfiguration
- **Safety**: Motion pausing, verification, structured error reporting

### Action Servers (`action_servers.py`)
- **Purpose**: High-level mission interface for behavior trees
- **Actions**: GoToNamedPose, PickAndPlace, ToolChange
- **Integration**: Wraps planning, execution, safety, and recovery

## Configuration

### Primary Configuration Files

- **`arm_params.yaml`**: Complete arm specification (inherited from arm_template)
- **`controller_config.yaml`**: Runtime controller settings with stubs
- **`safety_params.yaml`**: Safety limits, thresholds, and fault codes

### Key Parameters

```yaml
# Hardware interface
hardware_interface:
  update_rate: 250.0          # 250 Hz control loop
  mock_mode: true             # Start in mock mode
  
# Safety monitoring  
safety_monitor:
  fault_response_timeout: 0.15 # 150ms max response time
  update_rate: 100.0          # 100 Hz monitoring
  
# Named poses (from arm_params.yaml)
mission_poses:
  es_ready: [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
  sc_sample_collect: [0.0, -0.8, 0.0, -0.8, 0.0, 0.0]
  # ... more mission-specific poses
```

## Usage

### Launch System
```bash
# Full system launch (MOCK MODE - safe for testing)
ros2 launch arm_control arm_control.launch.py

# Individual components for testing
ros2 run arm_control safety_monitor
ros2 run arm_control hardware_interface  
ros2 run arm_control trajectory_executor
```

### Mission Integration
```python
# Behavior tree action calls
client = ActionClient(self, GoToNamedPose, '/arm/go_to_named_pose')

goal = GoToNamedPose.Goal()
goal.pose_name = 'es_ready'
goal.velocity_scaling = 0.1
goal.collision_checking = True

future = client.send_goal_async(goal)
```

### Direct Control
```python
# Pick and place operation
client = ActionClient(self, PickAndPlace, '/arm/pick_and_place')

goal = PickAndPlace.Goal()
goal.pick_pose = target_pose
goal.place_pose = destination_pose
goal.grasp_force = 50.0
goal.verify_grasp = True

future = client.send_goal_async(goal)
```

## Safety Features

### Multi-Layer Safety
1. **Hardware Interface**: Soft limit enforcement at 250 Hz
2. **Safety Monitor**: Comprehensive monitoring at 100 Hz  
3. **Trajectory Executor**: Real-time tracking error detection
4. **Action Servers**: High-level fault handling and recovery

### Emergency Response
- **E-Stop Integration**: Hardware and software emergency stops
- **Immediate Halt**: Motion stopping within 150ms of fault detection
- **Structured Faults**: Detailed error codes and recovery guidance
- **Automatic Recovery**: Self-recovery for non-critical faults

### Fault Codes
```yaml
# Joint faults (1xx)
JOINT_OVERCURRENT: 101
JOINT_OVERTEMP: 102
JOINT_POSITION_LIMIT: 103

# Force/Torque faults (2xx)  
FORCE_LIMIT_EXCEEDED: 201
TORQUE_LIMIT_EXCEEDED: 202

# E-stop faults (3xx)
ESTOP_ACTIVATED: 301
```

## Testing

### Phase-Gated Testing
Tests are organized by implementation phase to prevent running incomplete code:

```bash
# Phase 1: Safe component tests (✅ READY)
python3 -m pytest test/test_configuration.py -v
python3 -m pytest test/test_hardware_interface.py::TestMockHardware -v

# Phase 2: Integration tests (🚫 DO NOT RUN YET)
# python3 -m pytest test/test_integration.py -v

# Phase 3: System tests (🚫 DO NOT RUN YET)  
# python3 -m pytest test/test_system.py -v
```

See `test/README.md` for detailed testing procedures and safety notes.

## Development Workflow

### Implementation Phases

1. **✅ Phase 1 Complete**: Individual components with mock hardware
2. **🚧 Phase 2**: Integration testing and component communication  
3. **⏳ Phase 3**: Hardware-in-the-loop and system validation
4. **⏳ Phase 4**: Performance optimization and mission testing

### Adding Hardware Support

1. **Modify Hardware Interface**: Replace mock methods with real hardware protocols
2. **Update Configuration**: Set `mock_mode: false` in controller_config.yaml
3. **Test Incrementally**: Start with single joints, build up to full system
4. **Validate Safety**: Verify all fault detection and emergency stop functions

## Integration Points

### MoveIt Integration
- Receives planning results via FollowJointTrajectory action
- Updates URDF/SRDF for tool changes via parameter reload
- Publishes joint states for move_group state monitoring

### Behavior Tree Integration  
- Exposes three atomic actions: GoToNamedPose, PickAndPlace, ToolChange
- Provides structured feedback and error reporting
- Supports mission-specific named poses from configuration

### Hardware Integration
- CAN bus communication for motor control (TODO: implement)
- GPIO for emergency stop monitoring (TODO: implement)  
- Force/torque sensor integration (TODO: implement)
- Tool coupling mechanism control (TODO: implement)

## Monitoring and Diagnostics

### Status Topics
- `/arm/status`: Real-time arm status and metrics
- `/arm/safety_status`: Safety system diagnostics
- `/arm/faults`: Structured fault reporting

### Visualization
- RViz integration for real-time arm state
- Joint state publishing for transform tree
- Tool change visualization (planned)

## Future Enhancements

### Planned Features
- **Adaptive Control**: Dynamic parameter adjustment based on load
- **Predictive Maintenance**: Wear monitoring and component lifetime tracking
- **Advanced Grasping**: Multi-finger coordination and tactile feedback
- **Learning Integration**: Performance optimization through mission data

### Hardware Additions
- **Force/Torque Sensing**: 6-DOF F/T sensor at wrist
- **Vision Integration**: Eye-in-hand camera for fine manipulation
- **Tactile Feedback**: Pressure-sensitive gripper fingers
- **Tool Recognition**: Automatic tool type detection

## Contributing

### Code Standards
- Follow ROS 2 naming conventions
- Include comprehensive docstrings  
- Add safety checks for all hardware interactions
- Write phase-gated tests for new features

### Safety Requirements
- All hardware interactions must include fault detection
- Motion commands must respect safety limits
- New features require safety analysis and testing
- Emergency stop must be testable and verifiable

## Support

**Documentation**: See individual component docstrings and `test/README.md`
**Configuration**: Check YAML files in `config/` directory  
**Issues**: Review fault codes in `safety_params.yaml`
**Team Contact**: urc-arm-team@rover.com

---

**⚠️ SAFETY REMINDER**: This system controls a physical robotic arm. Always verify safety systems, use emergency stops, and test thoroughly before deployment. Start with mock mode and simulated data before attempting hardware operation. 