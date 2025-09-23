// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arm_control:msg/ArmStatus.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__MSG__DETAIL__ARM_STATUS__STRUCT_H_
#define ARM_CONTROL__MSG__DETAIL__ARM_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'IDLE'.
/**
  * Current arm state enum
 */
enum
{
  arm_control__msg__ArmStatus__IDLE = 0
};

/// Constant 'PLANNING'.
enum
{
  arm_control__msg__ArmStatus__PLANNING = 1
};

/// Constant 'EXECUTING'.
enum
{
  arm_control__msg__ArmStatus__EXECUTING = 2
};

/// Constant 'TOOL_CHANGE'.
enum
{
  arm_control__msg__ArmStatus__TOOL_CHANGE = 3
};

/// Constant 'FAULT'.
enum
{
  arm_control__msg__ArmStatus__FAULT = 4
};

/// Constant 'EMERGENCY_STOP'.
enum
{
  arm_control__msg__ArmStatus__EMERGENCY_STOP = 5
};

/// Constant 'CALIBRATING'.
enum
{
  arm_control__msg__ArmStatus__CALIBRATING = 6
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'current_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'joint_state'
#include "sensor_msgs/msg/detail/joint_state__struct.h"
// Member 'current_tool_name'
// Member 'error_message'
// Member 'current_mission'
// Member 'current_action'
#include "rosidl_runtime_c/string.h"
// Member 'tcp_velocity'
// Member 'tcp_angular_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'current_wrench'
#include "geometry_msgs/msg/detail/wrench_stamped__struct.h"
// Member 'joint_temperatures'
// Member 'joint_currents'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/ArmStatus in the package arm_control.
/**
  * =============================================================================
  * ARM STATUS MESSAGE
  * =============================================================================
  * Comprehensive status message for dashboards, ROS bags, and judge telemetry
  * Published at regular intervals for system monitoring
  * =============================================================================
 */
typedef struct arm_control__msg__ArmStatus
{
  /// Header for timestamp and frame information
  std_msgs__msg__Header header;
  uint8_t state;
  /// Current end-effector pose
  geometry_msgs__msg__PoseStamped current_pose;
  /// Current joint state
  sensor_msgs__msg__JointState joint_state;
  /// Current tool information
  rosidl_runtime_c__String current_tool_name;
  bool tool_attached;
  /// Current gripper opening (meters)
  float gripper_opening;
  /// Current gripper force (Newtons)
  float gripper_force;
  /// Safety status
  /// Overall safety status
  bool safety_ok;
  /// Emergency stop status
  bool estop_active;
  /// Current error code (0 = no error)
  uint8_t error_code;
  /// Human-readable error description
  rosidl_runtime_c__String error_message;
  /// Motion status
  /// True if arm is currently moving
  bool in_motion;
  /// Current velocity magnitude (rad/s)
  float velocity_norm;
  /// TCP linear velocity (m/s)
  geometry_msgs__msg__Vector3 tcp_velocity;
  /// TCP angular velocity (rad/s)
  geometry_msgs__msg__Vector3 tcp_angular_velocity;
  /// Force/Torque status
  /// Current F/T at TCP
  geometry_msgs__msg__WrenchStamped current_wrench;
  /// True if force limiting is active
  bool force_limit_active;
  /// Temperature monitoring
  /// Joint temperatures (Celsius)
  rosidl_runtime_c__float__Sequence joint_temperatures;
  /// Highest joint temperature
  float max_temperature;
  /// True if any joint over warning temp
  bool temperature_warning;
  /// Current monitoring
  /// Joint currents (Amps)
  rosidl_runtime_c__float__Sequence joint_currents;
  /// Highest joint current
  float max_current;
  /// True if any joint over current limit
  bool current_warning;
  /// Workspace status
  /// True if current pose is within workspace
  bool workspace_valid;
  /// Distance to workspace boundary (meters)
  float workspace_margin;
  /// Mission context
  /// Current mission type (ES, SC, DM, etc.)
  rosidl_runtime_c__String current_mission;
  /// Current action being executed
  rosidl_runtime_c__String current_action;
  /// Action progress percentage
  float action_progress;
  /// System health
  /// System uptime (seconds)
  float system_uptime;
  /// Total commands executed
  uint32_t command_count;
  /// Total faults encountered
  uint32_t fault_count;
  /// Recent command success rate
  float success_rate;
} arm_control__msg__ArmStatus;

// Struct for a sequence of arm_control__msg__ArmStatus.
typedef struct arm_control__msg__ArmStatus__Sequence
{
  arm_control__msg__ArmStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__msg__ArmStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARM_CONTROL__MSG__DETAIL__ARM_STATUS__STRUCT_H_
