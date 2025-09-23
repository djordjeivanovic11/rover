// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arm_control:msg/Fault.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__MSG__DETAIL__FAULT__STRUCT_H_
#define ARM_CONTROL__MSG__DETAIL__FAULT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'INFO'.
/**
  * Fault severity levels
 */
enum
{
  arm_control__msg__Fault__INFO = 0
};

/// Constant 'WARNING'.
enum
{
  arm_control__msg__Fault__WARNING = 1
};

/// Constant 'ERROR'.
enum
{
  arm_control__msg__Fault__ERROR = 2
};

/// Constant 'CRITICAL'.
enum
{
  arm_control__msg__Fault__CRITICAL = 3
};

/// Constant 'EMERGENCY'.
enum
{
  arm_control__msg__Fault__EMERGENCY = 4
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'fault_category'
// Member 'fault_description'
// Member 'component_name'
// Member 'joint_name'
// Member 'fault_units'
// Member 'reference_frame'
// Member 'recovery_action'
// Member 'current_action'
// Member 'related_faults'
// Member 'recommended_action'
// Member 'troubleshooting_info'
#include "rosidl_runtime_c/string.h"
// Member 'fault_location'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'time_since_last'
#include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in msg/Fault in the package arm_control.
/**
  * =============================================================================
  * FAULT MESSAGE
  * =============================================================================
  * Structured fault reporting for safety monitor and error recovery systems
  * Provides detailed fault information for logging and automated response
  * =============================================================================
 */
typedef struct arm_control__msg__Fault
{
  /// Header for timestamp and frame information
  std_msgs__msg__Header header;
  uint8_t severity;
  /// Fault code (matches safety_params.yaml fault_codes)
  uint32_t fault_code;
  /// Fault category for filtering and handling
  /// "JOINT", "FORCE_TORQUE", "ESTOP", "COMMUNICATION", "WORKSPACE", "SYSTEM"
  rosidl_runtime_c__String fault_category;
  /// Descriptive information
  /// Human-readable fault description
  rosidl_runtime_c__String fault_description;
  /// Component that generated the fault
  rosidl_runtime_c__String component_name;
  /// Specific joint name (if applicable)
  rosidl_runtime_c__String joint_name;
  /// Fault data
  /// Measured value that caused fault
  double fault_value;
  /// Threshold that was exceeded
  double fault_threshold;
  /// Units for fault_value and fault_threshold
  rosidl_runtime_c__String fault_units;
  /// Spatial information
  /// 3D location of fault (if spatial)
  geometry_msgs__msg__Point fault_location;
  /// Reference frame for fault_location
  rosidl_runtime_c__String reference_frame;
  /// Recovery information
  /// True if fault can be automatically recovered
  bool auto_recoverable;
  /// True if recovery was attempted
  bool recovery_attempted;
  /// True if recovery was successful
  bool recovery_successful;
  /// Description of recovery action taken
  rosidl_runtime_c__String recovery_action;
  /// System state at fault time
  /// System state when fault occurred (from ArmStatus)
  uint8_t system_state;
  /// Action being executed when fault occurred
  rosidl_runtime_c__String current_action;
  /// Action progress when fault occurred
  float action_progress;
  /// Additional context
  /// IDs of related/cascading faults
  rosidl_runtime_c__String__Sequence related_faults;
  /// Number of times this fault has occurred
  uint32_t occurrence_count;
  /// Time since last occurrence of this fault type
  builtin_interfaces__msg__Duration time_since_last;
  /// Operator guidance
  /// Recommended operator action
  rosidl_runtime_c__String recommended_action;
  /// Level of intervention needed
  uint8_t operator_intervention_required;
  /// Additional troubleshooting information
  rosidl_runtime_c__String troubleshooting_info;
} arm_control__msg__Fault;

// Struct for a sequence of arm_control__msg__Fault.
typedef struct arm_control__msg__Fault__Sequence
{
  arm_control__msg__Fault * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__msg__Fault__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARM_CONTROL__MSG__DETAIL__FAULT__STRUCT_H_
