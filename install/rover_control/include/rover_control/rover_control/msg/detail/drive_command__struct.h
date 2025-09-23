// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_control:msg/DriveCommand.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__STRUCT_H_
#define ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'control_mode'
// Member 'mission_mode'
// Member 'source'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/DriveCommand in the package rover_control.
/**
  * =============================================================================
  * DRIVE COMMAND MESSAGE
  * =============================================================================
  * Low-level drive command message for rover movement control.
  * Used internally by action servers and path followers.
  * =============================================================================
 */
typedef struct rover_control__msg__DriveCommand
{
  /// Header
  std_msgs__msg__Header header;
  /// Basic movement command
  /// Target linear and angular velocity
  geometry_msgs__msg__Twist velocity;
  /// Target acceleration (m/s²)
  float acceleration;
  /// Command duration (seconds, 0 = continuous)
  float duration;
  /// Control mode
  /// "velocity", "position", "trajectory"
  rosidl_runtime_c__String control_mode;
  /// True if command is relative to current state
  bool relative_command;
  /// Safety and limits
  /// Maximum allowed velocity (m/s)
  float max_velocity;
  /// Maximum allowed acceleration (m/s²)
  float max_acceleration;
  /// Apply mission-specific safety limits
  bool enforce_safety_limits;
  /// Mission mode for parameter selection
  rosidl_runtime_c__String mission_mode;
  /// Advanced control
  /// Path curvature (1/radius in m⁻¹)
  float curvature;
  /// Pure pursuit lookahead distance (m)
  float lookahead_distance;
  /// Enable wheel slip compensation
  bool use_slip_compensation;
  /// Execution parameters
  /// Command priority (higher = more important)
  int32_t priority;
  /// Command timeout (seconds)
  float timeout;
  /// Source of command (for logging)
  rosidl_runtime_c__String source;
} rover_control__msg__DriveCommand;

// Struct for a sequence of rover_control__msg__DriveCommand.
typedef struct rover_control__msg__DriveCommand__Sequence
{
  rover_control__msg__DriveCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__msg__DriveCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__STRUCT_H_
