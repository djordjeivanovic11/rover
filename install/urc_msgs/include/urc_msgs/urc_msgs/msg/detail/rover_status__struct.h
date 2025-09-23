// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from urc_msgs:msg/RoverStatus.idl
// generated code does not contain a copyright notice

#ifndef URC_MSGS__MSG__DETAIL__ROVER_STATUS__STRUCT_H_
#define URC_MSGS__MSG__DETAIL__ROVER_STATUS__STRUCT_H_

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
// Member 'current_mode'
#include "rosidl_runtime_c/string.h"
// Member 'current_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'current_velocity'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/RoverStatus in the package urc_msgs.
/**
  * Rover status message
 */
typedef struct urc_msgs__msg__RoverStatus
{
  std_msgs__msg__Header header;
  /// Battery information
  float battery_voltage;
  float battery_percentage;
  /// System status
  bool motors_enabled;
  bool emergency_stop;
  rosidl_runtime_c__String current_mode;
  /// Position information
  geometry_msgs__msg__Pose current_pose;
  geometry_msgs__msg__Twist current_velocity;
} urc_msgs__msg__RoverStatus;

// Struct for a sequence of urc_msgs__msg__RoverStatus.
typedef struct urc_msgs__msg__RoverStatus__Sequence
{
  urc_msgs__msg__RoverStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} urc_msgs__msg__RoverStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // URC_MSGS__MSG__DETAIL__ROVER_STATUS__STRUCT_H_
