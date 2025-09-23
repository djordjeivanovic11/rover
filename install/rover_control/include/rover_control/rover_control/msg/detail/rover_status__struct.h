// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_control:msg/RoverStatus.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__MSG__DETAIL__ROVER_STATUS__STRUCT_H_
#define ROVER_CONTROL__MSG__DETAIL__ROVER_STATUS__STRUCT_H_

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
// Member 'current_action'
// Member 'mission_mode'
// Member 'fault_messages'
// Member 'competition_mode'
#include "rosidl_runtime_c/string.h"
// Member 'current_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'current_velocity'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'odometry'
#include "nav_msgs/msg/detail/odometry__struct.h"
// Member 'motor_currents'
// Member 'motor_temperatures'
// Member 'active_faults'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'acceleration'
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'sun_direction'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/RoverStatus in the package rover_control.
/**
  * =============================================================================
  * ROVER STATUS MESSAGE
  * =============================================================================
  * Comprehensive rover status message for telemetry, diagnostics, and
  * mission monitoring. Includes URC competition-specific fields.
  * =============================================================================
 */
typedef struct rover_control__msg__RoverStatus
{
  /// Header
  std_msgs__msg__Header header;
  /// Current rover state
  /// Current action being executed
  rosidl_runtime_c__String current_action;
  /// Current mission mode
  rosidl_runtime_c__String mission_mode;
  /// True if in autonomous mode
  bool is_autonomous;
  /// True if all safety checks pass
  bool safety_ok;
  /// True if emergency stop is active
  bool emergency_stop_active;
  /// Position and motion
  /// Current pose in map frame
  geometry_msgs__msg__PoseStamped current_pose;
  /// Current linear and angular velocity
  geometry_msgs__msg__Twist current_velocity;
  /// Full odometry information
  nav_msgs__msg__Odometry odometry;
  /// Mission progress
  /// Overall mission progress
  float mission_progress_percentage;
  /// Remaining autonomy timer (seconds)
  float autonomy_time_remaining;
  /// Number of waypoints completed
  int32_t waypoints_completed;
  /// Total waypoints in current mission
  int32_t total_waypoints;
  /// Hardware status
  /// Battery voltage (V)
  float battery_voltage;
  /// Battery current (A)
  float battery_current;
  /// State of charge
  float battery_soc;
  /// Battery temperature (C)
  float battery_temperature;
  /// Current draw for each motor (A)
  rosidl_runtime_c__float__Sequence motor_currents;
  /// Temperature for each motor (C)
  rosidl_runtime_c__float__Sequence motor_temperatures;
  /// Communication and performance
  /// Link quality
  float communication_quality;
  /// Current bandwidth usage (Mbps)
  float bandwidth_usage;
  /// CPU usage percentage
  float cpu_usage;
  /// Memory usage percentage
  float memory_usage;
  /// Safety and diagnostics
  /// Array of active fault codes
  rosidl_runtime_c__int32__Sequence active_faults;
  /// Human-readable fault descriptions
  rosidl_runtime_c__String__Sequence fault_messages;
  /// True if rover is stuck
  bool stuck_detected;
  /// Current wheel slip ratio
  float slip_ratio;
  /// True if collision detected
  bool collision_detected;
  /// Environmental
  /// Ambient temperature (C)
  float ambient_temperature;
  /// Internal temperature (C)
  float internal_temperature;
  /// Humidity percentage
  float humidity;
  /// Current acceleration (m/s²)
  geometry_msgs__msg__Vector3 acceleration;
  /// Angular velocity from IMU (rad/s)
  geometry_msgs__msg__Vector3 angular_velocity;
  /// URC-specific telemetry
  /// URC competition mode
  rosidl_runtime_c__String competition_mode;
  /// Estimated score for current task
  float judge_score_estimate;
  /// True if visual servoing is active
  bool visual_servoing_active;
  /// Number of tennis balls detected
  int32_t tennis_balls_detected;
  /// Sun direction vector (for navigation bias)
  geometry_msgs__msg__Point sun_direction;
} rover_control__msg__RoverStatus;

// Struct for a sequence of rover_control__msg__RoverStatus.
typedef struct rover_control__msg__RoverStatus__Sequence
{
  rover_control__msg__RoverStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__msg__RoverStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_CONTROL__MSG__DETAIL__ROVER_STATUS__STRUCT_H_
