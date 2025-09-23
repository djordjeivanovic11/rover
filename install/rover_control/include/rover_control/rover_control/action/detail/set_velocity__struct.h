// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_control:action/SetVelocity.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__ACTION__DETAIL__SET_VELOCITY__STRUCT_H_
#define ROVER_CONTROL__ACTION__DETAIL__SET_VELOCITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_velocity'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'mission_mode'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/SetVelocity in the package rover_control.
typedef struct rover_control__action__SetVelocity_Goal
{
  /// GOAL
  /// Target linear and angular velocity
  geometry_msgs__msg__Twist target_velocity;
  /// Duration to maintain velocity (0 = indefinite)
  float duration;
  /// Maximum acceleration (m/s²)
  float acceleration_limit;
  /// Safety timeout (seconds)
  float timeout;
  /// Mission mode for safety limits
  rosidl_runtime_c__String mission_mode;
  /// Apply mission-specific velocity limits
  bool enforce_safety_limits;
  /// Gradually ramp to target velocity
  bool ramp_to_velocity;
} rover_control__action__SetVelocity_Goal;

// Struct for a sequence of rover_control__action__SetVelocity_Goal.
typedef struct rover_control__action__SetVelocity_Goal__Sequence
{
  rover_control__action__SetVelocity_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__SetVelocity_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'final_velocity'
// Member 'max_velocity_reached'
// already included above
// #include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in action/SetVelocity in the package rover_control.
typedef struct rover_control__action__SetVelocity_Result
{
  /// RESULT
  /// True if velocity command executed successfully
  bool success;
  /// Structured result code (0=SUCCESS, 1=CANCELLED, etc.)
  int32_t result_code;
  /// Human-readable error description
  rosidl_runtime_c__String error_message;
  /// Time velocity was maintained (seconds)
  float execution_time;
  /// Final velocity achieved
  geometry_msgs__msg__Twist final_velocity;
  /// Maximum velocity reached during execution
  geometry_msgs__msg__Twist max_velocity_reached;
  /// Distance traveled during execution (meters)
  float distance_traveled;
} rover_control__action__SetVelocity_Result;

// Struct for a sequence of rover_control__action__SetVelocity_Result.
typedef struct rover_control__action__SetVelocity_Result__Sequence
{
  rover_control__action__SetVelocity_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__SetVelocity_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_state'
// Member 'status_message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'current_velocity'
// already included above
// #include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in action/SetVelocity in the package rover_control.
typedef struct rover_control__action__SetVelocity_Feedback
{
  /// FEEDBACK
  /// Current execution state (RAMPING, MAINTAINING, STOPPING)
  rosidl_runtime_c__String current_state;
  /// Current rover velocity
  geometry_msgs__msg__Twist current_velocity;
  /// Time remaining (if duration specified)
  float time_remaining;
  /// Progress to target velocity
  float progress_percentage;
  /// Human-readable status update
  rosidl_runtime_c__String status_message;
  /// True if safety limits are constraining velocity
  bool safety_limit_active;
  /// Remaining autonomy timer (seconds)
  float autonomy_time_remaining;
} rover_control__action__SetVelocity_Feedback;

// Struct for a sequence of rover_control__action__SetVelocity_Feedback.
typedef struct rover_control__action__SetVelocity_Feedback__Sequence
{
  rover_control__action__SetVelocity_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__SetVelocity_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "rover_control/action/detail/set_velocity__struct.h"

/// Struct defined in action/SetVelocity in the package rover_control.
typedef struct rover_control__action__SetVelocity_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  rover_control__action__SetVelocity_Goal goal;
} rover_control__action__SetVelocity_SendGoal_Request;

// Struct for a sequence of rover_control__action__SetVelocity_SendGoal_Request.
typedef struct rover_control__action__SetVelocity_SendGoal_Request__Sequence
{
  rover_control__action__SetVelocity_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__SetVelocity_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/SetVelocity in the package rover_control.
typedef struct rover_control__action__SetVelocity_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} rover_control__action__SetVelocity_SendGoal_Response;

// Struct for a sequence of rover_control__action__SetVelocity_SendGoal_Response.
typedef struct rover_control__action__SetVelocity_SendGoal_Response__Sequence
{
  rover_control__action__SetVelocity_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__SetVelocity_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/SetVelocity in the package rover_control.
typedef struct rover_control__action__SetVelocity_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} rover_control__action__SetVelocity_GetResult_Request;

// Struct for a sequence of rover_control__action__SetVelocity_GetResult_Request.
typedef struct rover_control__action__SetVelocity_GetResult_Request__Sequence
{
  rover_control__action__SetVelocity_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__SetVelocity_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "rover_control/action/detail/set_velocity__struct.h"

/// Struct defined in action/SetVelocity in the package rover_control.
typedef struct rover_control__action__SetVelocity_GetResult_Response
{
  int8_t status;
  rover_control__action__SetVelocity_Result result;
} rover_control__action__SetVelocity_GetResult_Response;

// Struct for a sequence of rover_control__action__SetVelocity_GetResult_Response.
typedef struct rover_control__action__SetVelocity_GetResult_Response__Sequence
{
  rover_control__action__SetVelocity_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__SetVelocity_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "rover_control/action/detail/set_velocity__struct.h"

/// Struct defined in action/SetVelocity in the package rover_control.
typedef struct rover_control__action__SetVelocity_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  rover_control__action__SetVelocity_Feedback feedback;
} rover_control__action__SetVelocity_FeedbackMessage;

// Struct for a sequence of rover_control__action__SetVelocity_FeedbackMessage.
typedef struct rover_control__action__SetVelocity_FeedbackMessage__Sequence
{
  rover_control__action__SetVelocity_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__SetVelocity_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_CONTROL__ACTION__DETAIL__SET_VELOCITY__STRUCT_H_
