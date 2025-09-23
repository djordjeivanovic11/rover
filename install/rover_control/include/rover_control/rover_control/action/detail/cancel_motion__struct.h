// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_control:action/CancelMotion.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__ACTION__DETAIL__CANCEL_MOTION__STRUCT_H_
#define ROVER_CONTROL__ACTION__DETAIL__CANCEL_MOTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/CancelMotion in the package rover_control.
typedef struct rover_control__action__CancelMotion_Goal
{
  /// GOAL
  /// True for emergency stop (immediate halt)
  bool emergency_stop;
  /// Time to decelerate to stop (seconds, ignored if emergency)
  float deceleration_time;
  /// Cancel all running navigation actions
  bool cancel_all_actions;
  /// Reason for cancellation (for logging)
  rosidl_runtime_c__String reason;
} rover_control__action__CancelMotion_Goal;

// Struct for a sequence of rover_control__action__CancelMotion_Goal.
typedef struct rover_control__action__CancelMotion_Goal__Sequence
{
  rover_control__action__CancelMotion_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__CancelMotion_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'velocity_at_stop'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'final_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/CancelMotion in the package rover_control.
typedef struct rover_control__action__CancelMotion_Result
{
  /// RESULT
  /// True if motion canceled successfully
  bool success;
  /// Structured result code (0=SUCCESS, etc.)
  int32_t result_code;
  /// Human-readable error description
  rosidl_runtime_c__String error_message;
  /// Time taken to stop (seconds)
  float stop_time;
  /// Velocity when stop command was issued
  geometry_msgs__msg__Twist velocity_at_stop;
  /// Final pose after stopping
  geometry_msgs__msg__PoseStamped final_pose;
  /// Number of actions that were cancelled
  int32_t actions_cancelled;
} rover_control__action__CancelMotion_Result;

// Struct for a sequence of rover_control__action__CancelMotion_Result.
typedef struct rover_control__action__CancelMotion_Result__Sequence
{
  rover_control__action__CancelMotion_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__CancelMotion_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_state'
// Member 'status_message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'current_velocity'
// already included above
// #include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in action/CancelMotion in the package rover_control.
typedef struct rover_control__action__CancelMotion_Feedback
{
  /// FEEDBACK
  /// Current state (STOPPING, STOPPED)
  rosidl_runtime_c__String current_state;
  /// Current rover velocity
  geometry_msgs__msg__Twist current_velocity;
  /// Deceleration progress
  float progress_percentage;
  /// Human-readable status update
  rosidl_runtime_c__String status_message;
} rover_control__action__CancelMotion_Feedback;

// Struct for a sequence of rover_control__action__CancelMotion_Feedback.
typedef struct rover_control__action__CancelMotion_Feedback__Sequence
{
  rover_control__action__CancelMotion_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__CancelMotion_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "rover_control/action/detail/cancel_motion__struct.h"

/// Struct defined in action/CancelMotion in the package rover_control.
typedef struct rover_control__action__CancelMotion_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  rover_control__action__CancelMotion_Goal goal;
} rover_control__action__CancelMotion_SendGoal_Request;

// Struct for a sequence of rover_control__action__CancelMotion_SendGoal_Request.
typedef struct rover_control__action__CancelMotion_SendGoal_Request__Sequence
{
  rover_control__action__CancelMotion_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__CancelMotion_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/CancelMotion in the package rover_control.
typedef struct rover_control__action__CancelMotion_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} rover_control__action__CancelMotion_SendGoal_Response;

// Struct for a sequence of rover_control__action__CancelMotion_SendGoal_Response.
typedef struct rover_control__action__CancelMotion_SendGoal_Response__Sequence
{
  rover_control__action__CancelMotion_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__CancelMotion_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/CancelMotion in the package rover_control.
typedef struct rover_control__action__CancelMotion_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} rover_control__action__CancelMotion_GetResult_Request;

// Struct for a sequence of rover_control__action__CancelMotion_GetResult_Request.
typedef struct rover_control__action__CancelMotion_GetResult_Request__Sequence
{
  rover_control__action__CancelMotion_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__CancelMotion_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "rover_control/action/detail/cancel_motion__struct.h"

/// Struct defined in action/CancelMotion in the package rover_control.
typedef struct rover_control__action__CancelMotion_GetResult_Response
{
  int8_t status;
  rover_control__action__CancelMotion_Result result;
} rover_control__action__CancelMotion_GetResult_Response;

// Struct for a sequence of rover_control__action__CancelMotion_GetResult_Response.
typedef struct rover_control__action__CancelMotion_GetResult_Response__Sequence
{
  rover_control__action__CancelMotion_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__CancelMotion_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "rover_control/action/detail/cancel_motion__struct.h"

/// Struct defined in action/CancelMotion in the package rover_control.
typedef struct rover_control__action__CancelMotion_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  rover_control__action__CancelMotion_Feedback feedback;
} rover_control__action__CancelMotion_FeedbackMessage;

// Struct for a sequence of rover_control__action__CancelMotion_FeedbackMessage.
typedef struct rover_control__action__CancelMotion_FeedbackMessage__Sequence
{
  rover_control__action__CancelMotion_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__CancelMotion_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_CONTROL__ACTION__DETAIL__CANCEL_MOTION__STRUCT_H_
