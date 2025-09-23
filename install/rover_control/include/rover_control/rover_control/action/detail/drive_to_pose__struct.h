// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_control:action/DriveToPose.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__ACTION__DETAIL__DRIVE_TO_POSE__STRUCT_H_
#define ROVER_CONTROL__ACTION__DETAIL__DRIVE_TO_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'mission_mode'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/DriveToPose in the package rover_control.
typedef struct rover_control__action__DriveToPose_Goal
{
  /// GOAL
  /// Target pose in map frame
  geometry_msgs__msg__PoseStamped target_pose;
  /// Maximum velocity during approach (m/s)
  float max_velocity;
  /// Position tolerance for success (meters)
  float position_tolerance;
  /// Orientation tolerance for success (radians)
  float orientation_tolerance;
  /// Execution timeout (seconds)
  float timeout;
  /// Enable visual servoing for final 3m
  bool use_visual_servoing;
  /// Mission mode: exploration, science, return_to_base
  rosidl_runtime_c__String mission_mode;
  /// Allow reverse driving if needed
  bool reverse_allowed;
  /// Velocity for final approach (m/s)
  float approach_velocity;
} rover_control__action__DriveToPose_Goal;

// Struct for a sequence of rover_control__action__DriveToPose_Goal.
typedef struct rover_control__action__DriveToPose_Goal__Sequence
{
  rover_control__action__DriveToPose_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__DriveToPose_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'final_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'executed_path'
#include "nav_msgs/msg/detail/path__struct.h"

/// Struct defined in action/DriveToPose in the package rover_control.
typedef struct rover_control__action__DriveToPose_Result
{
  /// RESULT
  /// True if pose reached successfully
  bool success;
  /// Structured result code (0=SUCCESS, 1=CANCELLED, etc.)
  int32_t result_code;
  /// Human-readable error description
  rosidl_runtime_c__String error_message;
  /// Final position error (meters)
  float final_position_error;
  /// Final orientation error (radians)
  float final_orientation_error;
  /// Total execution time (seconds)
  float execution_time;
  /// Total distance traveled (meters)
  float distance_traveled;
  /// Final pose achieved
  geometry_msgs__msg__PoseStamped final_pose;
  /// Path actually executed
  nav_msgs__msg__Path executed_path;
} rover_control__action__DriveToPose_Result;

// Struct for a sequence of rover_control__action__DriveToPose_Result.
typedef struct rover_control__action__DriveToPose_Result__Sequence
{
  rover_control__action__DriveToPose_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__DriveToPose_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_state'
// Member 'status_message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/DriveToPose in the package rover_control.
typedef struct rover_control__action__DriveToPose_Feedback
{
  /// FEEDBACK
  /// Current execution state (PLANNING, EXECUTING, APPROACHING, etc.)
  rosidl_runtime_c__String current_state;
  /// Execution progress
  float progress_percentage;
  /// Current rover pose
  geometry_msgs__msg__PoseStamped current_pose;
  /// Estimated time to completion (seconds)
  float estimated_time_remaining;
  /// Distance remaining to target (meters)
  float distance_remaining;
  /// Current rover velocity (m/s)
  float current_velocity;
  /// Human-readable status update
  rosidl_runtime_c__String status_message;
  /// True if stuck condition detected
  bool stuck_detected;
  /// Remaining autonomy timer (seconds)
  float autonomy_time_remaining;
} rover_control__action__DriveToPose_Feedback;

// Struct for a sequence of rover_control__action__DriveToPose_Feedback.
typedef struct rover_control__action__DriveToPose_Feedback__Sequence
{
  rover_control__action__DriveToPose_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__DriveToPose_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "rover_control/action/detail/drive_to_pose__struct.h"

/// Struct defined in action/DriveToPose in the package rover_control.
typedef struct rover_control__action__DriveToPose_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  rover_control__action__DriveToPose_Goal goal;
} rover_control__action__DriveToPose_SendGoal_Request;

// Struct for a sequence of rover_control__action__DriveToPose_SendGoal_Request.
typedef struct rover_control__action__DriveToPose_SendGoal_Request__Sequence
{
  rover_control__action__DriveToPose_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__DriveToPose_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/DriveToPose in the package rover_control.
typedef struct rover_control__action__DriveToPose_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} rover_control__action__DriveToPose_SendGoal_Response;

// Struct for a sequence of rover_control__action__DriveToPose_SendGoal_Response.
typedef struct rover_control__action__DriveToPose_SendGoal_Response__Sequence
{
  rover_control__action__DriveToPose_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__DriveToPose_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/DriveToPose in the package rover_control.
typedef struct rover_control__action__DriveToPose_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} rover_control__action__DriveToPose_GetResult_Request;

// Struct for a sequence of rover_control__action__DriveToPose_GetResult_Request.
typedef struct rover_control__action__DriveToPose_GetResult_Request__Sequence
{
  rover_control__action__DriveToPose_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__DriveToPose_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "rover_control/action/detail/drive_to_pose__struct.h"

/// Struct defined in action/DriveToPose in the package rover_control.
typedef struct rover_control__action__DriveToPose_GetResult_Response
{
  int8_t status;
  rover_control__action__DriveToPose_Result result;
} rover_control__action__DriveToPose_GetResult_Response;

// Struct for a sequence of rover_control__action__DriveToPose_GetResult_Response.
typedef struct rover_control__action__DriveToPose_GetResult_Response__Sequence
{
  rover_control__action__DriveToPose_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__DriveToPose_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "rover_control/action/detail/drive_to_pose__struct.h"

/// Struct defined in action/DriveToPose in the package rover_control.
typedef struct rover_control__action__DriveToPose_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  rover_control__action__DriveToPose_Feedback feedback;
} rover_control__action__DriveToPose_FeedbackMessage;

// Struct for a sequence of rover_control__action__DriveToPose_FeedbackMessage.
typedef struct rover_control__action__DriveToPose_FeedbackMessage__Sequence
{
  rover_control__action__DriveToPose_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__DriveToPose_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_CONTROL__ACTION__DETAIL__DRIVE_TO_POSE__STRUCT_H_
