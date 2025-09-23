// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_control:action/FollowPath.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__ACTION__DETAIL__FOLLOW_PATH__STRUCT_H_
#define ROVER_CONTROL__ACTION__DETAIL__FOLLOW_PATH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'path'
#include "nav_msgs/msg/detail/path__struct.h"
// Member 'mission_mode'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/FollowPath in the package rover_control.
typedef struct rover_control__action__FollowPath_Goal
{
  /// GOAL
  /// Path to follow with waypoints
  nav_msgs__msg__Path path;
  /// Maximum velocity during execution (m/s)
  float max_velocity;
  /// Tolerance for reaching waypoints (meters)
  float waypoint_tolerance;
  /// Allow reverse driving if needed
  bool reverse_allowed;
  /// Total execution timeout (seconds)
  float timeout;
  /// Mission mode for parameter selection
  rosidl_runtime_c__String mission_mode;
  /// Skip waypoints that are unreachable
  bool skip_unreachable;
  /// Pure pursuit lookahead distance (meters)
  float lookahead_distance;
  /// Enable dynamic obstacle avoidance
  bool use_obstacle_avoidance;
} rover_control__action__FollowPath_Goal;

// Struct for a sequence of rover_control__action__FollowPath_Goal.
typedef struct rover_control__action__FollowPath_Goal__Sequence
{
  rover_control__action__FollowPath_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__FollowPath_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'executed_path'
// already included above
// #include "nav_msgs/msg/detail/path__struct.h"
// Member 'final_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/FollowPath in the package rover_control.
typedef struct rover_control__action__FollowPath_Result
{
  /// RESULT
  /// True if path completed successfully
  bool success;
  /// Structured result code (0=SUCCESS, 1=CANCELLED, etc.)
  int32_t result_code;
  /// Human-readable error description
  rosidl_runtime_c__String error_message;
  /// Number of waypoints successfully reached
  int32_t waypoints_completed;
  /// Total number of waypoints in path
  int32_t total_waypoints;
  /// Total distance traveled (meters)
  float total_distance;
  /// Total execution time (seconds)
  float execution_time;
  /// Actual path executed
  nav_msgs__msg__Path executed_path;
  /// Final pose achieved
  geometry_msgs__msg__PoseStamped final_pose;
  /// Average velocity during execution (m/s)
  float average_velocity;
} rover_control__action__FollowPath_Result;

// Struct for a sequence of rover_control__action__FollowPath_Result.
typedef struct rover_control__action__FollowPath_Result__Sequence
{
  rover_control__action__FollowPath_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__FollowPath_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_state'
// Member 'status_message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'current_pose'
// Member 'target_waypoint'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/FollowPath in the package rover_control.
typedef struct rover_control__action__FollowPath_Feedback
{
  /// FEEDBACK
  /// Current execution state (FOLLOWING, APPROACHING_WAYPOINT, etc.)
  rosidl_runtime_c__String current_state;
  /// Execution progress
  float progress_percentage;
  /// Index of current target waypoint
  int32_t current_waypoint;
  /// Current rover pose
  geometry_msgs__msg__PoseStamped current_pose;
  /// Estimated time to completion (seconds)
  float estimated_time_remaining;
  /// Distance remaining on path (meters)
  float distance_remaining;
  /// Current rover velocity (m/s)
  float current_velocity;
  /// Human-readable status update
  rosidl_runtime_c__String status_message;
  /// True if stuck condition detected
  bool stuck_detected;
  /// Remaining autonomy timer (seconds)
  float autonomy_time_remaining;
  /// Current target waypoint
  geometry_msgs__msg__PoseStamped target_waypoint;
} rover_control__action__FollowPath_Feedback;

// Struct for a sequence of rover_control__action__FollowPath_Feedback.
typedef struct rover_control__action__FollowPath_Feedback__Sequence
{
  rover_control__action__FollowPath_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__FollowPath_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "rover_control/action/detail/follow_path__struct.h"

/// Struct defined in action/FollowPath in the package rover_control.
typedef struct rover_control__action__FollowPath_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  rover_control__action__FollowPath_Goal goal;
} rover_control__action__FollowPath_SendGoal_Request;

// Struct for a sequence of rover_control__action__FollowPath_SendGoal_Request.
typedef struct rover_control__action__FollowPath_SendGoal_Request__Sequence
{
  rover_control__action__FollowPath_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__FollowPath_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/FollowPath in the package rover_control.
typedef struct rover_control__action__FollowPath_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} rover_control__action__FollowPath_SendGoal_Response;

// Struct for a sequence of rover_control__action__FollowPath_SendGoal_Response.
typedef struct rover_control__action__FollowPath_SendGoal_Response__Sequence
{
  rover_control__action__FollowPath_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__FollowPath_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/FollowPath in the package rover_control.
typedef struct rover_control__action__FollowPath_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} rover_control__action__FollowPath_GetResult_Request;

// Struct for a sequence of rover_control__action__FollowPath_GetResult_Request.
typedef struct rover_control__action__FollowPath_GetResult_Request__Sequence
{
  rover_control__action__FollowPath_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__FollowPath_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "rover_control/action/detail/follow_path__struct.h"

/// Struct defined in action/FollowPath in the package rover_control.
typedef struct rover_control__action__FollowPath_GetResult_Response
{
  int8_t status;
  rover_control__action__FollowPath_Result result;
} rover_control__action__FollowPath_GetResult_Response;

// Struct for a sequence of rover_control__action__FollowPath_GetResult_Response.
typedef struct rover_control__action__FollowPath_GetResult_Response__Sequence
{
  rover_control__action__FollowPath_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__FollowPath_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "rover_control/action/detail/follow_path__struct.h"

/// Struct defined in action/FollowPath in the package rover_control.
typedef struct rover_control__action__FollowPath_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  rover_control__action__FollowPath_Feedback feedback;
} rover_control__action__FollowPath_FeedbackMessage;

// Struct for a sequence of rover_control__action__FollowPath_FeedbackMessage.
typedef struct rover_control__action__FollowPath_FeedbackMessage__Sequence
{
  rover_control__action__FollowPath_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_control__action__FollowPath_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_CONTROL__ACTION__DETAIL__FOLLOW_PATH__STRUCT_H_
