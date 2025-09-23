// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arm_control:action/GoToNamedPose.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__ACTION__DETAIL__GO_TO_NAMED_POSE__STRUCT_H_
#define ARM_CONTROL__ACTION__DETAIL__GO_TO_NAMED_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose_name'
// Member 'planning_group'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/GoToNamedPose in the package arm_control.
typedef struct arm_control__action__GoToNamedPose_Goal
{
  /// GOAL
  /// Named pose from arm_params.yaml mission_poses
  rosidl_runtime_c__String pose_name;
  /// Velocity scaling factor
  float velocity_scaling;
  /// Acceleration scaling factor
  float acceleration_scaling;
  /// If true, only plan but don't execute
  bool plan_only;
  /// Planning timeout in seconds
  float planning_timeout;
  /// Execution timeout in seconds
  float execution_timeout;
  /// Enable collision checking
  bool collision_checking;
  /// Planning group to use
  rosidl_runtime_c__String planning_group;
} arm_control__action__GoToNamedPose_Goal;

// Struct for a sequence of arm_control__action__GoToNamedPose_Goal.
typedef struct arm_control__action__GoToNamedPose_Goal__Sequence
{
  arm_control__action__GoToNamedPose_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__GoToNamedPose_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'final_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/GoToNamedPose in the package arm_control.
typedef struct arm_control__action__GoToNamedPose_Result
{
  /// RESULT
  /// True if pose reached successfully
  bool success;
  /// Human-readable error description
  rosidl_runtime_c__String error_message;
  /// Structured error code from safety_params.yaml
  int32_t error_code;
  /// Final end-effector pose achieved
  geometry_msgs__msg__PoseStamped final_pose;
  /// Time spent planning (seconds)
  float planning_time;
  /// Time spent executing (seconds)
  float execution_time;
  /// Final position error (meters)
  float final_position_error;
  /// Final orientation error (radians)
  float final_orientation_error;
} arm_control__action__GoToNamedPose_Result;

// Struct for a sequence of arm_control__action__GoToNamedPose_Result.
typedef struct arm_control__action__GoToNamedPose_Result__Sequence
{
  arm_control__action__GoToNamedPose_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__GoToNamedPose_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_state'
// Member 'status_message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'current_joint_state'
#include "sensor_msgs/msg/detail/joint_state__struct.h"

/// Struct defined in action/GoToNamedPose in the package arm_control.
typedef struct arm_control__action__GoToNamedPose_Feedback
{
  /// FEEDBACK
  /// Current execution state (PLANNING, EXECUTING, etc.)
  rosidl_runtime_c__String current_state;
  /// Execution progress
  float progress_percentage;
  /// Current end-effector pose
  geometry_msgs__msg__PoseStamped current_pose;
  /// Estimated time to completion (seconds)
  float estimated_time_remaining;
  /// Current joint positions
  sensor_msgs__msg__JointState current_joint_state;
  /// True if collision detected
  bool collision_detected;
  /// Human-readable status update
  rosidl_runtime_c__String status_message;
} arm_control__action__GoToNamedPose_Feedback;

// Struct for a sequence of arm_control__action__GoToNamedPose_Feedback.
typedef struct arm_control__action__GoToNamedPose_Feedback__Sequence
{
  arm_control__action__GoToNamedPose_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__GoToNamedPose_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "arm_control/action/detail/go_to_named_pose__struct.h"

/// Struct defined in action/GoToNamedPose in the package arm_control.
typedef struct arm_control__action__GoToNamedPose_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  arm_control__action__GoToNamedPose_Goal goal;
} arm_control__action__GoToNamedPose_SendGoal_Request;

// Struct for a sequence of arm_control__action__GoToNamedPose_SendGoal_Request.
typedef struct arm_control__action__GoToNamedPose_SendGoal_Request__Sequence
{
  arm_control__action__GoToNamedPose_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__GoToNamedPose_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/GoToNamedPose in the package arm_control.
typedef struct arm_control__action__GoToNamedPose_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} arm_control__action__GoToNamedPose_SendGoal_Response;

// Struct for a sequence of arm_control__action__GoToNamedPose_SendGoal_Response.
typedef struct arm_control__action__GoToNamedPose_SendGoal_Response__Sequence
{
  arm_control__action__GoToNamedPose_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__GoToNamedPose_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/GoToNamedPose in the package arm_control.
typedef struct arm_control__action__GoToNamedPose_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} arm_control__action__GoToNamedPose_GetResult_Request;

// Struct for a sequence of arm_control__action__GoToNamedPose_GetResult_Request.
typedef struct arm_control__action__GoToNamedPose_GetResult_Request__Sequence
{
  arm_control__action__GoToNamedPose_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__GoToNamedPose_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "arm_control/action/detail/go_to_named_pose__struct.h"

/// Struct defined in action/GoToNamedPose in the package arm_control.
typedef struct arm_control__action__GoToNamedPose_GetResult_Response
{
  int8_t status;
  arm_control__action__GoToNamedPose_Result result;
} arm_control__action__GoToNamedPose_GetResult_Response;

// Struct for a sequence of arm_control__action__GoToNamedPose_GetResult_Response.
typedef struct arm_control__action__GoToNamedPose_GetResult_Response__Sequence
{
  arm_control__action__GoToNamedPose_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__GoToNamedPose_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "arm_control/action/detail/go_to_named_pose__struct.h"

/// Struct defined in action/GoToNamedPose in the package arm_control.
typedef struct arm_control__action__GoToNamedPose_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  arm_control__action__GoToNamedPose_Feedback feedback;
} arm_control__action__GoToNamedPose_FeedbackMessage;

// Struct for a sequence of arm_control__action__GoToNamedPose_FeedbackMessage.
typedef struct arm_control__action__GoToNamedPose_FeedbackMessage__Sequence
{
  arm_control__action__GoToNamedPose_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__GoToNamedPose_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARM_CONTROL__ACTION__DETAIL__GO_TO_NAMED_POSE__STRUCT_H_
