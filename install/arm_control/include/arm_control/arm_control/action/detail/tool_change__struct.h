// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arm_control:action/ToolChange.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__ACTION__DETAIL__TOOL_CHANGE__STRUCT_H_
#define ARM_CONTROL__ACTION__DETAIL__TOOL_CHANGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'new_tool_name'
// Member 'current_tool_name'
// Member 'change_strategy'
#include "rosidl_runtime_c/string.h"
// Member 'tool_dock_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/ToolChange in the package arm_control.
typedef struct arm_control__action__ToolChange_Goal
{
  /// GOAL
  /// Tool name from arm_params.yaml tools section
  rosidl_runtime_c__String new_tool_name;
  /// Current tool name (for validation)
  rosidl_runtime_c__String current_tool_name;
  /// Auto-detect current tool if true
  bool auto_detect_current;
  /// Tool docking station pose
  geometry_msgs__msg__PoseStamped tool_dock_pose;
  /// Distance for approach phase (meters)
  float approach_distance;
  /// Coupling operation timeout (seconds)
  float coupling_timeout;
  /// Verify tool change completion
  bool verify_tool_change;
  /// Update MoveIt kinematics after change
  bool update_kinematics;
  /// Slow velocity for tool change operations
  float velocity_scaling;
  /// Change strategy: "auto", "manual", "force"
  rosidl_runtime_c__String change_strategy;
} arm_control__action__ToolChange_Goal;

// Struct for a sequence of arm_control__action__ToolChange_Goal.
typedef struct arm_control__action__ToolChange_Goal__Sequence
{
  arm_control__action__ToolChange_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__ToolChange_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_message'
// Member 'old_tool_name'
// Member 'new_tool_name'
// Member 'tool_configuration'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'final_tool_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/ToolChange in the package arm_control.
typedef struct arm_control__action__ToolChange_Result
{
  /// RESULT
  /// True if tool change completed successfully
  bool success;
  /// Human-readable error description
  rosidl_runtime_c__String error_message;
  /// Structured error code
  int32_t error_code;
  /// Tool that was removed
  rosidl_runtime_c__String old_tool_name;
  /// Tool that was installed
  rosidl_runtime_c__String new_tool_name;
  /// True if coupling mechanism verified
  bool coupling_verified;
  /// True if kinematics successfully updated
  bool kinematics_updated;
  /// Final tool tip pose
  geometry_msgs__msg__PoseStamped final_tool_pose;
  /// Total operation time (seconds)
  float total_execution_time;
  /// New tool configuration loaded
  rosidl_runtime_c__String tool_configuration;
} arm_control__action__ToolChange_Result;

// Struct for a sequence of arm_control__action__ToolChange_Result.
typedef struct arm_control__action__ToolChange_Result__Sequence
{
  arm_control__action__ToolChange_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__ToolChange_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_phase'
// Member 'coupling_status'
// Member 'kinematics_status'
// Member 'status_message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/ToolChange in the package arm_control.
typedef struct arm_control__action__ToolChange_Feedback
{
  /// FEEDBACK
  /// Current phase (APPROACH, RELEASE, RETRACT, ACQUIRE, COUPLE, VERIFY)
  rosidl_runtime_c__String current_phase;
  /// Progress within current phase
  float phase_progress;
  /// Overall operation progress
  float overall_progress;
  /// True if tool detected at dock
  bool tool_detected;
  /// True if coupling mechanism engaged
  bool coupling_engaged;
  /// Coupling status: "LOCKED", "UNLOCKED", "ENGAGING", "ERROR"
  rosidl_runtime_c__String coupling_status;
  /// Kinematics update status: "LOADING", "UPDATED", "FAILED"
  rosidl_runtime_c__String kinematics_status;
  /// Human-readable status update
  rosidl_runtime_c__String status_message;
  /// Estimated time to completion (seconds)
  float estimated_time_remaining;
} arm_control__action__ToolChange_Feedback;

// Struct for a sequence of arm_control__action__ToolChange_Feedback.
typedef struct arm_control__action__ToolChange_Feedback__Sequence
{
  arm_control__action__ToolChange_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__ToolChange_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "arm_control/action/detail/tool_change__struct.h"

/// Struct defined in action/ToolChange in the package arm_control.
typedef struct arm_control__action__ToolChange_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  arm_control__action__ToolChange_Goal goal;
} arm_control__action__ToolChange_SendGoal_Request;

// Struct for a sequence of arm_control__action__ToolChange_SendGoal_Request.
typedef struct arm_control__action__ToolChange_SendGoal_Request__Sequence
{
  arm_control__action__ToolChange_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__ToolChange_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/ToolChange in the package arm_control.
typedef struct arm_control__action__ToolChange_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} arm_control__action__ToolChange_SendGoal_Response;

// Struct for a sequence of arm_control__action__ToolChange_SendGoal_Response.
typedef struct arm_control__action__ToolChange_SendGoal_Response__Sequence
{
  arm_control__action__ToolChange_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__ToolChange_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/ToolChange in the package arm_control.
typedef struct arm_control__action__ToolChange_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} arm_control__action__ToolChange_GetResult_Request;

// Struct for a sequence of arm_control__action__ToolChange_GetResult_Request.
typedef struct arm_control__action__ToolChange_GetResult_Request__Sequence
{
  arm_control__action__ToolChange_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__ToolChange_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "arm_control/action/detail/tool_change__struct.h"

/// Struct defined in action/ToolChange in the package arm_control.
typedef struct arm_control__action__ToolChange_GetResult_Response
{
  int8_t status;
  arm_control__action__ToolChange_Result result;
} arm_control__action__ToolChange_GetResult_Response;

// Struct for a sequence of arm_control__action__ToolChange_GetResult_Response.
typedef struct arm_control__action__ToolChange_GetResult_Response__Sequence
{
  arm_control__action__ToolChange_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__ToolChange_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "arm_control/action/detail/tool_change__struct.h"

/// Struct defined in action/ToolChange in the package arm_control.
typedef struct arm_control__action__ToolChange_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  arm_control__action__ToolChange_Feedback feedback;
} arm_control__action__ToolChange_FeedbackMessage;

// Struct for a sequence of arm_control__action__ToolChange_FeedbackMessage.
typedef struct arm_control__action__ToolChange_FeedbackMessage__Sequence
{
  arm_control__action__ToolChange_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__ToolChange_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARM_CONTROL__ACTION__DETAIL__TOOL_CHANGE__STRUCT_H_
