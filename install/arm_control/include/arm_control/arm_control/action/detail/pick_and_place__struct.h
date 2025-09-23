// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arm_control:action/PickAndPlace.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__ACTION__DETAIL__PICK_AND_PLACE__STRUCT_H_
#define ARM_CONTROL__ACTION__DETAIL__PICK_AND_PLACE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pick_pose'
// Member 'place_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'approach_offset'
// Member 'retreat_offset'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'grasp_strategy'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/PickAndPlace in the package arm_control.
typedef struct arm_control__action__PickAndPlace_Goal
{
  /// GOAL
  /// Object pickup pose
  geometry_msgs__msg__PoseStamped pick_pose;
  /// Object placement pose
  geometry_msgs__msg__PoseStamped place_pose;
  /// Pre-grasp approach offset [x,y,z] in meters
  geometry_msgs__msg__Vector3 approach_offset;
  /// Post-grasp retreat offset [x,y,z] in meters
  geometry_msgs__msg__Vector3 retreat_offset;
  /// Desired grasp force in Newtons
  float grasp_force;
  /// Grasp timeout in seconds
  float grasp_timeout;
  /// Grasp strategy: "force", "position", "adaptive"
  rosidl_runtime_c__String grasp_strategy;
  /// Enable grasp verification
  bool verify_grasp;
  /// Lift height after grasp (meters)
  float lift_height;
  /// Force for placing object (Newtons)
  float place_force;
  /// Use gentle placement strategy
  bool gentle_place;
  /// Movement velocity scaling
  float velocity_scaling;
  /// Movement acceleration scaling
  float acceleration_scaling;
} arm_control__action__PickAndPlace_Goal;

// Struct for a sequence of arm_control__action__PickAndPlace_Goal.
typedef struct arm_control__action__PickAndPlace_Goal__Sequence
{
  arm_control__action__PickAndPlace_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__PickAndPlace_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_message'
// Member 'failure_phase'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'final_object_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'final_joint_state'
#include "sensor_msgs/msg/detail/joint_state__struct.h"

/// Struct defined in action/PickAndPlace in the package arm_control.
typedef struct arm_control__action__PickAndPlace_Result
{
  /// RESULT
  /// True if pick and place completed
  bool success;
  /// Human-readable error description
  rosidl_runtime_c__String error_message;
  /// Structured error code
  int32_t error_code;
  /// Phase where failure occurred (APPROACH, GRASP, etc.)
  rosidl_runtime_c__String failure_phase;
  /// True if grasp was successful
  bool grasp_successful;
  /// True if place was successful
  bool place_successful;
  /// Actual grasp force achieved (Newtons)
  float grasp_force_achieved;
  /// Final object pose (if known)
  geometry_msgs__msg__PoseStamped final_object_pose;
  /// Total execution time (seconds)
  float total_execution_time;
  /// Final joint configuration
  sensor_msgs__msg__JointState final_joint_state;
} arm_control__action__PickAndPlace_Result;

// Struct for a sequence of arm_control__action__PickAndPlace_Result.
typedef struct arm_control__action__PickAndPlace_Result__Sequence
{
  arm_control__action__PickAndPlace_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__PickAndPlace_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_phase'
// Member 'grasp_status'
// Member 'status_message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/PickAndPlace in the package arm_control.
typedef struct arm_control__action__PickAndPlace_Feedback
{
  /// FEEDBACK
  /// Current phase (PLANNING, APPROACH, GRASP, LIFT, TRANSPORT, PLACE, RETREAT)
  rosidl_runtime_c__String current_phase;
  /// Progress within current phase
  float phase_progress;
  /// Overall operation progress
  float overall_progress;
  /// Current end-effector pose
  geometry_msgs__msg__PoseStamped current_pose;
  /// Current grasp force (Newtons)
  float current_grasp_force;
  /// True if object detected in gripper
  bool object_detected;
  /// Grasp status: "SECURE", "SOFT", "FAILED", "UNKNOWN"
  rosidl_runtime_c__String grasp_status;
  /// Human-readable status update
  rosidl_runtime_c__String status_message;
  /// Estimated time to completion (seconds)
  float estimated_time_remaining;
} arm_control__action__PickAndPlace_Feedback;

// Struct for a sequence of arm_control__action__PickAndPlace_Feedback.
typedef struct arm_control__action__PickAndPlace_Feedback__Sequence
{
  arm_control__action__PickAndPlace_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__PickAndPlace_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "arm_control/action/detail/pick_and_place__struct.h"

/// Struct defined in action/PickAndPlace in the package arm_control.
typedef struct arm_control__action__PickAndPlace_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  arm_control__action__PickAndPlace_Goal goal;
} arm_control__action__PickAndPlace_SendGoal_Request;

// Struct for a sequence of arm_control__action__PickAndPlace_SendGoal_Request.
typedef struct arm_control__action__PickAndPlace_SendGoal_Request__Sequence
{
  arm_control__action__PickAndPlace_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__PickAndPlace_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/PickAndPlace in the package arm_control.
typedef struct arm_control__action__PickAndPlace_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} arm_control__action__PickAndPlace_SendGoal_Response;

// Struct for a sequence of arm_control__action__PickAndPlace_SendGoal_Response.
typedef struct arm_control__action__PickAndPlace_SendGoal_Response__Sequence
{
  arm_control__action__PickAndPlace_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__PickAndPlace_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/PickAndPlace in the package arm_control.
typedef struct arm_control__action__PickAndPlace_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} arm_control__action__PickAndPlace_GetResult_Request;

// Struct for a sequence of arm_control__action__PickAndPlace_GetResult_Request.
typedef struct arm_control__action__PickAndPlace_GetResult_Request__Sequence
{
  arm_control__action__PickAndPlace_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__PickAndPlace_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "arm_control/action/detail/pick_and_place__struct.h"

/// Struct defined in action/PickAndPlace in the package arm_control.
typedef struct arm_control__action__PickAndPlace_GetResult_Response
{
  int8_t status;
  arm_control__action__PickAndPlace_Result result;
} arm_control__action__PickAndPlace_GetResult_Response;

// Struct for a sequence of arm_control__action__PickAndPlace_GetResult_Response.
typedef struct arm_control__action__PickAndPlace_GetResult_Response__Sequence
{
  arm_control__action__PickAndPlace_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__PickAndPlace_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "arm_control/action/detail/pick_and_place__struct.h"

/// Struct defined in action/PickAndPlace in the package arm_control.
typedef struct arm_control__action__PickAndPlace_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  arm_control__action__PickAndPlace_Feedback feedback;
} arm_control__action__PickAndPlace_FeedbackMessage;

// Struct for a sequence of arm_control__action__PickAndPlace_FeedbackMessage.
typedef struct arm_control__action__PickAndPlace_FeedbackMessage__Sequence
{
  arm_control__action__PickAndPlace_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_control__action__PickAndPlace_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARM_CONTROL__ACTION__DETAIL__PICK_AND_PLACE__STRUCT_H_
