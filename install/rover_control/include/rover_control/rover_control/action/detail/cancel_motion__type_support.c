// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rover_control:action/CancelMotion.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rover_control/action/detail/cancel_motion__rosidl_typesupport_introspection_c.h"
#include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rover_control/action/detail/cancel_motion__functions.h"
#include "rover_control/action/detail/cancel_motion__struct.h"


// Include directives for member types
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__CancelMotion_Goal__rosidl_typesupport_introspection_c__CancelMotion_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__CancelMotion_Goal__init(message_memory);
}

void rover_control__action__CancelMotion_Goal__rosidl_typesupport_introspection_c__CancelMotion_Goal_fini_function(void * message_memory)
{
  rover_control__action__CancelMotion_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__CancelMotion_Goal__rosidl_typesupport_introspection_c__CancelMotion_Goal_message_member_array[4] = {
  {
    "emergency_stop",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Goal, emergency_stop),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "deceleration_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Goal, deceleration_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cancel_all_actions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Goal, cancel_all_actions),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reason",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Goal, reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__CancelMotion_Goal__rosidl_typesupport_introspection_c__CancelMotion_Goal_message_members = {
  "rover_control__action",  // message namespace
  "CancelMotion_Goal",  // message name
  4,  // number of fields
  sizeof(rover_control__action__CancelMotion_Goal),
  rover_control__action__CancelMotion_Goal__rosidl_typesupport_introspection_c__CancelMotion_Goal_message_member_array,  // message members
  rover_control__action__CancelMotion_Goal__rosidl_typesupport_introspection_c__CancelMotion_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__CancelMotion_Goal__rosidl_typesupport_introspection_c__CancelMotion_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__CancelMotion_Goal__rosidl_typesupport_introspection_c__CancelMotion_Goal_message_type_support_handle = {
  0,
  &rover_control__action__CancelMotion_Goal__rosidl_typesupport_introspection_c__CancelMotion_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_Goal)() {
  if (!rover_control__action__CancelMotion_Goal__rosidl_typesupport_introspection_c__CancelMotion_Goal_message_type_support_handle.typesupport_identifier) {
    rover_control__action__CancelMotion_Goal__rosidl_typesupport_introspection_c__CancelMotion_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__CancelMotion_Goal__rosidl_typesupport_introspection_c__CancelMotion_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rover_control/action/detail/cancel_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__functions.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__struct.h"


// Include directives for member types
// Member `error_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `velocity_at_stop`
#include "geometry_msgs/msg/twist.h"
// Member `velocity_at_stop`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"
// Member `final_pose`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `final_pose`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__CancelMotion_Result__rosidl_typesupport_introspection_c__CancelMotion_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__CancelMotion_Result__init(message_memory);
}

void rover_control__action__CancelMotion_Result__rosidl_typesupport_introspection_c__CancelMotion_Result_fini_function(void * message_memory)
{
  rover_control__action__CancelMotion_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__CancelMotion_Result__rosidl_typesupport_introspection_c__CancelMotion_Result_message_member_array[7] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Result, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Result, result_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Result, error_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stop_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Result, stop_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity_at_stop",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Result, velocity_at_stop),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Result, final_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "actions_cancelled",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Result, actions_cancelled),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__CancelMotion_Result__rosidl_typesupport_introspection_c__CancelMotion_Result_message_members = {
  "rover_control__action",  // message namespace
  "CancelMotion_Result",  // message name
  7,  // number of fields
  sizeof(rover_control__action__CancelMotion_Result),
  rover_control__action__CancelMotion_Result__rosidl_typesupport_introspection_c__CancelMotion_Result_message_member_array,  // message members
  rover_control__action__CancelMotion_Result__rosidl_typesupport_introspection_c__CancelMotion_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__CancelMotion_Result__rosidl_typesupport_introspection_c__CancelMotion_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__CancelMotion_Result__rosidl_typesupport_introspection_c__CancelMotion_Result_message_type_support_handle = {
  0,
  &rover_control__action__CancelMotion_Result__rosidl_typesupport_introspection_c__CancelMotion_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_Result)() {
  rover_control__action__CancelMotion_Result__rosidl_typesupport_introspection_c__CancelMotion_Result_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  rover_control__action__CancelMotion_Result__rosidl_typesupport_introspection_c__CancelMotion_Result_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!rover_control__action__CancelMotion_Result__rosidl_typesupport_introspection_c__CancelMotion_Result_message_type_support_handle.typesupport_identifier) {
    rover_control__action__CancelMotion_Result__rosidl_typesupport_introspection_c__CancelMotion_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__CancelMotion_Result__rosidl_typesupport_introspection_c__CancelMotion_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rover_control/action/detail/cancel_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__functions.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__struct.h"


// Include directives for member types
// Member `current_state`
// Member `status_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `current_velocity`
// already included above
// #include "geometry_msgs/msg/twist.h"
// Member `current_velocity`
// already included above
// #include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__CancelMotion_Feedback__rosidl_typesupport_introspection_c__CancelMotion_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__CancelMotion_Feedback__init(message_memory);
}

void rover_control__action__CancelMotion_Feedback__rosidl_typesupport_introspection_c__CancelMotion_Feedback_fini_function(void * message_memory)
{
  rover_control__action__CancelMotion_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__CancelMotion_Feedback__rosidl_typesupport_introspection_c__CancelMotion_Feedback_message_member_array[4] = {
  {
    "current_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Feedback, current_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Feedback, current_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "progress_percentage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Feedback, progress_percentage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status_message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_Feedback, status_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__CancelMotion_Feedback__rosidl_typesupport_introspection_c__CancelMotion_Feedback_message_members = {
  "rover_control__action",  // message namespace
  "CancelMotion_Feedback",  // message name
  4,  // number of fields
  sizeof(rover_control__action__CancelMotion_Feedback),
  rover_control__action__CancelMotion_Feedback__rosidl_typesupport_introspection_c__CancelMotion_Feedback_message_member_array,  // message members
  rover_control__action__CancelMotion_Feedback__rosidl_typesupport_introspection_c__CancelMotion_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__CancelMotion_Feedback__rosidl_typesupport_introspection_c__CancelMotion_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__CancelMotion_Feedback__rosidl_typesupport_introspection_c__CancelMotion_Feedback_message_type_support_handle = {
  0,
  &rover_control__action__CancelMotion_Feedback__rosidl_typesupport_introspection_c__CancelMotion_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_Feedback)() {
  rover_control__action__CancelMotion_Feedback__rosidl_typesupport_introspection_c__CancelMotion_Feedback_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  if (!rover_control__action__CancelMotion_Feedback__rosidl_typesupport_introspection_c__CancelMotion_Feedback_message_type_support_handle.typesupport_identifier) {
    rover_control__action__CancelMotion_Feedback__rosidl_typesupport_introspection_c__CancelMotion_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__CancelMotion_Feedback__rosidl_typesupport_introspection_c__CancelMotion_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rover_control/action/detail/cancel_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__functions.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "rover_control/action/cancel_motion.h"
// Member `goal`
// already included above
// #include "rover_control/action/detail/cancel_motion__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__CancelMotion_SendGoal_Request__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__CancelMotion_SendGoal_Request__init(message_memory);
}

void rover_control__action__CancelMotion_SendGoal_Request__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_fini_function(void * message_memory)
{
  rover_control__action__CancelMotion_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__CancelMotion_SendGoal_Request__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__CancelMotion_SendGoal_Request__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_message_members = {
  "rover_control__action",  // message namespace
  "CancelMotion_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(rover_control__action__CancelMotion_SendGoal_Request),
  rover_control__action__CancelMotion_SendGoal_Request__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_message_member_array,  // message members
  rover_control__action__CancelMotion_SendGoal_Request__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__CancelMotion_SendGoal_Request__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__CancelMotion_SendGoal_Request__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_message_type_support_handle = {
  0,
  &rover_control__action__CancelMotion_SendGoal_Request__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_SendGoal_Request)() {
  rover_control__action__CancelMotion_SendGoal_Request__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  rover_control__action__CancelMotion_SendGoal_Request__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_Goal)();
  if (!rover_control__action__CancelMotion_SendGoal_Request__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    rover_control__action__CancelMotion_SendGoal_Request__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__CancelMotion_SendGoal_Request__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rover_control/action/detail/cancel_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__functions.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__CancelMotion_SendGoal_Response__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__CancelMotion_SendGoal_Response__init(message_memory);
}

void rover_control__action__CancelMotion_SendGoal_Response__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Response_fini_function(void * message_memory)
{
  rover_control__action__CancelMotion_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__CancelMotion_SendGoal_Response__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__CancelMotion_SendGoal_Response__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Response_message_members = {
  "rover_control__action",  // message namespace
  "CancelMotion_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(rover_control__action__CancelMotion_SendGoal_Response),
  rover_control__action__CancelMotion_SendGoal_Response__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Response_message_member_array,  // message members
  rover_control__action__CancelMotion_SendGoal_Response__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__CancelMotion_SendGoal_Response__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__CancelMotion_SendGoal_Response__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Response_message_type_support_handle = {
  0,
  &rover_control__action__CancelMotion_SendGoal_Response__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_SendGoal_Response)() {
  rover_control__action__CancelMotion_SendGoal_Response__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!rover_control__action__CancelMotion_SendGoal_Response__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    rover_control__action__CancelMotion_SendGoal_Response__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__CancelMotion_SendGoal_Response__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_service_members = {
  "rover_control__action",  // service namespace
  "CancelMotion_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_service_type_support_handle = {
  0,
  &rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_SendGoal)() {
  if (!rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_service_type_support_handle.typesupport_identifier) {
    rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_SendGoal_Response)()->data;
  }

  return &rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "rover_control/action/detail/cancel_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__functions.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__CancelMotion_GetResult_Request__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__CancelMotion_GetResult_Request__init(message_memory);
}

void rover_control__action__CancelMotion_GetResult_Request__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Request_fini_function(void * message_memory)
{
  rover_control__action__CancelMotion_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__CancelMotion_GetResult_Request__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__CancelMotion_GetResult_Request__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Request_message_members = {
  "rover_control__action",  // message namespace
  "CancelMotion_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(rover_control__action__CancelMotion_GetResult_Request),
  rover_control__action__CancelMotion_GetResult_Request__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Request_message_member_array,  // message members
  rover_control__action__CancelMotion_GetResult_Request__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__CancelMotion_GetResult_Request__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__CancelMotion_GetResult_Request__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Request_message_type_support_handle = {
  0,
  &rover_control__action__CancelMotion_GetResult_Request__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_GetResult_Request)() {
  rover_control__action__CancelMotion_GetResult_Request__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!rover_control__action__CancelMotion_GetResult_Request__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    rover_control__action__CancelMotion_GetResult_Request__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__CancelMotion_GetResult_Request__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rover_control/action/detail/cancel_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__functions.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "rover_control/action/cancel_motion.h"
// Member `result`
// already included above
// #include "rover_control/action/detail/cancel_motion__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__CancelMotion_GetResult_Response__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__CancelMotion_GetResult_Response__init(message_memory);
}

void rover_control__action__CancelMotion_GetResult_Response__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Response_fini_function(void * message_memory)
{
  rover_control__action__CancelMotion_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__CancelMotion_GetResult_Response__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__CancelMotion_GetResult_Response__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Response_message_members = {
  "rover_control__action",  // message namespace
  "CancelMotion_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(rover_control__action__CancelMotion_GetResult_Response),
  rover_control__action__CancelMotion_GetResult_Response__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Response_message_member_array,  // message members
  rover_control__action__CancelMotion_GetResult_Response__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__CancelMotion_GetResult_Response__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__CancelMotion_GetResult_Response__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Response_message_type_support_handle = {
  0,
  &rover_control__action__CancelMotion_GetResult_Response__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_GetResult_Response)() {
  rover_control__action__CancelMotion_GetResult_Response__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_Result)();
  if (!rover_control__action__CancelMotion_GetResult_Response__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    rover_control__action__CancelMotion_GetResult_Response__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__CancelMotion_GetResult_Response__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_GetResult_service_members = {
  "rover_control__action",  // service namespace
  "CancelMotion_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_GetResult_service_type_support_handle = {
  0,
  &rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_GetResult)() {
  if (!rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_GetResult_service_type_support_handle.typesupport_identifier) {
    rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_GetResult_Response)()->data;
  }

  return &rover_control__action__detail__cancel_motion__rosidl_typesupport_introspection_c__CancelMotion_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "rover_control/action/detail/cancel_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__functions.h"
// already included above
// #include "rover_control/action/detail/cancel_motion__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "rover_control/action/cancel_motion.h"
// Member `feedback`
// already included above
// #include "rover_control/action/detail/cancel_motion__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__CancelMotion_FeedbackMessage__rosidl_typesupport_introspection_c__CancelMotion_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__CancelMotion_FeedbackMessage__init(message_memory);
}

void rover_control__action__CancelMotion_FeedbackMessage__rosidl_typesupport_introspection_c__CancelMotion_FeedbackMessage_fini_function(void * message_memory)
{
  rover_control__action__CancelMotion_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__CancelMotion_FeedbackMessage__rosidl_typesupport_introspection_c__CancelMotion_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__CancelMotion_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__CancelMotion_FeedbackMessage__rosidl_typesupport_introspection_c__CancelMotion_FeedbackMessage_message_members = {
  "rover_control__action",  // message namespace
  "CancelMotion_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(rover_control__action__CancelMotion_FeedbackMessage),
  rover_control__action__CancelMotion_FeedbackMessage__rosidl_typesupport_introspection_c__CancelMotion_FeedbackMessage_message_member_array,  // message members
  rover_control__action__CancelMotion_FeedbackMessage__rosidl_typesupport_introspection_c__CancelMotion_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__CancelMotion_FeedbackMessage__rosidl_typesupport_introspection_c__CancelMotion_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__CancelMotion_FeedbackMessage__rosidl_typesupport_introspection_c__CancelMotion_FeedbackMessage_message_type_support_handle = {
  0,
  &rover_control__action__CancelMotion_FeedbackMessage__rosidl_typesupport_introspection_c__CancelMotion_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_FeedbackMessage)() {
  rover_control__action__CancelMotion_FeedbackMessage__rosidl_typesupport_introspection_c__CancelMotion_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  rover_control__action__CancelMotion_FeedbackMessage__rosidl_typesupport_introspection_c__CancelMotion_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, CancelMotion_Feedback)();
  if (!rover_control__action__CancelMotion_FeedbackMessage__rosidl_typesupport_introspection_c__CancelMotion_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    rover_control__action__CancelMotion_FeedbackMessage__rosidl_typesupport_introspection_c__CancelMotion_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__CancelMotion_FeedbackMessage__rosidl_typesupport_introspection_c__CancelMotion_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
