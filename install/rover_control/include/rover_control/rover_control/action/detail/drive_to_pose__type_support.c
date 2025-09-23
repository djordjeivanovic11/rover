// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rover_control:action/DriveToPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rover_control/action/detail/drive_to_pose__rosidl_typesupport_introspection_c.h"
#include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rover_control/action/detail/drive_to_pose__functions.h"
#include "rover_control/action/detail/drive_to_pose__struct.h"


// Include directives for member types
// Member `target_pose`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `target_pose`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `mission_mode`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__DriveToPose_Goal__rosidl_typesupport_introspection_c__DriveToPose_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__DriveToPose_Goal__init(message_memory);
}

void rover_control__action__DriveToPose_Goal__rosidl_typesupport_introspection_c__DriveToPose_Goal_fini_function(void * message_memory)
{
  rover_control__action__DriveToPose_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__DriveToPose_Goal__rosidl_typesupport_introspection_c__DriveToPose_Goal_message_member_array[9] = {
  {
    "target_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Goal, target_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Goal, max_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position_tolerance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Goal, position_tolerance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "orientation_tolerance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Goal, orientation_tolerance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timeout",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Goal, timeout),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "use_visual_servoing",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Goal, use_visual_servoing),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mission_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Goal, mission_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reverse_allowed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Goal, reverse_allowed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "approach_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Goal, approach_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__DriveToPose_Goal__rosidl_typesupport_introspection_c__DriveToPose_Goal_message_members = {
  "rover_control__action",  // message namespace
  "DriveToPose_Goal",  // message name
  9,  // number of fields
  sizeof(rover_control__action__DriveToPose_Goal),
  rover_control__action__DriveToPose_Goal__rosidl_typesupport_introspection_c__DriveToPose_Goal_message_member_array,  // message members
  rover_control__action__DriveToPose_Goal__rosidl_typesupport_introspection_c__DriveToPose_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__DriveToPose_Goal__rosidl_typesupport_introspection_c__DriveToPose_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__DriveToPose_Goal__rosidl_typesupport_introspection_c__DriveToPose_Goal_message_type_support_handle = {
  0,
  &rover_control__action__DriveToPose_Goal__rosidl_typesupport_introspection_c__DriveToPose_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_Goal)() {
  rover_control__action__DriveToPose_Goal__rosidl_typesupport_introspection_c__DriveToPose_Goal_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!rover_control__action__DriveToPose_Goal__rosidl_typesupport_introspection_c__DriveToPose_Goal_message_type_support_handle.typesupport_identifier) {
    rover_control__action__DriveToPose_Goal__rosidl_typesupport_introspection_c__DriveToPose_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__DriveToPose_Goal__rosidl_typesupport_introspection_c__DriveToPose_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rover_control/action/detail/drive_to_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__functions.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__struct.h"


// Include directives for member types
// Member `error_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `final_pose`
// already included above
// #include "geometry_msgs/msg/pose_stamped.h"
// Member `final_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `executed_path`
#include "nav_msgs/msg/path.h"
// Member `executed_path`
#include "nav_msgs/msg/detail/path__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__DriveToPose_Result__rosidl_typesupport_introspection_c__DriveToPose_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__DriveToPose_Result__init(message_memory);
}

void rover_control__action__DriveToPose_Result__rosidl_typesupport_introspection_c__DriveToPose_Result_fini_function(void * message_memory)
{
  rover_control__action__DriveToPose_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__DriveToPose_Result__rosidl_typesupport_introspection_c__DriveToPose_Result_message_member_array[9] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Result, success),  // bytes offset in struct
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
    offsetof(rover_control__action__DriveToPose_Result, result_code),  // bytes offset in struct
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
    offsetof(rover_control__action__DriveToPose_Result, error_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_position_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Result, final_position_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_orientation_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Result, final_orientation_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "execution_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Result, execution_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance_traveled",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Result, distance_traveled),  // bytes offset in struct
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
    offsetof(rover_control__action__DriveToPose_Result, final_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "executed_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Result, executed_path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__DriveToPose_Result__rosidl_typesupport_introspection_c__DriveToPose_Result_message_members = {
  "rover_control__action",  // message namespace
  "DriveToPose_Result",  // message name
  9,  // number of fields
  sizeof(rover_control__action__DriveToPose_Result),
  rover_control__action__DriveToPose_Result__rosidl_typesupport_introspection_c__DriveToPose_Result_message_member_array,  // message members
  rover_control__action__DriveToPose_Result__rosidl_typesupport_introspection_c__DriveToPose_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__DriveToPose_Result__rosidl_typesupport_introspection_c__DriveToPose_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__DriveToPose_Result__rosidl_typesupport_introspection_c__DriveToPose_Result_message_type_support_handle = {
  0,
  &rover_control__action__DriveToPose_Result__rosidl_typesupport_introspection_c__DriveToPose_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_Result)() {
  rover_control__action__DriveToPose_Result__rosidl_typesupport_introspection_c__DriveToPose_Result_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  rover_control__action__DriveToPose_Result__rosidl_typesupport_introspection_c__DriveToPose_Result_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav_msgs, msg, Path)();
  if (!rover_control__action__DriveToPose_Result__rosidl_typesupport_introspection_c__DriveToPose_Result_message_type_support_handle.typesupport_identifier) {
    rover_control__action__DriveToPose_Result__rosidl_typesupport_introspection_c__DriveToPose_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__DriveToPose_Result__rosidl_typesupport_introspection_c__DriveToPose_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rover_control/action/detail/drive_to_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__functions.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__struct.h"


// Include directives for member types
// Member `current_state`
// Member `status_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `current_pose`
// already included above
// #include "geometry_msgs/msg/pose_stamped.h"
// Member `current_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__DriveToPose_Feedback__rosidl_typesupport_introspection_c__DriveToPose_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__DriveToPose_Feedback__init(message_memory);
}

void rover_control__action__DriveToPose_Feedback__rosidl_typesupport_introspection_c__DriveToPose_Feedback_fini_function(void * message_memory)
{
  rover_control__action__DriveToPose_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__DriveToPose_Feedback__rosidl_typesupport_introspection_c__DriveToPose_Feedback_message_member_array[9] = {
  {
    "current_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Feedback, current_state),  // bytes offset in struct
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
    offsetof(rover_control__action__DriveToPose_Feedback, progress_percentage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Feedback, current_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "estimated_time_remaining",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Feedback, estimated_time_remaining),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance_remaining",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Feedback, distance_remaining),  // bytes offset in struct
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
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Feedback, current_velocity),  // bytes offset in struct
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
    offsetof(rover_control__action__DriveToPose_Feedback, status_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stuck_detected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Feedback, stuck_detected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "autonomy_time_remaining",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_Feedback, autonomy_time_remaining),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__DriveToPose_Feedback__rosidl_typesupport_introspection_c__DriveToPose_Feedback_message_members = {
  "rover_control__action",  // message namespace
  "DriveToPose_Feedback",  // message name
  9,  // number of fields
  sizeof(rover_control__action__DriveToPose_Feedback),
  rover_control__action__DriveToPose_Feedback__rosidl_typesupport_introspection_c__DriveToPose_Feedback_message_member_array,  // message members
  rover_control__action__DriveToPose_Feedback__rosidl_typesupport_introspection_c__DriveToPose_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__DriveToPose_Feedback__rosidl_typesupport_introspection_c__DriveToPose_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__DriveToPose_Feedback__rosidl_typesupport_introspection_c__DriveToPose_Feedback_message_type_support_handle = {
  0,
  &rover_control__action__DriveToPose_Feedback__rosidl_typesupport_introspection_c__DriveToPose_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_Feedback)() {
  rover_control__action__DriveToPose_Feedback__rosidl_typesupport_introspection_c__DriveToPose_Feedback_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!rover_control__action__DriveToPose_Feedback__rosidl_typesupport_introspection_c__DriveToPose_Feedback_message_type_support_handle.typesupport_identifier) {
    rover_control__action__DriveToPose_Feedback__rosidl_typesupport_introspection_c__DriveToPose_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__DriveToPose_Feedback__rosidl_typesupport_introspection_c__DriveToPose_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rover_control/action/detail/drive_to_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__functions.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "rover_control/action/drive_to_pose.h"
// Member `goal`
// already included above
// #include "rover_control/action/detail/drive_to_pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__DriveToPose_SendGoal_Request__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__DriveToPose_SendGoal_Request__init(message_memory);
}

void rover_control__action__DriveToPose_SendGoal_Request__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_fini_function(void * message_memory)
{
  rover_control__action__DriveToPose_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__DriveToPose_SendGoal_Request__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_SendGoal_Request, goal_id),  // bytes offset in struct
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
    offsetof(rover_control__action__DriveToPose_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__DriveToPose_SendGoal_Request__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_message_members = {
  "rover_control__action",  // message namespace
  "DriveToPose_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(rover_control__action__DriveToPose_SendGoal_Request),
  rover_control__action__DriveToPose_SendGoal_Request__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_message_member_array,  // message members
  rover_control__action__DriveToPose_SendGoal_Request__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__DriveToPose_SendGoal_Request__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__DriveToPose_SendGoal_Request__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_message_type_support_handle = {
  0,
  &rover_control__action__DriveToPose_SendGoal_Request__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_SendGoal_Request)() {
  rover_control__action__DriveToPose_SendGoal_Request__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  rover_control__action__DriveToPose_SendGoal_Request__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_Goal)();
  if (!rover_control__action__DriveToPose_SendGoal_Request__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    rover_control__action__DriveToPose_SendGoal_Request__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__DriveToPose_SendGoal_Request__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rover_control/action/detail/drive_to_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__functions.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__DriveToPose_SendGoal_Response__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__DriveToPose_SendGoal_Response__init(message_memory);
}

void rover_control__action__DriveToPose_SendGoal_Response__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Response_fini_function(void * message_memory)
{
  rover_control__action__DriveToPose_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__DriveToPose_SendGoal_Response__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_SendGoal_Response, accepted),  // bytes offset in struct
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
    offsetof(rover_control__action__DriveToPose_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__DriveToPose_SendGoal_Response__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Response_message_members = {
  "rover_control__action",  // message namespace
  "DriveToPose_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(rover_control__action__DriveToPose_SendGoal_Response),
  rover_control__action__DriveToPose_SendGoal_Response__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Response_message_member_array,  // message members
  rover_control__action__DriveToPose_SendGoal_Response__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__DriveToPose_SendGoal_Response__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__DriveToPose_SendGoal_Response__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Response_message_type_support_handle = {
  0,
  &rover_control__action__DriveToPose_SendGoal_Response__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_SendGoal_Response)() {
  rover_control__action__DriveToPose_SendGoal_Response__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!rover_control__action__DriveToPose_SendGoal_Response__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    rover_control__action__DriveToPose_SendGoal_Response__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__DriveToPose_SendGoal_Response__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_service_members = {
  "rover_control__action",  // service namespace
  "DriveToPose_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_service_type_support_handle = {
  0,
  &rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_SendGoal)() {
  if (!rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_service_type_support_handle.typesupport_identifier) {
    rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_SendGoal_Response)()->data;
  }

  return &rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "rover_control/action/detail/drive_to_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__functions.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__struct.h"


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

void rover_control__action__DriveToPose_GetResult_Request__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__DriveToPose_GetResult_Request__init(message_memory);
}

void rover_control__action__DriveToPose_GetResult_Request__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Request_fini_function(void * message_memory)
{
  rover_control__action__DriveToPose_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__DriveToPose_GetResult_Request__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__DriveToPose_GetResult_Request__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Request_message_members = {
  "rover_control__action",  // message namespace
  "DriveToPose_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(rover_control__action__DriveToPose_GetResult_Request),
  rover_control__action__DriveToPose_GetResult_Request__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Request_message_member_array,  // message members
  rover_control__action__DriveToPose_GetResult_Request__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__DriveToPose_GetResult_Request__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__DriveToPose_GetResult_Request__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Request_message_type_support_handle = {
  0,
  &rover_control__action__DriveToPose_GetResult_Request__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_GetResult_Request)() {
  rover_control__action__DriveToPose_GetResult_Request__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!rover_control__action__DriveToPose_GetResult_Request__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    rover_control__action__DriveToPose_GetResult_Request__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__DriveToPose_GetResult_Request__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rover_control/action/detail/drive_to_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__functions.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "rover_control/action/drive_to_pose.h"
// Member `result`
// already included above
// #include "rover_control/action/detail/drive_to_pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__DriveToPose_GetResult_Response__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__DriveToPose_GetResult_Response__init(message_memory);
}

void rover_control__action__DriveToPose_GetResult_Response__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Response_fini_function(void * message_memory)
{
  rover_control__action__DriveToPose_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__DriveToPose_GetResult_Response__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_GetResult_Response, status),  // bytes offset in struct
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
    offsetof(rover_control__action__DriveToPose_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__DriveToPose_GetResult_Response__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Response_message_members = {
  "rover_control__action",  // message namespace
  "DriveToPose_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(rover_control__action__DriveToPose_GetResult_Response),
  rover_control__action__DriveToPose_GetResult_Response__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Response_message_member_array,  // message members
  rover_control__action__DriveToPose_GetResult_Response__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__DriveToPose_GetResult_Response__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__DriveToPose_GetResult_Response__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Response_message_type_support_handle = {
  0,
  &rover_control__action__DriveToPose_GetResult_Response__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_GetResult_Response)() {
  rover_control__action__DriveToPose_GetResult_Response__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_Result)();
  if (!rover_control__action__DriveToPose_GetResult_Response__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    rover_control__action__DriveToPose_GetResult_Response__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__DriveToPose_GetResult_Response__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_GetResult_service_members = {
  "rover_control__action",  // service namespace
  "DriveToPose_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_GetResult_service_type_support_handle = {
  0,
  &rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_GetResult)() {
  if (!rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_GetResult_service_type_support_handle.typesupport_identifier) {
    rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_GetResult_Response)()->data;
  }

  return &rover_control__action__detail__drive_to_pose__rosidl_typesupport_introspection_c__DriveToPose_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "rover_control/action/detail/drive_to_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rover_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__functions.h"
// already included above
// #include "rover_control/action/detail/drive_to_pose__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "rover_control/action/drive_to_pose.h"
// Member `feedback`
// already included above
// #include "rover_control/action/detail/drive_to_pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rover_control__action__DriveToPose_FeedbackMessage__rosidl_typesupport_introspection_c__DriveToPose_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rover_control__action__DriveToPose_FeedbackMessage__init(message_memory);
}

void rover_control__action__DriveToPose_FeedbackMessage__rosidl_typesupport_introspection_c__DriveToPose_FeedbackMessage_fini_function(void * message_memory)
{
  rover_control__action__DriveToPose_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rover_control__action__DriveToPose_FeedbackMessage__rosidl_typesupport_introspection_c__DriveToPose_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rover_control__action__DriveToPose_FeedbackMessage, goal_id),  // bytes offset in struct
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
    offsetof(rover_control__action__DriveToPose_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rover_control__action__DriveToPose_FeedbackMessage__rosidl_typesupport_introspection_c__DriveToPose_FeedbackMessage_message_members = {
  "rover_control__action",  // message namespace
  "DriveToPose_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(rover_control__action__DriveToPose_FeedbackMessage),
  rover_control__action__DriveToPose_FeedbackMessage__rosidl_typesupport_introspection_c__DriveToPose_FeedbackMessage_message_member_array,  // message members
  rover_control__action__DriveToPose_FeedbackMessage__rosidl_typesupport_introspection_c__DriveToPose_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  rover_control__action__DriveToPose_FeedbackMessage__rosidl_typesupport_introspection_c__DriveToPose_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rover_control__action__DriveToPose_FeedbackMessage__rosidl_typesupport_introspection_c__DriveToPose_FeedbackMessage_message_type_support_handle = {
  0,
  &rover_control__action__DriveToPose_FeedbackMessage__rosidl_typesupport_introspection_c__DriveToPose_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rover_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_FeedbackMessage)() {
  rover_control__action__DriveToPose_FeedbackMessage__rosidl_typesupport_introspection_c__DriveToPose_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  rover_control__action__DriveToPose_FeedbackMessage__rosidl_typesupport_introspection_c__DriveToPose_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rover_control, action, DriveToPose_Feedback)();
  if (!rover_control__action__DriveToPose_FeedbackMessage__rosidl_typesupport_introspection_c__DriveToPose_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    rover_control__action__DriveToPose_FeedbackMessage__rosidl_typesupport_introspection_c__DriveToPose_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rover_control__action__DriveToPose_FeedbackMessage__rosidl_typesupport_introspection_c__DriveToPose_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
