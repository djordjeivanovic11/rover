// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from arm_control:action/PickAndPlace.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "arm_control/action/detail/pick_and_place__rosidl_typesupport_introspection_c.h"
#include "arm_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "arm_control/action/detail/pick_and_place__functions.h"
#include "arm_control/action/detail/pick_and_place__struct.h"


// Include directives for member types
// Member `pick_pose`
// Member `place_pose`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `pick_pose`
// Member `place_pose`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `approach_offset`
// Member `retreat_offset`
#include "geometry_msgs/msg/vector3.h"
// Member `approach_offset`
// Member `retreat_offset`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"
// Member `grasp_strategy`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arm_control__action__PickAndPlace_Goal__init(message_memory);
}

void arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_fini_function(void * message_memory)
{
  arm_control__action__PickAndPlace_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_message_member_array[13] = {
  {
    "pick_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Goal, pick_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "place_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Goal, place_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "approach_offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Goal, approach_offset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "retreat_offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Goal, retreat_offset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "grasp_force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Goal, grasp_force),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "grasp_timeout",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Goal, grasp_timeout),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "grasp_strategy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Goal, grasp_strategy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "verify_grasp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Goal, verify_grasp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lift_height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Goal, lift_height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "place_force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Goal, place_force),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gentle_place",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Goal, gentle_place),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity_scaling",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Goal, velocity_scaling),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acceleration_scaling",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Goal, acceleration_scaling),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_message_members = {
  "arm_control__action",  // message namespace
  "PickAndPlace_Goal",  // message name
  13,  // number of fields
  sizeof(arm_control__action__PickAndPlace_Goal),
  arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_message_member_array,  // message members
  arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_message_type_support_handle = {
  0,
  &arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arm_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_Goal)() {
  arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_message_type_support_handle.typesupport_identifier) {
    arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arm_control__action__PickAndPlace_Goal__rosidl_typesupport_introspection_c__PickAndPlace_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "arm_control/action/detail/pick_and_place__rosidl_typesupport_introspection_c.h"
// already included above
// #include "arm_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__functions.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__struct.h"


// Include directives for member types
// Member `error_message`
// Member `failure_phase`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `final_object_pose`
// already included above
// #include "geometry_msgs/msg/pose_stamped.h"
// Member `final_object_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `final_joint_state`
#include "sensor_msgs/msg/joint_state.h"
// Member `final_joint_state`
#include "sensor_msgs/msg/detail/joint_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void arm_control__action__PickAndPlace_Result__rosidl_typesupport_introspection_c__PickAndPlace_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arm_control__action__PickAndPlace_Result__init(message_memory);
}

void arm_control__action__PickAndPlace_Result__rosidl_typesupport_introspection_c__PickAndPlace_Result_fini_function(void * message_memory)
{
  arm_control__action__PickAndPlace_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember arm_control__action__PickAndPlace_Result__rosidl_typesupport_introspection_c__PickAndPlace_Result_message_member_array[10] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Result, success),  // bytes offset in struct
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
    offsetof(arm_control__action__PickAndPlace_Result, error_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Result, error_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "failure_phase",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Result, failure_phase),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "grasp_successful",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Result, grasp_successful),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "place_successful",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Result, place_successful),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "grasp_force_achieved",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Result, grasp_force_achieved),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_object_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Result, final_object_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "total_execution_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Result, total_execution_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_joint_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Result, final_joint_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arm_control__action__PickAndPlace_Result__rosidl_typesupport_introspection_c__PickAndPlace_Result_message_members = {
  "arm_control__action",  // message namespace
  "PickAndPlace_Result",  // message name
  10,  // number of fields
  sizeof(arm_control__action__PickAndPlace_Result),
  arm_control__action__PickAndPlace_Result__rosidl_typesupport_introspection_c__PickAndPlace_Result_message_member_array,  // message members
  arm_control__action__PickAndPlace_Result__rosidl_typesupport_introspection_c__PickAndPlace_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  arm_control__action__PickAndPlace_Result__rosidl_typesupport_introspection_c__PickAndPlace_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arm_control__action__PickAndPlace_Result__rosidl_typesupport_introspection_c__PickAndPlace_Result_message_type_support_handle = {
  0,
  &arm_control__action__PickAndPlace_Result__rosidl_typesupport_introspection_c__PickAndPlace_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arm_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_Result)() {
  arm_control__action__PickAndPlace_Result__rosidl_typesupport_introspection_c__PickAndPlace_Result_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  arm_control__action__PickAndPlace_Result__rosidl_typesupport_introspection_c__PickAndPlace_Result_message_member_array[9].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, JointState)();
  if (!arm_control__action__PickAndPlace_Result__rosidl_typesupport_introspection_c__PickAndPlace_Result_message_type_support_handle.typesupport_identifier) {
    arm_control__action__PickAndPlace_Result__rosidl_typesupport_introspection_c__PickAndPlace_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arm_control__action__PickAndPlace_Result__rosidl_typesupport_introspection_c__PickAndPlace_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "arm_control/action/detail/pick_and_place__rosidl_typesupport_introspection_c.h"
// already included above
// #include "arm_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__functions.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__struct.h"


// Include directives for member types
// Member `current_phase`
// Member `grasp_status`
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

void arm_control__action__PickAndPlace_Feedback__rosidl_typesupport_introspection_c__PickAndPlace_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arm_control__action__PickAndPlace_Feedback__init(message_memory);
}

void arm_control__action__PickAndPlace_Feedback__rosidl_typesupport_introspection_c__PickAndPlace_Feedback_fini_function(void * message_memory)
{
  arm_control__action__PickAndPlace_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember arm_control__action__PickAndPlace_Feedback__rosidl_typesupport_introspection_c__PickAndPlace_Feedback_message_member_array[9] = {
  {
    "current_phase",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Feedback, current_phase),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "phase_progress",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Feedback, phase_progress),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "overall_progress",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Feedback, overall_progress),  // bytes offset in struct
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
    offsetof(arm_control__action__PickAndPlace_Feedback, current_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_grasp_force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Feedback, current_grasp_force),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_detected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Feedback, object_detected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "grasp_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_Feedback, grasp_status),  // bytes offset in struct
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
    offsetof(arm_control__action__PickAndPlace_Feedback, status_message),  // bytes offset in struct
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
    offsetof(arm_control__action__PickAndPlace_Feedback, estimated_time_remaining),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arm_control__action__PickAndPlace_Feedback__rosidl_typesupport_introspection_c__PickAndPlace_Feedback_message_members = {
  "arm_control__action",  // message namespace
  "PickAndPlace_Feedback",  // message name
  9,  // number of fields
  sizeof(arm_control__action__PickAndPlace_Feedback),
  arm_control__action__PickAndPlace_Feedback__rosidl_typesupport_introspection_c__PickAndPlace_Feedback_message_member_array,  // message members
  arm_control__action__PickAndPlace_Feedback__rosidl_typesupport_introspection_c__PickAndPlace_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  arm_control__action__PickAndPlace_Feedback__rosidl_typesupport_introspection_c__PickAndPlace_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arm_control__action__PickAndPlace_Feedback__rosidl_typesupport_introspection_c__PickAndPlace_Feedback_message_type_support_handle = {
  0,
  &arm_control__action__PickAndPlace_Feedback__rosidl_typesupport_introspection_c__PickAndPlace_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arm_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_Feedback)() {
  arm_control__action__PickAndPlace_Feedback__rosidl_typesupport_introspection_c__PickAndPlace_Feedback_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!arm_control__action__PickAndPlace_Feedback__rosidl_typesupport_introspection_c__PickAndPlace_Feedback_message_type_support_handle.typesupport_identifier) {
    arm_control__action__PickAndPlace_Feedback__rosidl_typesupport_introspection_c__PickAndPlace_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arm_control__action__PickAndPlace_Feedback__rosidl_typesupport_introspection_c__PickAndPlace_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "arm_control/action/detail/pick_and_place__rosidl_typesupport_introspection_c.h"
// already included above
// #include "arm_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__functions.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "arm_control/action/pick_and_place.h"
// Member `goal`
// already included above
// #include "arm_control/action/detail/pick_and_place__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void arm_control__action__PickAndPlace_SendGoal_Request__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arm_control__action__PickAndPlace_SendGoal_Request__init(message_memory);
}

void arm_control__action__PickAndPlace_SendGoal_Request__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_fini_function(void * message_memory)
{
  arm_control__action__PickAndPlace_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember arm_control__action__PickAndPlace_SendGoal_Request__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_SendGoal_Request, goal_id),  // bytes offset in struct
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
    offsetof(arm_control__action__PickAndPlace_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arm_control__action__PickAndPlace_SendGoal_Request__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_message_members = {
  "arm_control__action",  // message namespace
  "PickAndPlace_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(arm_control__action__PickAndPlace_SendGoal_Request),
  arm_control__action__PickAndPlace_SendGoal_Request__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_message_member_array,  // message members
  arm_control__action__PickAndPlace_SendGoal_Request__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  arm_control__action__PickAndPlace_SendGoal_Request__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arm_control__action__PickAndPlace_SendGoal_Request__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_message_type_support_handle = {
  0,
  &arm_control__action__PickAndPlace_SendGoal_Request__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arm_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_SendGoal_Request)() {
  arm_control__action__PickAndPlace_SendGoal_Request__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  arm_control__action__PickAndPlace_SendGoal_Request__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_Goal)();
  if (!arm_control__action__PickAndPlace_SendGoal_Request__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    arm_control__action__PickAndPlace_SendGoal_Request__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arm_control__action__PickAndPlace_SendGoal_Request__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "arm_control/action/detail/pick_and_place__rosidl_typesupport_introspection_c.h"
// already included above
// #include "arm_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__functions.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void arm_control__action__PickAndPlace_SendGoal_Response__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arm_control__action__PickAndPlace_SendGoal_Response__init(message_memory);
}

void arm_control__action__PickAndPlace_SendGoal_Response__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Response_fini_function(void * message_memory)
{
  arm_control__action__PickAndPlace_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember arm_control__action__PickAndPlace_SendGoal_Response__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_SendGoal_Response, accepted),  // bytes offset in struct
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
    offsetof(arm_control__action__PickAndPlace_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arm_control__action__PickAndPlace_SendGoal_Response__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Response_message_members = {
  "arm_control__action",  // message namespace
  "PickAndPlace_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(arm_control__action__PickAndPlace_SendGoal_Response),
  arm_control__action__PickAndPlace_SendGoal_Response__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Response_message_member_array,  // message members
  arm_control__action__PickAndPlace_SendGoal_Response__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  arm_control__action__PickAndPlace_SendGoal_Response__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arm_control__action__PickAndPlace_SendGoal_Response__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Response_message_type_support_handle = {
  0,
  &arm_control__action__PickAndPlace_SendGoal_Response__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arm_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_SendGoal_Response)() {
  arm_control__action__PickAndPlace_SendGoal_Response__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!arm_control__action__PickAndPlace_SendGoal_Response__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    arm_control__action__PickAndPlace_SendGoal_Response__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arm_control__action__PickAndPlace_SendGoal_Response__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "arm_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_service_members = {
  "arm_control__action",  // service namespace
  "PickAndPlace_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_service_type_support_handle = {
  0,
  &arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arm_control
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_SendGoal)() {
  if (!arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_service_type_support_handle.typesupport_identifier) {
    arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_SendGoal_Response)()->data;
  }

  return &arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "arm_control/action/detail/pick_and_place__rosidl_typesupport_introspection_c.h"
// already included above
// #include "arm_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__functions.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__struct.h"


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

void arm_control__action__PickAndPlace_GetResult_Request__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arm_control__action__PickAndPlace_GetResult_Request__init(message_memory);
}

void arm_control__action__PickAndPlace_GetResult_Request__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Request_fini_function(void * message_memory)
{
  arm_control__action__PickAndPlace_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember arm_control__action__PickAndPlace_GetResult_Request__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arm_control__action__PickAndPlace_GetResult_Request__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Request_message_members = {
  "arm_control__action",  // message namespace
  "PickAndPlace_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(arm_control__action__PickAndPlace_GetResult_Request),
  arm_control__action__PickAndPlace_GetResult_Request__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Request_message_member_array,  // message members
  arm_control__action__PickAndPlace_GetResult_Request__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  arm_control__action__PickAndPlace_GetResult_Request__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arm_control__action__PickAndPlace_GetResult_Request__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Request_message_type_support_handle = {
  0,
  &arm_control__action__PickAndPlace_GetResult_Request__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arm_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_GetResult_Request)() {
  arm_control__action__PickAndPlace_GetResult_Request__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!arm_control__action__PickAndPlace_GetResult_Request__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    arm_control__action__PickAndPlace_GetResult_Request__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arm_control__action__PickAndPlace_GetResult_Request__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "arm_control/action/detail/pick_and_place__rosidl_typesupport_introspection_c.h"
// already included above
// #include "arm_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__functions.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "arm_control/action/pick_and_place.h"
// Member `result`
// already included above
// #include "arm_control/action/detail/pick_and_place__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void arm_control__action__PickAndPlace_GetResult_Response__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arm_control__action__PickAndPlace_GetResult_Response__init(message_memory);
}

void arm_control__action__PickAndPlace_GetResult_Response__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Response_fini_function(void * message_memory)
{
  arm_control__action__PickAndPlace_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember arm_control__action__PickAndPlace_GetResult_Response__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_GetResult_Response, status),  // bytes offset in struct
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
    offsetof(arm_control__action__PickAndPlace_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arm_control__action__PickAndPlace_GetResult_Response__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Response_message_members = {
  "arm_control__action",  // message namespace
  "PickAndPlace_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(arm_control__action__PickAndPlace_GetResult_Response),
  arm_control__action__PickAndPlace_GetResult_Response__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Response_message_member_array,  // message members
  arm_control__action__PickAndPlace_GetResult_Response__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  arm_control__action__PickAndPlace_GetResult_Response__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arm_control__action__PickAndPlace_GetResult_Response__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Response_message_type_support_handle = {
  0,
  &arm_control__action__PickAndPlace_GetResult_Response__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arm_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_GetResult_Response)() {
  arm_control__action__PickAndPlace_GetResult_Response__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_Result)();
  if (!arm_control__action__PickAndPlace_GetResult_Response__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    arm_control__action__PickAndPlace_GetResult_Response__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arm_control__action__PickAndPlace_GetResult_Response__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "arm_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_service_members = {
  "arm_control__action",  // service namespace
  "PickAndPlace_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_service_type_support_handle = {
  0,
  &arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arm_control
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_GetResult)() {
  if (!arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_service_type_support_handle.typesupport_identifier) {
    arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_GetResult_Response)()->data;
  }

  return &arm_control__action__detail__pick_and_place__rosidl_typesupport_introspection_c__PickAndPlace_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "arm_control/action/detail/pick_and_place__rosidl_typesupport_introspection_c.h"
// already included above
// #include "arm_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__functions.h"
// already included above
// #include "arm_control/action/detail/pick_and_place__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "arm_control/action/pick_and_place.h"
// Member `feedback`
// already included above
// #include "arm_control/action/detail/pick_and_place__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void arm_control__action__PickAndPlace_FeedbackMessage__rosidl_typesupport_introspection_c__PickAndPlace_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arm_control__action__PickAndPlace_FeedbackMessage__init(message_memory);
}

void arm_control__action__PickAndPlace_FeedbackMessage__rosidl_typesupport_introspection_c__PickAndPlace_FeedbackMessage_fini_function(void * message_memory)
{
  arm_control__action__PickAndPlace_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember arm_control__action__PickAndPlace_FeedbackMessage__rosidl_typesupport_introspection_c__PickAndPlace_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arm_control__action__PickAndPlace_FeedbackMessage, goal_id),  // bytes offset in struct
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
    offsetof(arm_control__action__PickAndPlace_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arm_control__action__PickAndPlace_FeedbackMessage__rosidl_typesupport_introspection_c__PickAndPlace_FeedbackMessage_message_members = {
  "arm_control__action",  // message namespace
  "PickAndPlace_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(arm_control__action__PickAndPlace_FeedbackMessage),
  arm_control__action__PickAndPlace_FeedbackMessage__rosidl_typesupport_introspection_c__PickAndPlace_FeedbackMessage_message_member_array,  // message members
  arm_control__action__PickAndPlace_FeedbackMessage__rosidl_typesupport_introspection_c__PickAndPlace_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  arm_control__action__PickAndPlace_FeedbackMessage__rosidl_typesupport_introspection_c__PickAndPlace_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arm_control__action__PickAndPlace_FeedbackMessage__rosidl_typesupport_introspection_c__PickAndPlace_FeedbackMessage_message_type_support_handle = {
  0,
  &arm_control__action__PickAndPlace_FeedbackMessage__rosidl_typesupport_introspection_c__PickAndPlace_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arm_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_FeedbackMessage)() {
  arm_control__action__PickAndPlace_FeedbackMessage__rosidl_typesupport_introspection_c__PickAndPlace_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  arm_control__action__PickAndPlace_FeedbackMessage__rosidl_typesupport_introspection_c__PickAndPlace_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arm_control, action, PickAndPlace_Feedback)();
  if (!arm_control__action__PickAndPlace_FeedbackMessage__rosidl_typesupport_introspection_c__PickAndPlace_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    arm_control__action__PickAndPlace_FeedbackMessage__rosidl_typesupport_introspection_c__PickAndPlace_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arm_control__action__PickAndPlace_FeedbackMessage__rosidl_typesupport_introspection_c__PickAndPlace_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
