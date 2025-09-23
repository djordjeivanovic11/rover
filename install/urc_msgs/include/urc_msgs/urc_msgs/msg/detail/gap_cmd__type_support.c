// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from urc_msgs:msg/GapCmd.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "urc_msgs/msg/detail/gap_cmd__rosidl_typesupport_introspection_c.h"
#include "urc_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "urc_msgs/msg/detail/gap_cmd__functions.h"
#include "urc_msgs/msg/detail/gap_cmd__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void urc_msgs__msg__GapCmd__rosidl_typesupport_introspection_c__GapCmd_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  urc_msgs__msg__GapCmd__init(message_memory);
}

void urc_msgs__msg__GapCmd__rosidl_typesupport_introspection_c__GapCmd_fini_function(void * message_memory)
{
  urc_msgs__msg__GapCmd__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember urc_msgs__msg__GapCmd__rosidl_typesupport_introspection_c__GapCmd_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(urc_msgs__msg__GapCmd, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers urc_msgs__msg__GapCmd__rosidl_typesupport_introspection_c__GapCmd_message_members = {
  "urc_msgs__msg",  // message namespace
  "GapCmd",  // message name
  1,  // number of fields
  sizeof(urc_msgs__msg__GapCmd),
  urc_msgs__msg__GapCmd__rosidl_typesupport_introspection_c__GapCmd_message_member_array,  // message members
  urc_msgs__msg__GapCmd__rosidl_typesupport_introspection_c__GapCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  urc_msgs__msg__GapCmd__rosidl_typesupport_introspection_c__GapCmd_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t urc_msgs__msg__GapCmd__rosidl_typesupport_introspection_c__GapCmd_message_type_support_handle = {
  0,
  &urc_msgs__msg__GapCmd__rosidl_typesupport_introspection_c__GapCmd_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_urc_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, urc_msgs, msg, GapCmd)() {
  if (!urc_msgs__msg__GapCmd__rosidl_typesupport_introspection_c__GapCmd_message_type_support_handle.typesupport_identifier) {
    urc_msgs__msg__GapCmd__rosidl_typesupport_introspection_c__GapCmd_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &urc_msgs__msg__GapCmd__rosidl_typesupport_introspection_c__GapCmd_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
