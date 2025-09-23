// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from urc_msgs:msg/ScienceData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "urc_msgs/msg/detail/science_data__rosidl_typesupport_introspection_c.h"
#include "urc_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "urc_msgs/msg/detail/science_data__functions.h"
#include "urc_msgs/msg/detail/science_data__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `sample_id`
// Member `detected_compounds`
#include "rosidl_runtime_c/string_functions.h"
// Member `sample_location`
#include "geometry_msgs/msg/point.h"
// Member `sample_location`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__ScienceData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  urc_msgs__msg__ScienceData__init(message_memory);
}

void urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__ScienceData_fini_function(void * message_memory)
{
  urc_msgs__msg__ScienceData__fini(message_memory);
}

size_t urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__size_function__ScienceData__detected_compounds(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__get_const_function__ScienceData__detected_compounds(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__get_function__ScienceData__detected_compounds(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__fetch_function__ScienceData__detected_compounds(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__get_const_function__ScienceData__detected_compounds(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__assign_function__ScienceData__detected_compounds(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__get_function__ScienceData__detected_compounds(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__resize_function__ScienceData__detected_compounds(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__ScienceData_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(urc_msgs__msg__ScienceData, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sample_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(urc_msgs__msg__ScienceData, sample_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sample_location",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(urc_msgs__msg__ScienceData, sample_location),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ph_level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(urc_msgs__msg__ScienceData, ph_level),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(urc_msgs__msg__ScienceData, temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "moisture",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(urc_msgs__msg__ScienceData, moisture),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "detected_compounds",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(urc_msgs__msg__ScienceData, detected_compounds),  // bytes offset in struct
    NULL,  // default value
    urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__size_function__ScienceData__detected_compounds,  // size() function pointer
    urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__get_const_function__ScienceData__detected_compounds,  // get_const(index) function pointer
    urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__get_function__ScienceData__detected_compounds,  // get(index) function pointer
    urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__fetch_function__ScienceData__detected_compounds,  // fetch(index, &value) function pointer
    urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__assign_function__ScienceData__detected_compounds,  // assign(index, value) function pointer
    urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__resize_function__ScienceData__detected_compounds  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__ScienceData_message_members = {
  "urc_msgs__msg",  // message namespace
  "ScienceData",  // message name
  7,  // number of fields
  sizeof(urc_msgs__msg__ScienceData),
  urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__ScienceData_message_member_array,  // message members
  urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__ScienceData_init_function,  // function to initialize message memory (memory has to be allocated)
  urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__ScienceData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__ScienceData_message_type_support_handle = {
  0,
  &urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__ScienceData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_urc_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, urc_msgs, msg, ScienceData)() {
  urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__ScienceData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__ScienceData_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__ScienceData_message_type_support_handle.typesupport_identifier) {
    urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__ScienceData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &urc_msgs__msg__ScienceData__rosidl_typesupport_introspection_c__ScienceData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
