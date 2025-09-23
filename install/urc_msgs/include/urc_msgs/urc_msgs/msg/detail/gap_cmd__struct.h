// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from urc_msgs:msg/GapCmd.idl
// generated code does not contain a copyright notice

#ifndef URC_MSGS__MSG__DETAIL__GAP_CMD__STRUCT_H_
#define URC_MSGS__MSG__DETAIL__GAP_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/GapCmd in the package urc_msgs.
typedef struct urc_msgs__msg__GapCmd
{
  uint8_t structure_needs_at_least_one_member;
} urc_msgs__msg__GapCmd;

// Struct for a sequence of urc_msgs__msg__GapCmd.
typedef struct urc_msgs__msg__GapCmd__Sequence
{
  urc_msgs__msg__GapCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} urc_msgs__msg__GapCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // URC_MSGS__MSG__DETAIL__GAP_CMD__STRUCT_H_
