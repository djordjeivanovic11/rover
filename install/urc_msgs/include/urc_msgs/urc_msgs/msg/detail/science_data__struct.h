// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from urc_msgs:msg/ScienceData.idl
// generated code does not contain a copyright notice

#ifndef URC_MSGS__MSG__DETAIL__SCIENCE_DATA__STRUCT_H_
#define URC_MSGS__MSG__DETAIL__SCIENCE_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'sample_id'
// Member 'detected_compounds'
#include "rosidl_runtime_c/string.h"
// Member 'sample_location'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/ScienceData in the package urc_msgs.
/**
  * Science data message
 */
typedef struct urc_msgs__msg__ScienceData
{
  std_msgs__msg__Header header;
  /// Sample information
  rosidl_runtime_c__String sample_id;
  geometry_msgs__msg__Point sample_location;
  /// Sensor readings
  float ph_level;
  float temperature;
  float moisture;
  rosidl_runtime_c__String__Sequence detected_compounds;
} urc_msgs__msg__ScienceData;

// Struct for a sequence of urc_msgs__msg__ScienceData.
typedef struct urc_msgs__msg__ScienceData__Sequence
{
  urc_msgs__msg__ScienceData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} urc_msgs__msg__ScienceData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // URC_MSGS__MSG__DETAIL__SCIENCE_DATA__STRUCT_H_
