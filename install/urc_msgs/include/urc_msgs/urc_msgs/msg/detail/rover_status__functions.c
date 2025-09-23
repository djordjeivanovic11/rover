// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from urc_msgs:msg/RoverStatus.idl
// generated code does not contain a copyright notice
#include "urc_msgs/msg/detail/rover_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `current_mode`
#include "rosidl_runtime_c/string_functions.h"
// Member `current_pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `current_velocity`
#include "geometry_msgs/msg/detail/twist__functions.h"

bool
urc_msgs__msg__RoverStatus__init(urc_msgs__msg__RoverStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    urc_msgs__msg__RoverStatus__fini(msg);
    return false;
  }
  // battery_voltage
  // battery_percentage
  // motors_enabled
  // emergency_stop
  // current_mode
  if (!rosidl_runtime_c__String__init(&msg->current_mode)) {
    urc_msgs__msg__RoverStatus__fini(msg);
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__Pose__init(&msg->current_pose)) {
    urc_msgs__msg__RoverStatus__fini(msg);
    return false;
  }
  // current_velocity
  if (!geometry_msgs__msg__Twist__init(&msg->current_velocity)) {
    urc_msgs__msg__RoverStatus__fini(msg);
    return false;
  }
  return true;
}

void
urc_msgs__msg__RoverStatus__fini(urc_msgs__msg__RoverStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // battery_voltage
  // battery_percentage
  // motors_enabled
  // emergency_stop
  // current_mode
  rosidl_runtime_c__String__fini(&msg->current_mode);
  // current_pose
  geometry_msgs__msg__Pose__fini(&msg->current_pose);
  // current_velocity
  geometry_msgs__msg__Twist__fini(&msg->current_velocity);
}

bool
urc_msgs__msg__RoverStatus__are_equal(const urc_msgs__msg__RoverStatus * lhs, const urc_msgs__msg__RoverStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // battery_percentage
  if (lhs->battery_percentage != rhs->battery_percentage) {
    return false;
  }
  // motors_enabled
  if (lhs->motors_enabled != rhs->motors_enabled) {
    return false;
  }
  // emergency_stop
  if (lhs->emergency_stop != rhs->emergency_stop) {
    return false;
  }
  // current_mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_mode), &(rhs->current_mode)))
  {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->current_pose), &(rhs->current_pose)))
  {
    return false;
  }
  // current_velocity
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->current_velocity), &(rhs->current_velocity)))
  {
    return false;
  }
  return true;
}

bool
urc_msgs__msg__RoverStatus__copy(
  const urc_msgs__msg__RoverStatus * input,
  urc_msgs__msg__RoverStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // battery_percentage
  output->battery_percentage = input->battery_percentage;
  // motors_enabled
  output->motors_enabled = input->motors_enabled;
  // emergency_stop
  output->emergency_stop = input->emergency_stop;
  // current_mode
  if (!rosidl_runtime_c__String__copy(
      &(input->current_mode), &(output->current_mode)))
  {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->current_pose), &(output->current_pose)))
  {
    return false;
  }
  // current_velocity
  if (!geometry_msgs__msg__Twist__copy(
      &(input->current_velocity), &(output->current_velocity)))
  {
    return false;
  }
  return true;
}

urc_msgs__msg__RoverStatus *
urc_msgs__msg__RoverStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  urc_msgs__msg__RoverStatus * msg = (urc_msgs__msg__RoverStatus *)allocator.allocate(sizeof(urc_msgs__msg__RoverStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(urc_msgs__msg__RoverStatus));
  bool success = urc_msgs__msg__RoverStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
urc_msgs__msg__RoverStatus__destroy(urc_msgs__msg__RoverStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    urc_msgs__msg__RoverStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
urc_msgs__msg__RoverStatus__Sequence__init(urc_msgs__msg__RoverStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  urc_msgs__msg__RoverStatus * data = NULL;

  if (size) {
    data = (urc_msgs__msg__RoverStatus *)allocator.zero_allocate(size, sizeof(urc_msgs__msg__RoverStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = urc_msgs__msg__RoverStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        urc_msgs__msg__RoverStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
urc_msgs__msg__RoverStatus__Sequence__fini(urc_msgs__msg__RoverStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      urc_msgs__msg__RoverStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

urc_msgs__msg__RoverStatus__Sequence *
urc_msgs__msg__RoverStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  urc_msgs__msg__RoverStatus__Sequence * array = (urc_msgs__msg__RoverStatus__Sequence *)allocator.allocate(sizeof(urc_msgs__msg__RoverStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = urc_msgs__msg__RoverStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
urc_msgs__msg__RoverStatus__Sequence__destroy(urc_msgs__msg__RoverStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    urc_msgs__msg__RoverStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
urc_msgs__msg__RoverStatus__Sequence__are_equal(const urc_msgs__msg__RoverStatus__Sequence * lhs, const urc_msgs__msg__RoverStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!urc_msgs__msg__RoverStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
urc_msgs__msg__RoverStatus__Sequence__copy(
  const urc_msgs__msg__RoverStatus__Sequence * input,
  urc_msgs__msg__RoverStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(urc_msgs__msg__RoverStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    urc_msgs__msg__RoverStatus * data =
      (urc_msgs__msg__RoverStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!urc_msgs__msg__RoverStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          urc_msgs__msg__RoverStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!urc_msgs__msg__RoverStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
