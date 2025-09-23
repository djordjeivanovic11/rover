// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_control:msg/DriveCommand.idl
// generated code does not contain a copyright notice
#include "rover_control/msg/detail/drive_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `control_mode`
// Member `mission_mode`
// Member `source`
#include "rosidl_runtime_c/string_functions.h"

bool
rover_control__msg__DriveCommand__init(rover_control__msg__DriveCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rover_control__msg__DriveCommand__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__init(&msg->velocity)) {
    rover_control__msg__DriveCommand__fini(msg);
    return false;
  }
  // acceleration
  // duration
  // control_mode
  if (!rosidl_runtime_c__String__init(&msg->control_mode)) {
    rover_control__msg__DriveCommand__fini(msg);
    return false;
  }
  // relative_command
  // max_velocity
  // max_acceleration
  // enforce_safety_limits
  // mission_mode
  if (!rosidl_runtime_c__String__init(&msg->mission_mode)) {
    rover_control__msg__DriveCommand__fini(msg);
    return false;
  }
  // curvature
  // lookahead_distance
  // use_slip_compensation
  // priority
  // timeout
  // source
  if (!rosidl_runtime_c__String__init(&msg->source)) {
    rover_control__msg__DriveCommand__fini(msg);
    return false;
  }
  return true;
}

void
rover_control__msg__DriveCommand__fini(rover_control__msg__DriveCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // velocity
  geometry_msgs__msg__Twist__fini(&msg->velocity);
  // acceleration
  // duration
  // control_mode
  rosidl_runtime_c__String__fini(&msg->control_mode);
  // relative_command
  // max_velocity
  // max_acceleration
  // enforce_safety_limits
  // mission_mode
  rosidl_runtime_c__String__fini(&msg->mission_mode);
  // curvature
  // lookahead_distance
  // use_slip_compensation
  // priority
  // timeout
  // source
  rosidl_runtime_c__String__fini(&msg->source);
}

bool
rover_control__msg__DriveCommand__are_equal(const rover_control__msg__DriveCommand * lhs, const rover_control__msg__DriveCommand * rhs)
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
  // velocity
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // acceleration
  if (lhs->acceleration != rhs->acceleration) {
    return false;
  }
  // duration
  if (lhs->duration != rhs->duration) {
    return false;
  }
  // control_mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->control_mode), &(rhs->control_mode)))
  {
    return false;
  }
  // relative_command
  if (lhs->relative_command != rhs->relative_command) {
    return false;
  }
  // max_velocity
  if (lhs->max_velocity != rhs->max_velocity) {
    return false;
  }
  // max_acceleration
  if (lhs->max_acceleration != rhs->max_acceleration) {
    return false;
  }
  // enforce_safety_limits
  if (lhs->enforce_safety_limits != rhs->enforce_safety_limits) {
    return false;
  }
  // mission_mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission_mode), &(rhs->mission_mode)))
  {
    return false;
  }
  // curvature
  if (lhs->curvature != rhs->curvature) {
    return false;
  }
  // lookahead_distance
  if (lhs->lookahead_distance != rhs->lookahead_distance) {
    return false;
  }
  // use_slip_compensation
  if (lhs->use_slip_compensation != rhs->use_slip_compensation) {
    return false;
  }
  // priority
  if (lhs->priority != rhs->priority) {
    return false;
  }
  // timeout
  if (lhs->timeout != rhs->timeout) {
    return false;
  }
  // source
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->source), &(rhs->source)))
  {
    return false;
  }
  return true;
}

bool
rover_control__msg__DriveCommand__copy(
  const rover_control__msg__DriveCommand * input,
  rover_control__msg__DriveCommand * output)
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
  // velocity
  if (!geometry_msgs__msg__Twist__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // acceleration
  output->acceleration = input->acceleration;
  // duration
  output->duration = input->duration;
  // control_mode
  if (!rosidl_runtime_c__String__copy(
      &(input->control_mode), &(output->control_mode)))
  {
    return false;
  }
  // relative_command
  output->relative_command = input->relative_command;
  // max_velocity
  output->max_velocity = input->max_velocity;
  // max_acceleration
  output->max_acceleration = input->max_acceleration;
  // enforce_safety_limits
  output->enforce_safety_limits = input->enforce_safety_limits;
  // mission_mode
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_mode), &(output->mission_mode)))
  {
    return false;
  }
  // curvature
  output->curvature = input->curvature;
  // lookahead_distance
  output->lookahead_distance = input->lookahead_distance;
  // use_slip_compensation
  output->use_slip_compensation = input->use_slip_compensation;
  // priority
  output->priority = input->priority;
  // timeout
  output->timeout = input->timeout;
  // source
  if (!rosidl_runtime_c__String__copy(
      &(input->source), &(output->source)))
  {
    return false;
  }
  return true;
}

rover_control__msg__DriveCommand *
rover_control__msg__DriveCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__msg__DriveCommand * msg = (rover_control__msg__DriveCommand *)allocator.allocate(sizeof(rover_control__msg__DriveCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__msg__DriveCommand));
  bool success = rover_control__msg__DriveCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__msg__DriveCommand__destroy(rover_control__msg__DriveCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__msg__DriveCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__msg__DriveCommand__Sequence__init(rover_control__msg__DriveCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__msg__DriveCommand * data = NULL;

  if (size) {
    data = (rover_control__msg__DriveCommand *)allocator.zero_allocate(size, sizeof(rover_control__msg__DriveCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__msg__DriveCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__msg__DriveCommand__fini(&data[i - 1]);
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
rover_control__msg__DriveCommand__Sequence__fini(rover_control__msg__DriveCommand__Sequence * array)
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
      rover_control__msg__DriveCommand__fini(&array->data[i]);
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

rover_control__msg__DriveCommand__Sequence *
rover_control__msg__DriveCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__msg__DriveCommand__Sequence * array = (rover_control__msg__DriveCommand__Sequence *)allocator.allocate(sizeof(rover_control__msg__DriveCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__msg__DriveCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__msg__DriveCommand__Sequence__destroy(rover_control__msg__DriveCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__msg__DriveCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__msg__DriveCommand__Sequence__are_equal(const rover_control__msg__DriveCommand__Sequence * lhs, const rover_control__msg__DriveCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__msg__DriveCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__msg__DriveCommand__Sequence__copy(
  const rover_control__msg__DriveCommand__Sequence * input,
  rover_control__msg__DriveCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__msg__DriveCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__msg__DriveCommand * data =
      (rover_control__msg__DriveCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__msg__DriveCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__msg__DriveCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__msg__DriveCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
