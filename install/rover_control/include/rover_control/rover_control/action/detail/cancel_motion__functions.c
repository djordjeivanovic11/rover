// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_control:action/CancelMotion.idl
// generated code does not contain a copyright notice
#include "rover_control/action/detail/cancel_motion__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"

bool
rover_control__action__CancelMotion_Goal__init(rover_control__action__CancelMotion_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // emergency_stop
  msg->emergency_stop = false;
  // deceleration_time
  msg->deceleration_time = 1.0f;
  // cancel_all_actions
  msg->cancel_all_actions = true;
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    rover_control__action__CancelMotion_Goal__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->reason, "user_request");
    if (!success) {
      goto abort_init_0;
    }
  }
  return true;
abort_init_0:
  return false;
}

void
rover_control__action__CancelMotion_Goal__fini(rover_control__action__CancelMotion_Goal * msg)
{
  if (!msg) {
    return;
  }
  // emergency_stop
  // deceleration_time
  // cancel_all_actions
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
}

bool
rover_control__action__CancelMotion_Goal__are_equal(const rover_control__action__CancelMotion_Goal * lhs, const rover_control__action__CancelMotion_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // emergency_stop
  if (lhs->emergency_stop != rhs->emergency_stop) {
    return false;
  }
  // deceleration_time
  if (lhs->deceleration_time != rhs->deceleration_time) {
    return false;
  }
  // cancel_all_actions
  if (lhs->cancel_all_actions != rhs->cancel_all_actions) {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  return true;
}

bool
rover_control__action__CancelMotion_Goal__copy(
  const rover_control__action__CancelMotion_Goal * input,
  rover_control__action__CancelMotion_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // emergency_stop
  output->emergency_stop = input->emergency_stop;
  // deceleration_time
  output->deceleration_time = input->deceleration_time;
  // cancel_all_actions
  output->cancel_all_actions = input->cancel_all_actions;
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  return true;
}

rover_control__action__CancelMotion_Goal *
rover_control__action__CancelMotion_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_Goal * msg = (rover_control__action__CancelMotion_Goal *)allocator.allocate(sizeof(rover_control__action__CancelMotion_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__CancelMotion_Goal));
  bool success = rover_control__action__CancelMotion_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__CancelMotion_Goal__destroy(rover_control__action__CancelMotion_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__CancelMotion_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__CancelMotion_Goal__Sequence__init(rover_control__action__CancelMotion_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_Goal * data = NULL;

  if (size) {
    data = (rover_control__action__CancelMotion_Goal *)allocator.zero_allocate(size, sizeof(rover_control__action__CancelMotion_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__CancelMotion_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__CancelMotion_Goal__fini(&data[i - 1]);
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
rover_control__action__CancelMotion_Goal__Sequence__fini(rover_control__action__CancelMotion_Goal__Sequence * array)
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
      rover_control__action__CancelMotion_Goal__fini(&array->data[i]);
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

rover_control__action__CancelMotion_Goal__Sequence *
rover_control__action__CancelMotion_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_Goal__Sequence * array = (rover_control__action__CancelMotion_Goal__Sequence *)allocator.allocate(sizeof(rover_control__action__CancelMotion_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__CancelMotion_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__CancelMotion_Goal__Sequence__destroy(rover_control__action__CancelMotion_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__CancelMotion_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__CancelMotion_Goal__Sequence__are_equal(const rover_control__action__CancelMotion_Goal__Sequence * lhs, const rover_control__action__CancelMotion_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__CancelMotion_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__CancelMotion_Goal__Sequence__copy(
  const rover_control__action__CancelMotion_Goal__Sequence * input,
  rover_control__action__CancelMotion_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__CancelMotion_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__CancelMotion_Goal * data =
      (rover_control__action__CancelMotion_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__CancelMotion_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__CancelMotion_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__CancelMotion_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `error_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `velocity_at_stop`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `final_pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
rover_control__action__CancelMotion_Result__init(rover_control__action__CancelMotion_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // result_code
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    rover_control__action__CancelMotion_Result__fini(msg);
    return false;
  }
  // stop_time
  // velocity_at_stop
  if (!geometry_msgs__msg__Twist__init(&msg->velocity_at_stop)) {
    rover_control__action__CancelMotion_Result__fini(msg);
    return false;
  }
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->final_pose)) {
    rover_control__action__CancelMotion_Result__fini(msg);
    return false;
  }
  // actions_cancelled
  return true;
}

void
rover_control__action__CancelMotion_Result__fini(rover_control__action__CancelMotion_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // result_code
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
  // stop_time
  // velocity_at_stop
  geometry_msgs__msg__Twist__fini(&msg->velocity_at_stop);
  // final_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->final_pose);
  // actions_cancelled
}

bool
rover_control__action__CancelMotion_Result__are_equal(const rover_control__action__CancelMotion_Result * lhs, const rover_control__action__CancelMotion_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // result_code
  if (lhs->result_code != rhs->result_code) {
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_message), &(rhs->error_message)))
  {
    return false;
  }
  // stop_time
  if (lhs->stop_time != rhs->stop_time) {
    return false;
  }
  // velocity_at_stop
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->velocity_at_stop), &(rhs->velocity_at_stop)))
  {
    return false;
  }
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->final_pose), &(rhs->final_pose)))
  {
    return false;
  }
  // actions_cancelled
  if (lhs->actions_cancelled != rhs->actions_cancelled) {
    return false;
  }
  return true;
}

bool
rover_control__action__CancelMotion_Result__copy(
  const rover_control__action__CancelMotion_Result * input,
  rover_control__action__CancelMotion_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // result_code
  output->result_code = input->result_code;
  // error_message
  if (!rosidl_runtime_c__String__copy(
      &(input->error_message), &(output->error_message)))
  {
    return false;
  }
  // stop_time
  output->stop_time = input->stop_time;
  // velocity_at_stop
  if (!geometry_msgs__msg__Twist__copy(
      &(input->velocity_at_stop), &(output->velocity_at_stop)))
  {
    return false;
  }
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->final_pose), &(output->final_pose)))
  {
    return false;
  }
  // actions_cancelled
  output->actions_cancelled = input->actions_cancelled;
  return true;
}

rover_control__action__CancelMotion_Result *
rover_control__action__CancelMotion_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_Result * msg = (rover_control__action__CancelMotion_Result *)allocator.allocate(sizeof(rover_control__action__CancelMotion_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__CancelMotion_Result));
  bool success = rover_control__action__CancelMotion_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__CancelMotion_Result__destroy(rover_control__action__CancelMotion_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__CancelMotion_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__CancelMotion_Result__Sequence__init(rover_control__action__CancelMotion_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_Result * data = NULL;

  if (size) {
    data = (rover_control__action__CancelMotion_Result *)allocator.zero_allocate(size, sizeof(rover_control__action__CancelMotion_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__CancelMotion_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__CancelMotion_Result__fini(&data[i - 1]);
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
rover_control__action__CancelMotion_Result__Sequence__fini(rover_control__action__CancelMotion_Result__Sequence * array)
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
      rover_control__action__CancelMotion_Result__fini(&array->data[i]);
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

rover_control__action__CancelMotion_Result__Sequence *
rover_control__action__CancelMotion_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_Result__Sequence * array = (rover_control__action__CancelMotion_Result__Sequence *)allocator.allocate(sizeof(rover_control__action__CancelMotion_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__CancelMotion_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__CancelMotion_Result__Sequence__destroy(rover_control__action__CancelMotion_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__CancelMotion_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__CancelMotion_Result__Sequence__are_equal(const rover_control__action__CancelMotion_Result__Sequence * lhs, const rover_control__action__CancelMotion_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__CancelMotion_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__CancelMotion_Result__Sequence__copy(
  const rover_control__action__CancelMotion_Result__Sequence * input,
  rover_control__action__CancelMotion_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__CancelMotion_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__CancelMotion_Result * data =
      (rover_control__action__CancelMotion_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__CancelMotion_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__CancelMotion_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__CancelMotion_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `current_state`
// Member `status_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `current_velocity`
// already included above
// #include "geometry_msgs/msg/detail/twist__functions.h"

bool
rover_control__action__CancelMotion_Feedback__init(rover_control__action__CancelMotion_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // current_state
  if (!rosidl_runtime_c__String__init(&msg->current_state)) {
    rover_control__action__CancelMotion_Feedback__fini(msg);
    return false;
  }
  // current_velocity
  if (!geometry_msgs__msg__Twist__init(&msg->current_velocity)) {
    rover_control__action__CancelMotion_Feedback__fini(msg);
    return false;
  }
  // progress_percentage
  // status_message
  if (!rosidl_runtime_c__String__init(&msg->status_message)) {
    rover_control__action__CancelMotion_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
rover_control__action__CancelMotion_Feedback__fini(rover_control__action__CancelMotion_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // current_state
  rosidl_runtime_c__String__fini(&msg->current_state);
  // current_velocity
  geometry_msgs__msg__Twist__fini(&msg->current_velocity);
  // progress_percentage
  // status_message
  rosidl_runtime_c__String__fini(&msg->status_message);
}

bool
rover_control__action__CancelMotion_Feedback__are_equal(const rover_control__action__CancelMotion_Feedback * lhs, const rover_control__action__CancelMotion_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_state), &(rhs->current_state)))
  {
    return false;
  }
  // current_velocity
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->current_velocity), &(rhs->current_velocity)))
  {
    return false;
  }
  // progress_percentage
  if (lhs->progress_percentage != rhs->progress_percentage) {
    return false;
  }
  // status_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status_message), &(rhs->status_message)))
  {
    return false;
  }
  return true;
}

bool
rover_control__action__CancelMotion_Feedback__copy(
  const rover_control__action__CancelMotion_Feedback * input,
  rover_control__action__CancelMotion_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // current_state
  if (!rosidl_runtime_c__String__copy(
      &(input->current_state), &(output->current_state)))
  {
    return false;
  }
  // current_velocity
  if (!geometry_msgs__msg__Twist__copy(
      &(input->current_velocity), &(output->current_velocity)))
  {
    return false;
  }
  // progress_percentage
  output->progress_percentage = input->progress_percentage;
  // status_message
  if (!rosidl_runtime_c__String__copy(
      &(input->status_message), &(output->status_message)))
  {
    return false;
  }
  return true;
}

rover_control__action__CancelMotion_Feedback *
rover_control__action__CancelMotion_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_Feedback * msg = (rover_control__action__CancelMotion_Feedback *)allocator.allocate(sizeof(rover_control__action__CancelMotion_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__CancelMotion_Feedback));
  bool success = rover_control__action__CancelMotion_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__CancelMotion_Feedback__destroy(rover_control__action__CancelMotion_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__CancelMotion_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__CancelMotion_Feedback__Sequence__init(rover_control__action__CancelMotion_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_Feedback * data = NULL;

  if (size) {
    data = (rover_control__action__CancelMotion_Feedback *)allocator.zero_allocate(size, sizeof(rover_control__action__CancelMotion_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__CancelMotion_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__CancelMotion_Feedback__fini(&data[i - 1]);
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
rover_control__action__CancelMotion_Feedback__Sequence__fini(rover_control__action__CancelMotion_Feedback__Sequence * array)
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
      rover_control__action__CancelMotion_Feedback__fini(&array->data[i]);
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

rover_control__action__CancelMotion_Feedback__Sequence *
rover_control__action__CancelMotion_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_Feedback__Sequence * array = (rover_control__action__CancelMotion_Feedback__Sequence *)allocator.allocate(sizeof(rover_control__action__CancelMotion_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__CancelMotion_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__CancelMotion_Feedback__Sequence__destroy(rover_control__action__CancelMotion_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__CancelMotion_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__CancelMotion_Feedback__Sequence__are_equal(const rover_control__action__CancelMotion_Feedback__Sequence * lhs, const rover_control__action__CancelMotion_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__CancelMotion_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__CancelMotion_Feedback__Sequence__copy(
  const rover_control__action__CancelMotion_Feedback__Sequence * input,
  rover_control__action__CancelMotion_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__CancelMotion_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__CancelMotion_Feedback * data =
      (rover_control__action__CancelMotion_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__CancelMotion_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__CancelMotion_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__CancelMotion_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "rover_control/action/detail/cancel_motion__functions.h"

bool
rover_control__action__CancelMotion_SendGoal_Request__init(rover_control__action__CancelMotion_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    rover_control__action__CancelMotion_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!rover_control__action__CancelMotion_Goal__init(&msg->goal)) {
    rover_control__action__CancelMotion_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
rover_control__action__CancelMotion_SendGoal_Request__fini(rover_control__action__CancelMotion_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  rover_control__action__CancelMotion_Goal__fini(&msg->goal);
}

bool
rover_control__action__CancelMotion_SendGoal_Request__are_equal(const rover_control__action__CancelMotion_SendGoal_Request * lhs, const rover_control__action__CancelMotion_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!rover_control__action__CancelMotion_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
rover_control__action__CancelMotion_SendGoal_Request__copy(
  const rover_control__action__CancelMotion_SendGoal_Request * input,
  rover_control__action__CancelMotion_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!rover_control__action__CancelMotion_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

rover_control__action__CancelMotion_SendGoal_Request *
rover_control__action__CancelMotion_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_SendGoal_Request * msg = (rover_control__action__CancelMotion_SendGoal_Request *)allocator.allocate(sizeof(rover_control__action__CancelMotion_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__CancelMotion_SendGoal_Request));
  bool success = rover_control__action__CancelMotion_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__CancelMotion_SendGoal_Request__destroy(rover_control__action__CancelMotion_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__CancelMotion_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__CancelMotion_SendGoal_Request__Sequence__init(rover_control__action__CancelMotion_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_SendGoal_Request * data = NULL;

  if (size) {
    data = (rover_control__action__CancelMotion_SendGoal_Request *)allocator.zero_allocate(size, sizeof(rover_control__action__CancelMotion_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__CancelMotion_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__CancelMotion_SendGoal_Request__fini(&data[i - 1]);
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
rover_control__action__CancelMotion_SendGoal_Request__Sequence__fini(rover_control__action__CancelMotion_SendGoal_Request__Sequence * array)
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
      rover_control__action__CancelMotion_SendGoal_Request__fini(&array->data[i]);
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

rover_control__action__CancelMotion_SendGoal_Request__Sequence *
rover_control__action__CancelMotion_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_SendGoal_Request__Sequence * array = (rover_control__action__CancelMotion_SendGoal_Request__Sequence *)allocator.allocate(sizeof(rover_control__action__CancelMotion_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__CancelMotion_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__CancelMotion_SendGoal_Request__Sequence__destroy(rover_control__action__CancelMotion_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__CancelMotion_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__CancelMotion_SendGoal_Request__Sequence__are_equal(const rover_control__action__CancelMotion_SendGoal_Request__Sequence * lhs, const rover_control__action__CancelMotion_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__CancelMotion_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__CancelMotion_SendGoal_Request__Sequence__copy(
  const rover_control__action__CancelMotion_SendGoal_Request__Sequence * input,
  rover_control__action__CancelMotion_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__CancelMotion_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__CancelMotion_SendGoal_Request * data =
      (rover_control__action__CancelMotion_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__CancelMotion_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__CancelMotion_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__CancelMotion_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
rover_control__action__CancelMotion_SendGoal_Response__init(rover_control__action__CancelMotion_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    rover_control__action__CancelMotion_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
rover_control__action__CancelMotion_SendGoal_Response__fini(rover_control__action__CancelMotion_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
rover_control__action__CancelMotion_SendGoal_Response__are_equal(const rover_control__action__CancelMotion_SendGoal_Response * lhs, const rover_control__action__CancelMotion_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
rover_control__action__CancelMotion_SendGoal_Response__copy(
  const rover_control__action__CancelMotion_SendGoal_Response * input,
  rover_control__action__CancelMotion_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

rover_control__action__CancelMotion_SendGoal_Response *
rover_control__action__CancelMotion_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_SendGoal_Response * msg = (rover_control__action__CancelMotion_SendGoal_Response *)allocator.allocate(sizeof(rover_control__action__CancelMotion_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__CancelMotion_SendGoal_Response));
  bool success = rover_control__action__CancelMotion_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__CancelMotion_SendGoal_Response__destroy(rover_control__action__CancelMotion_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__CancelMotion_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__CancelMotion_SendGoal_Response__Sequence__init(rover_control__action__CancelMotion_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_SendGoal_Response * data = NULL;

  if (size) {
    data = (rover_control__action__CancelMotion_SendGoal_Response *)allocator.zero_allocate(size, sizeof(rover_control__action__CancelMotion_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__CancelMotion_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__CancelMotion_SendGoal_Response__fini(&data[i - 1]);
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
rover_control__action__CancelMotion_SendGoal_Response__Sequence__fini(rover_control__action__CancelMotion_SendGoal_Response__Sequence * array)
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
      rover_control__action__CancelMotion_SendGoal_Response__fini(&array->data[i]);
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

rover_control__action__CancelMotion_SendGoal_Response__Sequence *
rover_control__action__CancelMotion_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_SendGoal_Response__Sequence * array = (rover_control__action__CancelMotion_SendGoal_Response__Sequence *)allocator.allocate(sizeof(rover_control__action__CancelMotion_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__CancelMotion_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__CancelMotion_SendGoal_Response__Sequence__destroy(rover_control__action__CancelMotion_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__CancelMotion_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__CancelMotion_SendGoal_Response__Sequence__are_equal(const rover_control__action__CancelMotion_SendGoal_Response__Sequence * lhs, const rover_control__action__CancelMotion_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__CancelMotion_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__CancelMotion_SendGoal_Response__Sequence__copy(
  const rover_control__action__CancelMotion_SendGoal_Response__Sequence * input,
  rover_control__action__CancelMotion_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__CancelMotion_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__CancelMotion_SendGoal_Response * data =
      (rover_control__action__CancelMotion_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__CancelMotion_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__CancelMotion_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__CancelMotion_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
rover_control__action__CancelMotion_GetResult_Request__init(rover_control__action__CancelMotion_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    rover_control__action__CancelMotion_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
rover_control__action__CancelMotion_GetResult_Request__fini(rover_control__action__CancelMotion_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
rover_control__action__CancelMotion_GetResult_Request__are_equal(const rover_control__action__CancelMotion_GetResult_Request * lhs, const rover_control__action__CancelMotion_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
rover_control__action__CancelMotion_GetResult_Request__copy(
  const rover_control__action__CancelMotion_GetResult_Request * input,
  rover_control__action__CancelMotion_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

rover_control__action__CancelMotion_GetResult_Request *
rover_control__action__CancelMotion_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_GetResult_Request * msg = (rover_control__action__CancelMotion_GetResult_Request *)allocator.allocate(sizeof(rover_control__action__CancelMotion_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__CancelMotion_GetResult_Request));
  bool success = rover_control__action__CancelMotion_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__CancelMotion_GetResult_Request__destroy(rover_control__action__CancelMotion_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__CancelMotion_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__CancelMotion_GetResult_Request__Sequence__init(rover_control__action__CancelMotion_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_GetResult_Request * data = NULL;

  if (size) {
    data = (rover_control__action__CancelMotion_GetResult_Request *)allocator.zero_allocate(size, sizeof(rover_control__action__CancelMotion_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__CancelMotion_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__CancelMotion_GetResult_Request__fini(&data[i - 1]);
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
rover_control__action__CancelMotion_GetResult_Request__Sequence__fini(rover_control__action__CancelMotion_GetResult_Request__Sequence * array)
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
      rover_control__action__CancelMotion_GetResult_Request__fini(&array->data[i]);
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

rover_control__action__CancelMotion_GetResult_Request__Sequence *
rover_control__action__CancelMotion_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_GetResult_Request__Sequence * array = (rover_control__action__CancelMotion_GetResult_Request__Sequence *)allocator.allocate(sizeof(rover_control__action__CancelMotion_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__CancelMotion_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__CancelMotion_GetResult_Request__Sequence__destroy(rover_control__action__CancelMotion_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__CancelMotion_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__CancelMotion_GetResult_Request__Sequence__are_equal(const rover_control__action__CancelMotion_GetResult_Request__Sequence * lhs, const rover_control__action__CancelMotion_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__CancelMotion_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__CancelMotion_GetResult_Request__Sequence__copy(
  const rover_control__action__CancelMotion_GetResult_Request__Sequence * input,
  rover_control__action__CancelMotion_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__CancelMotion_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__CancelMotion_GetResult_Request * data =
      (rover_control__action__CancelMotion_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__CancelMotion_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__CancelMotion_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__CancelMotion_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "rover_control/action/detail/cancel_motion__functions.h"

bool
rover_control__action__CancelMotion_GetResult_Response__init(rover_control__action__CancelMotion_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!rover_control__action__CancelMotion_Result__init(&msg->result)) {
    rover_control__action__CancelMotion_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
rover_control__action__CancelMotion_GetResult_Response__fini(rover_control__action__CancelMotion_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  rover_control__action__CancelMotion_Result__fini(&msg->result);
}

bool
rover_control__action__CancelMotion_GetResult_Response__are_equal(const rover_control__action__CancelMotion_GetResult_Response * lhs, const rover_control__action__CancelMotion_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!rover_control__action__CancelMotion_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
rover_control__action__CancelMotion_GetResult_Response__copy(
  const rover_control__action__CancelMotion_GetResult_Response * input,
  rover_control__action__CancelMotion_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!rover_control__action__CancelMotion_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

rover_control__action__CancelMotion_GetResult_Response *
rover_control__action__CancelMotion_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_GetResult_Response * msg = (rover_control__action__CancelMotion_GetResult_Response *)allocator.allocate(sizeof(rover_control__action__CancelMotion_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__CancelMotion_GetResult_Response));
  bool success = rover_control__action__CancelMotion_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__CancelMotion_GetResult_Response__destroy(rover_control__action__CancelMotion_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__CancelMotion_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__CancelMotion_GetResult_Response__Sequence__init(rover_control__action__CancelMotion_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_GetResult_Response * data = NULL;

  if (size) {
    data = (rover_control__action__CancelMotion_GetResult_Response *)allocator.zero_allocate(size, sizeof(rover_control__action__CancelMotion_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__CancelMotion_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__CancelMotion_GetResult_Response__fini(&data[i - 1]);
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
rover_control__action__CancelMotion_GetResult_Response__Sequence__fini(rover_control__action__CancelMotion_GetResult_Response__Sequence * array)
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
      rover_control__action__CancelMotion_GetResult_Response__fini(&array->data[i]);
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

rover_control__action__CancelMotion_GetResult_Response__Sequence *
rover_control__action__CancelMotion_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_GetResult_Response__Sequence * array = (rover_control__action__CancelMotion_GetResult_Response__Sequence *)allocator.allocate(sizeof(rover_control__action__CancelMotion_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__CancelMotion_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__CancelMotion_GetResult_Response__Sequence__destroy(rover_control__action__CancelMotion_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__CancelMotion_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__CancelMotion_GetResult_Response__Sequence__are_equal(const rover_control__action__CancelMotion_GetResult_Response__Sequence * lhs, const rover_control__action__CancelMotion_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__CancelMotion_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__CancelMotion_GetResult_Response__Sequence__copy(
  const rover_control__action__CancelMotion_GetResult_Response__Sequence * input,
  rover_control__action__CancelMotion_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__CancelMotion_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__CancelMotion_GetResult_Response * data =
      (rover_control__action__CancelMotion_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__CancelMotion_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__CancelMotion_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__CancelMotion_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "rover_control/action/detail/cancel_motion__functions.h"

bool
rover_control__action__CancelMotion_FeedbackMessage__init(rover_control__action__CancelMotion_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    rover_control__action__CancelMotion_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!rover_control__action__CancelMotion_Feedback__init(&msg->feedback)) {
    rover_control__action__CancelMotion_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
rover_control__action__CancelMotion_FeedbackMessage__fini(rover_control__action__CancelMotion_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  rover_control__action__CancelMotion_Feedback__fini(&msg->feedback);
}

bool
rover_control__action__CancelMotion_FeedbackMessage__are_equal(const rover_control__action__CancelMotion_FeedbackMessage * lhs, const rover_control__action__CancelMotion_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!rover_control__action__CancelMotion_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
rover_control__action__CancelMotion_FeedbackMessage__copy(
  const rover_control__action__CancelMotion_FeedbackMessage * input,
  rover_control__action__CancelMotion_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!rover_control__action__CancelMotion_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

rover_control__action__CancelMotion_FeedbackMessage *
rover_control__action__CancelMotion_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_FeedbackMessage * msg = (rover_control__action__CancelMotion_FeedbackMessage *)allocator.allocate(sizeof(rover_control__action__CancelMotion_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__CancelMotion_FeedbackMessage));
  bool success = rover_control__action__CancelMotion_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__CancelMotion_FeedbackMessage__destroy(rover_control__action__CancelMotion_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__CancelMotion_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__CancelMotion_FeedbackMessage__Sequence__init(rover_control__action__CancelMotion_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_FeedbackMessage * data = NULL;

  if (size) {
    data = (rover_control__action__CancelMotion_FeedbackMessage *)allocator.zero_allocate(size, sizeof(rover_control__action__CancelMotion_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__CancelMotion_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__CancelMotion_FeedbackMessage__fini(&data[i - 1]);
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
rover_control__action__CancelMotion_FeedbackMessage__Sequence__fini(rover_control__action__CancelMotion_FeedbackMessage__Sequence * array)
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
      rover_control__action__CancelMotion_FeedbackMessage__fini(&array->data[i]);
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

rover_control__action__CancelMotion_FeedbackMessage__Sequence *
rover_control__action__CancelMotion_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__CancelMotion_FeedbackMessage__Sequence * array = (rover_control__action__CancelMotion_FeedbackMessage__Sequence *)allocator.allocate(sizeof(rover_control__action__CancelMotion_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__CancelMotion_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__CancelMotion_FeedbackMessage__Sequence__destroy(rover_control__action__CancelMotion_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__CancelMotion_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__CancelMotion_FeedbackMessage__Sequence__are_equal(const rover_control__action__CancelMotion_FeedbackMessage__Sequence * lhs, const rover_control__action__CancelMotion_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__CancelMotion_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__CancelMotion_FeedbackMessage__Sequence__copy(
  const rover_control__action__CancelMotion_FeedbackMessage__Sequence * input,
  rover_control__action__CancelMotion_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__CancelMotion_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__CancelMotion_FeedbackMessage * data =
      (rover_control__action__CancelMotion_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__CancelMotion_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__CancelMotion_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__CancelMotion_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
