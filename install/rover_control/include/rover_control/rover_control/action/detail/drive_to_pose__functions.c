// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_control:action/DriveToPose.idl
// generated code does not contain a copyright notice
#include "rover_control/action/detail/drive_to_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `target_pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `mission_mode`
#include "rosidl_runtime_c/string_functions.h"

bool
rover_control__action__DriveToPose_Goal__init(rover_control__action__DriveToPose_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // target_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->target_pose)) {
    rover_control__action__DriveToPose_Goal__fini(msg);
    return false;
  }
  // max_velocity
  msg->max_velocity = 1.0f;
  // position_tolerance
  msg->position_tolerance = 0.1f;
  // orientation_tolerance
  msg->orientation_tolerance = 0.1f;
  // timeout
  msg->timeout = 60.0f;
  // use_visual_servoing
  msg->use_visual_servoing = false;
  // mission_mode
  if (!rosidl_runtime_c__String__init(&msg->mission_mode)) {
    rover_control__action__DriveToPose_Goal__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->mission_mode, "exploration");
    if (!success) {
      goto abort_init_0;
    }
  }
  // reverse_allowed
  msg->reverse_allowed = false;
  // approach_velocity
  msg->approach_velocity = 0.5f;
  return true;
abort_init_0:
  return false;
}

void
rover_control__action__DriveToPose_Goal__fini(rover_control__action__DriveToPose_Goal * msg)
{
  if (!msg) {
    return;
  }
  // target_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->target_pose);
  // max_velocity
  // position_tolerance
  // orientation_tolerance
  // timeout
  // use_visual_servoing
  // mission_mode
  rosidl_runtime_c__String__fini(&msg->mission_mode);
  // reverse_allowed
  // approach_velocity
}

bool
rover_control__action__DriveToPose_Goal__are_equal(const rover_control__action__DriveToPose_Goal * lhs, const rover_control__action__DriveToPose_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->target_pose), &(rhs->target_pose)))
  {
    return false;
  }
  // max_velocity
  if (lhs->max_velocity != rhs->max_velocity) {
    return false;
  }
  // position_tolerance
  if (lhs->position_tolerance != rhs->position_tolerance) {
    return false;
  }
  // orientation_tolerance
  if (lhs->orientation_tolerance != rhs->orientation_tolerance) {
    return false;
  }
  // timeout
  if (lhs->timeout != rhs->timeout) {
    return false;
  }
  // use_visual_servoing
  if (lhs->use_visual_servoing != rhs->use_visual_servoing) {
    return false;
  }
  // mission_mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission_mode), &(rhs->mission_mode)))
  {
    return false;
  }
  // reverse_allowed
  if (lhs->reverse_allowed != rhs->reverse_allowed) {
    return false;
  }
  // approach_velocity
  if (lhs->approach_velocity != rhs->approach_velocity) {
    return false;
  }
  return true;
}

bool
rover_control__action__DriveToPose_Goal__copy(
  const rover_control__action__DriveToPose_Goal * input,
  rover_control__action__DriveToPose_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // target_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->target_pose), &(output->target_pose)))
  {
    return false;
  }
  // max_velocity
  output->max_velocity = input->max_velocity;
  // position_tolerance
  output->position_tolerance = input->position_tolerance;
  // orientation_tolerance
  output->orientation_tolerance = input->orientation_tolerance;
  // timeout
  output->timeout = input->timeout;
  // use_visual_servoing
  output->use_visual_servoing = input->use_visual_servoing;
  // mission_mode
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_mode), &(output->mission_mode)))
  {
    return false;
  }
  // reverse_allowed
  output->reverse_allowed = input->reverse_allowed;
  // approach_velocity
  output->approach_velocity = input->approach_velocity;
  return true;
}

rover_control__action__DriveToPose_Goal *
rover_control__action__DriveToPose_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_Goal * msg = (rover_control__action__DriveToPose_Goal *)allocator.allocate(sizeof(rover_control__action__DriveToPose_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__DriveToPose_Goal));
  bool success = rover_control__action__DriveToPose_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__DriveToPose_Goal__destroy(rover_control__action__DriveToPose_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__DriveToPose_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__DriveToPose_Goal__Sequence__init(rover_control__action__DriveToPose_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_Goal * data = NULL;

  if (size) {
    data = (rover_control__action__DriveToPose_Goal *)allocator.zero_allocate(size, sizeof(rover_control__action__DriveToPose_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__DriveToPose_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__DriveToPose_Goal__fini(&data[i - 1]);
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
rover_control__action__DriveToPose_Goal__Sequence__fini(rover_control__action__DriveToPose_Goal__Sequence * array)
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
      rover_control__action__DriveToPose_Goal__fini(&array->data[i]);
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

rover_control__action__DriveToPose_Goal__Sequence *
rover_control__action__DriveToPose_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_Goal__Sequence * array = (rover_control__action__DriveToPose_Goal__Sequence *)allocator.allocate(sizeof(rover_control__action__DriveToPose_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__DriveToPose_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__DriveToPose_Goal__Sequence__destroy(rover_control__action__DriveToPose_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__DriveToPose_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__DriveToPose_Goal__Sequence__are_equal(const rover_control__action__DriveToPose_Goal__Sequence * lhs, const rover_control__action__DriveToPose_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__DriveToPose_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__DriveToPose_Goal__Sequence__copy(
  const rover_control__action__DriveToPose_Goal__Sequence * input,
  rover_control__action__DriveToPose_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__DriveToPose_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__DriveToPose_Goal * data =
      (rover_control__action__DriveToPose_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__DriveToPose_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__DriveToPose_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__DriveToPose_Goal__copy(
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
// Member `final_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `executed_path`
#include "nav_msgs/msg/detail/path__functions.h"

bool
rover_control__action__DriveToPose_Result__init(rover_control__action__DriveToPose_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // result_code
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    rover_control__action__DriveToPose_Result__fini(msg);
    return false;
  }
  // final_position_error
  // final_orientation_error
  // execution_time
  // distance_traveled
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->final_pose)) {
    rover_control__action__DriveToPose_Result__fini(msg);
    return false;
  }
  // executed_path
  if (!nav_msgs__msg__Path__init(&msg->executed_path)) {
    rover_control__action__DriveToPose_Result__fini(msg);
    return false;
  }
  return true;
}

void
rover_control__action__DriveToPose_Result__fini(rover_control__action__DriveToPose_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // result_code
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
  // final_position_error
  // final_orientation_error
  // execution_time
  // distance_traveled
  // final_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->final_pose);
  // executed_path
  nav_msgs__msg__Path__fini(&msg->executed_path);
}

bool
rover_control__action__DriveToPose_Result__are_equal(const rover_control__action__DriveToPose_Result * lhs, const rover_control__action__DriveToPose_Result * rhs)
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
  // final_position_error
  if (lhs->final_position_error != rhs->final_position_error) {
    return false;
  }
  // final_orientation_error
  if (lhs->final_orientation_error != rhs->final_orientation_error) {
    return false;
  }
  // execution_time
  if (lhs->execution_time != rhs->execution_time) {
    return false;
  }
  // distance_traveled
  if (lhs->distance_traveled != rhs->distance_traveled) {
    return false;
  }
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->final_pose), &(rhs->final_pose)))
  {
    return false;
  }
  // executed_path
  if (!nav_msgs__msg__Path__are_equal(
      &(lhs->executed_path), &(rhs->executed_path)))
  {
    return false;
  }
  return true;
}

bool
rover_control__action__DriveToPose_Result__copy(
  const rover_control__action__DriveToPose_Result * input,
  rover_control__action__DriveToPose_Result * output)
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
  // final_position_error
  output->final_position_error = input->final_position_error;
  // final_orientation_error
  output->final_orientation_error = input->final_orientation_error;
  // execution_time
  output->execution_time = input->execution_time;
  // distance_traveled
  output->distance_traveled = input->distance_traveled;
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->final_pose), &(output->final_pose)))
  {
    return false;
  }
  // executed_path
  if (!nav_msgs__msg__Path__copy(
      &(input->executed_path), &(output->executed_path)))
  {
    return false;
  }
  return true;
}

rover_control__action__DriveToPose_Result *
rover_control__action__DriveToPose_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_Result * msg = (rover_control__action__DriveToPose_Result *)allocator.allocate(sizeof(rover_control__action__DriveToPose_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__DriveToPose_Result));
  bool success = rover_control__action__DriveToPose_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__DriveToPose_Result__destroy(rover_control__action__DriveToPose_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__DriveToPose_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__DriveToPose_Result__Sequence__init(rover_control__action__DriveToPose_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_Result * data = NULL;

  if (size) {
    data = (rover_control__action__DriveToPose_Result *)allocator.zero_allocate(size, sizeof(rover_control__action__DriveToPose_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__DriveToPose_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__DriveToPose_Result__fini(&data[i - 1]);
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
rover_control__action__DriveToPose_Result__Sequence__fini(rover_control__action__DriveToPose_Result__Sequence * array)
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
      rover_control__action__DriveToPose_Result__fini(&array->data[i]);
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

rover_control__action__DriveToPose_Result__Sequence *
rover_control__action__DriveToPose_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_Result__Sequence * array = (rover_control__action__DriveToPose_Result__Sequence *)allocator.allocate(sizeof(rover_control__action__DriveToPose_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__DriveToPose_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__DriveToPose_Result__Sequence__destroy(rover_control__action__DriveToPose_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__DriveToPose_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__DriveToPose_Result__Sequence__are_equal(const rover_control__action__DriveToPose_Result__Sequence * lhs, const rover_control__action__DriveToPose_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__DriveToPose_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__DriveToPose_Result__Sequence__copy(
  const rover_control__action__DriveToPose_Result__Sequence * input,
  rover_control__action__DriveToPose_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__DriveToPose_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__DriveToPose_Result * data =
      (rover_control__action__DriveToPose_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__DriveToPose_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__DriveToPose_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__DriveToPose_Result__copy(
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
// Member `current_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
rover_control__action__DriveToPose_Feedback__init(rover_control__action__DriveToPose_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // current_state
  if (!rosidl_runtime_c__String__init(&msg->current_state)) {
    rover_control__action__DriveToPose_Feedback__fini(msg);
    return false;
  }
  // progress_percentage
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->current_pose)) {
    rover_control__action__DriveToPose_Feedback__fini(msg);
    return false;
  }
  // estimated_time_remaining
  // distance_remaining
  // current_velocity
  // status_message
  if (!rosidl_runtime_c__String__init(&msg->status_message)) {
    rover_control__action__DriveToPose_Feedback__fini(msg);
    return false;
  }
  // stuck_detected
  // autonomy_time_remaining
  return true;
}

void
rover_control__action__DriveToPose_Feedback__fini(rover_control__action__DriveToPose_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // current_state
  rosidl_runtime_c__String__fini(&msg->current_state);
  // progress_percentage
  // current_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->current_pose);
  // estimated_time_remaining
  // distance_remaining
  // current_velocity
  // status_message
  rosidl_runtime_c__String__fini(&msg->status_message);
  // stuck_detected
  // autonomy_time_remaining
}

bool
rover_control__action__DriveToPose_Feedback__are_equal(const rover_control__action__DriveToPose_Feedback * lhs, const rover_control__action__DriveToPose_Feedback * rhs)
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
  // progress_percentage
  if (lhs->progress_percentage != rhs->progress_percentage) {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->current_pose), &(rhs->current_pose)))
  {
    return false;
  }
  // estimated_time_remaining
  if (lhs->estimated_time_remaining != rhs->estimated_time_remaining) {
    return false;
  }
  // distance_remaining
  if (lhs->distance_remaining != rhs->distance_remaining) {
    return false;
  }
  // current_velocity
  if (lhs->current_velocity != rhs->current_velocity) {
    return false;
  }
  // status_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status_message), &(rhs->status_message)))
  {
    return false;
  }
  // stuck_detected
  if (lhs->stuck_detected != rhs->stuck_detected) {
    return false;
  }
  // autonomy_time_remaining
  if (lhs->autonomy_time_remaining != rhs->autonomy_time_remaining) {
    return false;
  }
  return true;
}

bool
rover_control__action__DriveToPose_Feedback__copy(
  const rover_control__action__DriveToPose_Feedback * input,
  rover_control__action__DriveToPose_Feedback * output)
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
  // progress_percentage
  output->progress_percentage = input->progress_percentage;
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->current_pose), &(output->current_pose)))
  {
    return false;
  }
  // estimated_time_remaining
  output->estimated_time_remaining = input->estimated_time_remaining;
  // distance_remaining
  output->distance_remaining = input->distance_remaining;
  // current_velocity
  output->current_velocity = input->current_velocity;
  // status_message
  if (!rosidl_runtime_c__String__copy(
      &(input->status_message), &(output->status_message)))
  {
    return false;
  }
  // stuck_detected
  output->stuck_detected = input->stuck_detected;
  // autonomy_time_remaining
  output->autonomy_time_remaining = input->autonomy_time_remaining;
  return true;
}

rover_control__action__DriveToPose_Feedback *
rover_control__action__DriveToPose_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_Feedback * msg = (rover_control__action__DriveToPose_Feedback *)allocator.allocate(sizeof(rover_control__action__DriveToPose_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__DriveToPose_Feedback));
  bool success = rover_control__action__DriveToPose_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__DriveToPose_Feedback__destroy(rover_control__action__DriveToPose_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__DriveToPose_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__DriveToPose_Feedback__Sequence__init(rover_control__action__DriveToPose_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_Feedback * data = NULL;

  if (size) {
    data = (rover_control__action__DriveToPose_Feedback *)allocator.zero_allocate(size, sizeof(rover_control__action__DriveToPose_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__DriveToPose_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__DriveToPose_Feedback__fini(&data[i - 1]);
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
rover_control__action__DriveToPose_Feedback__Sequence__fini(rover_control__action__DriveToPose_Feedback__Sequence * array)
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
      rover_control__action__DriveToPose_Feedback__fini(&array->data[i]);
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

rover_control__action__DriveToPose_Feedback__Sequence *
rover_control__action__DriveToPose_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_Feedback__Sequence * array = (rover_control__action__DriveToPose_Feedback__Sequence *)allocator.allocate(sizeof(rover_control__action__DriveToPose_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__DriveToPose_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__DriveToPose_Feedback__Sequence__destroy(rover_control__action__DriveToPose_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__DriveToPose_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__DriveToPose_Feedback__Sequence__are_equal(const rover_control__action__DriveToPose_Feedback__Sequence * lhs, const rover_control__action__DriveToPose_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__DriveToPose_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__DriveToPose_Feedback__Sequence__copy(
  const rover_control__action__DriveToPose_Feedback__Sequence * input,
  rover_control__action__DriveToPose_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__DriveToPose_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__DriveToPose_Feedback * data =
      (rover_control__action__DriveToPose_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__DriveToPose_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__DriveToPose_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__DriveToPose_Feedback__copy(
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
// #include "rover_control/action/detail/drive_to_pose__functions.h"

bool
rover_control__action__DriveToPose_SendGoal_Request__init(rover_control__action__DriveToPose_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    rover_control__action__DriveToPose_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!rover_control__action__DriveToPose_Goal__init(&msg->goal)) {
    rover_control__action__DriveToPose_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
rover_control__action__DriveToPose_SendGoal_Request__fini(rover_control__action__DriveToPose_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  rover_control__action__DriveToPose_Goal__fini(&msg->goal);
}

bool
rover_control__action__DriveToPose_SendGoal_Request__are_equal(const rover_control__action__DriveToPose_SendGoal_Request * lhs, const rover_control__action__DriveToPose_SendGoal_Request * rhs)
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
  if (!rover_control__action__DriveToPose_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
rover_control__action__DriveToPose_SendGoal_Request__copy(
  const rover_control__action__DriveToPose_SendGoal_Request * input,
  rover_control__action__DriveToPose_SendGoal_Request * output)
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
  if (!rover_control__action__DriveToPose_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

rover_control__action__DriveToPose_SendGoal_Request *
rover_control__action__DriveToPose_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_SendGoal_Request * msg = (rover_control__action__DriveToPose_SendGoal_Request *)allocator.allocate(sizeof(rover_control__action__DriveToPose_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__DriveToPose_SendGoal_Request));
  bool success = rover_control__action__DriveToPose_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__DriveToPose_SendGoal_Request__destroy(rover_control__action__DriveToPose_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__DriveToPose_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__DriveToPose_SendGoal_Request__Sequence__init(rover_control__action__DriveToPose_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_SendGoal_Request * data = NULL;

  if (size) {
    data = (rover_control__action__DriveToPose_SendGoal_Request *)allocator.zero_allocate(size, sizeof(rover_control__action__DriveToPose_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__DriveToPose_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__DriveToPose_SendGoal_Request__fini(&data[i - 1]);
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
rover_control__action__DriveToPose_SendGoal_Request__Sequence__fini(rover_control__action__DriveToPose_SendGoal_Request__Sequence * array)
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
      rover_control__action__DriveToPose_SendGoal_Request__fini(&array->data[i]);
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

rover_control__action__DriveToPose_SendGoal_Request__Sequence *
rover_control__action__DriveToPose_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_SendGoal_Request__Sequence * array = (rover_control__action__DriveToPose_SendGoal_Request__Sequence *)allocator.allocate(sizeof(rover_control__action__DriveToPose_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__DriveToPose_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__DriveToPose_SendGoal_Request__Sequence__destroy(rover_control__action__DriveToPose_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__DriveToPose_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__DriveToPose_SendGoal_Request__Sequence__are_equal(const rover_control__action__DriveToPose_SendGoal_Request__Sequence * lhs, const rover_control__action__DriveToPose_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__DriveToPose_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__DriveToPose_SendGoal_Request__Sequence__copy(
  const rover_control__action__DriveToPose_SendGoal_Request__Sequence * input,
  rover_control__action__DriveToPose_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__DriveToPose_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__DriveToPose_SendGoal_Request * data =
      (rover_control__action__DriveToPose_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__DriveToPose_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__DriveToPose_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__DriveToPose_SendGoal_Request__copy(
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
rover_control__action__DriveToPose_SendGoal_Response__init(rover_control__action__DriveToPose_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    rover_control__action__DriveToPose_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
rover_control__action__DriveToPose_SendGoal_Response__fini(rover_control__action__DriveToPose_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
rover_control__action__DriveToPose_SendGoal_Response__are_equal(const rover_control__action__DriveToPose_SendGoal_Response * lhs, const rover_control__action__DriveToPose_SendGoal_Response * rhs)
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
rover_control__action__DriveToPose_SendGoal_Response__copy(
  const rover_control__action__DriveToPose_SendGoal_Response * input,
  rover_control__action__DriveToPose_SendGoal_Response * output)
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

rover_control__action__DriveToPose_SendGoal_Response *
rover_control__action__DriveToPose_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_SendGoal_Response * msg = (rover_control__action__DriveToPose_SendGoal_Response *)allocator.allocate(sizeof(rover_control__action__DriveToPose_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__DriveToPose_SendGoal_Response));
  bool success = rover_control__action__DriveToPose_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__DriveToPose_SendGoal_Response__destroy(rover_control__action__DriveToPose_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__DriveToPose_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__DriveToPose_SendGoal_Response__Sequence__init(rover_control__action__DriveToPose_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_SendGoal_Response * data = NULL;

  if (size) {
    data = (rover_control__action__DriveToPose_SendGoal_Response *)allocator.zero_allocate(size, sizeof(rover_control__action__DriveToPose_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__DriveToPose_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__DriveToPose_SendGoal_Response__fini(&data[i - 1]);
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
rover_control__action__DriveToPose_SendGoal_Response__Sequence__fini(rover_control__action__DriveToPose_SendGoal_Response__Sequence * array)
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
      rover_control__action__DriveToPose_SendGoal_Response__fini(&array->data[i]);
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

rover_control__action__DriveToPose_SendGoal_Response__Sequence *
rover_control__action__DriveToPose_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_SendGoal_Response__Sequence * array = (rover_control__action__DriveToPose_SendGoal_Response__Sequence *)allocator.allocate(sizeof(rover_control__action__DriveToPose_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__DriveToPose_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__DriveToPose_SendGoal_Response__Sequence__destroy(rover_control__action__DriveToPose_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__DriveToPose_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__DriveToPose_SendGoal_Response__Sequence__are_equal(const rover_control__action__DriveToPose_SendGoal_Response__Sequence * lhs, const rover_control__action__DriveToPose_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__DriveToPose_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__DriveToPose_SendGoal_Response__Sequence__copy(
  const rover_control__action__DriveToPose_SendGoal_Response__Sequence * input,
  rover_control__action__DriveToPose_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__DriveToPose_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__DriveToPose_SendGoal_Response * data =
      (rover_control__action__DriveToPose_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__DriveToPose_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__DriveToPose_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__DriveToPose_SendGoal_Response__copy(
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
rover_control__action__DriveToPose_GetResult_Request__init(rover_control__action__DriveToPose_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    rover_control__action__DriveToPose_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
rover_control__action__DriveToPose_GetResult_Request__fini(rover_control__action__DriveToPose_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
rover_control__action__DriveToPose_GetResult_Request__are_equal(const rover_control__action__DriveToPose_GetResult_Request * lhs, const rover_control__action__DriveToPose_GetResult_Request * rhs)
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
rover_control__action__DriveToPose_GetResult_Request__copy(
  const rover_control__action__DriveToPose_GetResult_Request * input,
  rover_control__action__DriveToPose_GetResult_Request * output)
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

rover_control__action__DriveToPose_GetResult_Request *
rover_control__action__DriveToPose_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_GetResult_Request * msg = (rover_control__action__DriveToPose_GetResult_Request *)allocator.allocate(sizeof(rover_control__action__DriveToPose_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__DriveToPose_GetResult_Request));
  bool success = rover_control__action__DriveToPose_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__DriveToPose_GetResult_Request__destroy(rover_control__action__DriveToPose_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__DriveToPose_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__DriveToPose_GetResult_Request__Sequence__init(rover_control__action__DriveToPose_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_GetResult_Request * data = NULL;

  if (size) {
    data = (rover_control__action__DriveToPose_GetResult_Request *)allocator.zero_allocate(size, sizeof(rover_control__action__DriveToPose_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__DriveToPose_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__DriveToPose_GetResult_Request__fini(&data[i - 1]);
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
rover_control__action__DriveToPose_GetResult_Request__Sequence__fini(rover_control__action__DriveToPose_GetResult_Request__Sequence * array)
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
      rover_control__action__DriveToPose_GetResult_Request__fini(&array->data[i]);
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

rover_control__action__DriveToPose_GetResult_Request__Sequence *
rover_control__action__DriveToPose_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_GetResult_Request__Sequence * array = (rover_control__action__DriveToPose_GetResult_Request__Sequence *)allocator.allocate(sizeof(rover_control__action__DriveToPose_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__DriveToPose_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__DriveToPose_GetResult_Request__Sequence__destroy(rover_control__action__DriveToPose_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__DriveToPose_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__DriveToPose_GetResult_Request__Sequence__are_equal(const rover_control__action__DriveToPose_GetResult_Request__Sequence * lhs, const rover_control__action__DriveToPose_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__DriveToPose_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__DriveToPose_GetResult_Request__Sequence__copy(
  const rover_control__action__DriveToPose_GetResult_Request__Sequence * input,
  rover_control__action__DriveToPose_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__DriveToPose_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__DriveToPose_GetResult_Request * data =
      (rover_control__action__DriveToPose_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__DriveToPose_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__DriveToPose_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__DriveToPose_GetResult_Request__copy(
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
// #include "rover_control/action/detail/drive_to_pose__functions.h"

bool
rover_control__action__DriveToPose_GetResult_Response__init(rover_control__action__DriveToPose_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!rover_control__action__DriveToPose_Result__init(&msg->result)) {
    rover_control__action__DriveToPose_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
rover_control__action__DriveToPose_GetResult_Response__fini(rover_control__action__DriveToPose_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  rover_control__action__DriveToPose_Result__fini(&msg->result);
}

bool
rover_control__action__DriveToPose_GetResult_Response__are_equal(const rover_control__action__DriveToPose_GetResult_Response * lhs, const rover_control__action__DriveToPose_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!rover_control__action__DriveToPose_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
rover_control__action__DriveToPose_GetResult_Response__copy(
  const rover_control__action__DriveToPose_GetResult_Response * input,
  rover_control__action__DriveToPose_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!rover_control__action__DriveToPose_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

rover_control__action__DriveToPose_GetResult_Response *
rover_control__action__DriveToPose_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_GetResult_Response * msg = (rover_control__action__DriveToPose_GetResult_Response *)allocator.allocate(sizeof(rover_control__action__DriveToPose_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__DriveToPose_GetResult_Response));
  bool success = rover_control__action__DriveToPose_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__DriveToPose_GetResult_Response__destroy(rover_control__action__DriveToPose_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__DriveToPose_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__DriveToPose_GetResult_Response__Sequence__init(rover_control__action__DriveToPose_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_GetResult_Response * data = NULL;

  if (size) {
    data = (rover_control__action__DriveToPose_GetResult_Response *)allocator.zero_allocate(size, sizeof(rover_control__action__DriveToPose_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__DriveToPose_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__DriveToPose_GetResult_Response__fini(&data[i - 1]);
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
rover_control__action__DriveToPose_GetResult_Response__Sequence__fini(rover_control__action__DriveToPose_GetResult_Response__Sequence * array)
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
      rover_control__action__DriveToPose_GetResult_Response__fini(&array->data[i]);
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

rover_control__action__DriveToPose_GetResult_Response__Sequence *
rover_control__action__DriveToPose_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_GetResult_Response__Sequence * array = (rover_control__action__DriveToPose_GetResult_Response__Sequence *)allocator.allocate(sizeof(rover_control__action__DriveToPose_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__DriveToPose_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__DriveToPose_GetResult_Response__Sequence__destroy(rover_control__action__DriveToPose_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__DriveToPose_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__DriveToPose_GetResult_Response__Sequence__are_equal(const rover_control__action__DriveToPose_GetResult_Response__Sequence * lhs, const rover_control__action__DriveToPose_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__DriveToPose_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__DriveToPose_GetResult_Response__Sequence__copy(
  const rover_control__action__DriveToPose_GetResult_Response__Sequence * input,
  rover_control__action__DriveToPose_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__DriveToPose_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__DriveToPose_GetResult_Response * data =
      (rover_control__action__DriveToPose_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__DriveToPose_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__DriveToPose_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__DriveToPose_GetResult_Response__copy(
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
// #include "rover_control/action/detail/drive_to_pose__functions.h"

bool
rover_control__action__DriveToPose_FeedbackMessage__init(rover_control__action__DriveToPose_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    rover_control__action__DriveToPose_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!rover_control__action__DriveToPose_Feedback__init(&msg->feedback)) {
    rover_control__action__DriveToPose_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
rover_control__action__DriveToPose_FeedbackMessage__fini(rover_control__action__DriveToPose_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  rover_control__action__DriveToPose_Feedback__fini(&msg->feedback);
}

bool
rover_control__action__DriveToPose_FeedbackMessage__are_equal(const rover_control__action__DriveToPose_FeedbackMessage * lhs, const rover_control__action__DriveToPose_FeedbackMessage * rhs)
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
  if (!rover_control__action__DriveToPose_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
rover_control__action__DriveToPose_FeedbackMessage__copy(
  const rover_control__action__DriveToPose_FeedbackMessage * input,
  rover_control__action__DriveToPose_FeedbackMessage * output)
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
  if (!rover_control__action__DriveToPose_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

rover_control__action__DriveToPose_FeedbackMessage *
rover_control__action__DriveToPose_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_FeedbackMessage * msg = (rover_control__action__DriveToPose_FeedbackMessage *)allocator.allocate(sizeof(rover_control__action__DriveToPose_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__action__DriveToPose_FeedbackMessage));
  bool success = rover_control__action__DriveToPose_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__action__DriveToPose_FeedbackMessage__destroy(rover_control__action__DriveToPose_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__action__DriveToPose_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__action__DriveToPose_FeedbackMessage__Sequence__init(rover_control__action__DriveToPose_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_FeedbackMessage * data = NULL;

  if (size) {
    data = (rover_control__action__DriveToPose_FeedbackMessage *)allocator.zero_allocate(size, sizeof(rover_control__action__DriveToPose_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__action__DriveToPose_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__action__DriveToPose_FeedbackMessage__fini(&data[i - 1]);
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
rover_control__action__DriveToPose_FeedbackMessage__Sequence__fini(rover_control__action__DriveToPose_FeedbackMessage__Sequence * array)
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
      rover_control__action__DriveToPose_FeedbackMessage__fini(&array->data[i]);
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

rover_control__action__DriveToPose_FeedbackMessage__Sequence *
rover_control__action__DriveToPose_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__action__DriveToPose_FeedbackMessage__Sequence * array = (rover_control__action__DriveToPose_FeedbackMessage__Sequence *)allocator.allocate(sizeof(rover_control__action__DriveToPose_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__action__DriveToPose_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__action__DriveToPose_FeedbackMessage__Sequence__destroy(rover_control__action__DriveToPose_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__action__DriveToPose_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__action__DriveToPose_FeedbackMessage__Sequence__are_equal(const rover_control__action__DriveToPose_FeedbackMessage__Sequence * lhs, const rover_control__action__DriveToPose_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__action__DriveToPose_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__action__DriveToPose_FeedbackMessage__Sequence__copy(
  const rover_control__action__DriveToPose_FeedbackMessage__Sequence * input,
  rover_control__action__DriveToPose_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__action__DriveToPose_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__action__DriveToPose_FeedbackMessage * data =
      (rover_control__action__DriveToPose_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__action__DriveToPose_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__action__DriveToPose_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__action__DriveToPose_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
