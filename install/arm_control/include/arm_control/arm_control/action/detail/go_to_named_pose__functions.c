// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from arm_control:action/GoToNamedPose.idl
// generated code does not contain a copyright notice
#include "arm_control/action/detail/go_to_named_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose_name`
// Member `planning_group`
#include "rosidl_runtime_c/string_functions.h"

bool
arm_control__action__GoToNamedPose_Goal__init(arm_control__action__GoToNamedPose_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // pose_name
  if (!rosidl_runtime_c__String__init(&msg->pose_name)) {
    arm_control__action__GoToNamedPose_Goal__fini(msg);
    return false;
  }
  // velocity_scaling
  msg->velocity_scaling = 0.1f;
  // acceleration_scaling
  msg->acceleration_scaling = 0.1f;
  // plan_only
  msg->plan_only = false;
  // planning_timeout
  msg->planning_timeout = 5.0f;
  // execution_timeout
  msg->execution_timeout = 30.0f;
  // collision_checking
  msg->collision_checking = true;
  // planning_group
  if (!rosidl_runtime_c__String__init(&msg->planning_group)) {
    arm_control__action__GoToNamedPose_Goal__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->planning_group, "arm");
    if (!success) {
      goto abort_init_0;
    }
  }
  return true;
abort_init_0:
  return false;
}

void
arm_control__action__GoToNamedPose_Goal__fini(arm_control__action__GoToNamedPose_Goal * msg)
{
  if (!msg) {
    return;
  }
  // pose_name
  rosidl_runtime_c__String__fini(&msg->pose_name);
  // velocity_scaling
  // acceleration_scaling
  // plan_only
  // planning_timeout
  // execution_timeout
  // collision_checking
  // planning_group
  rosidl_runtime_c__String__fini(&msg->planning_group);
}

bool
arm_control__action__GoToNamedPose_Goal__are_equal(const arm_control__action__GoToNamedPose_Goal * lhs, const arm_control__action__GoToNamedPose_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->pose_name), &(rhs->pose_name)))
  {
    return false;
  }
  // velocity_scaling
  if (lhs->velocity_scaling != rhs->velocity_scaling) {
    return false;
  }
  // acceleration_scaling
  if (lhs->acceleration_scaling != rhs->acceleration_scaling) {
    return false;
  }
  // plan_only
  if (lhs->plan_only != rhs->plan_only) {
    return false;
  }
  // planning_timeout
  if (lhs->planning_timeout != rhs->planning_timeout) {
    return false;
  }
  // execution_timeout
  if (lhs->execution_timeout != rhs->execution_timeout) {
    return false;
  }
  // collision_checking
  if (lhs->collision_checking != rhs->collision_checking) {
    return false;
  }
  // planning_group
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->planning_group), &(rhs->planning_group)))
  {
    return false;
  }
  return true;
}

bool
arm_control__action__GoToNamedPose_Goal__copy(
  const arm_control__action__GoToNamedPose_Goal * input,
  arm_control__action__GoToNamedPose_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // pose_name
  if (!rosidl_runtime_c__String__copy(
      &(input->pose_name), &(output->pose_name)))
  {
    return false;
  }
  // velocity_scaling
  output->velocity_scaling = input->velocity_scaling;
  // acceleration_scaling
  output->acceleration_scaling = input->acceleration_scaling;
  // plan_only
  output->plan_only = input->plan_only;
  // planning_timeout
  output->planning_timeout = input->planning_timeout;
  // execution_timeout
  output->execution_timeout = input->execution_timeout;
  // collision_checking
  output->collision_checking = input->collision_checking;
  // planning_group
  if (!rosidl_runtime_c__String__copy(
      &(input->planning_group), &(output->planning_group)))
  {
    return false;
  }
  return true;
}

arm_control__action__GoToNamedPose_Goal *
arm_control__action__GoToNamedPose_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_Goal * msg = (arm_control__action__GoToNamedPose_Goal *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__GoToNamedPose_Goal));
  bool success = arm_control__action__GoToNamedPose_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__GoToNamedPose_Goal__destroy(arm_control__action__GoToNamedPose_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__GoToNamedPose_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__GoToNamedPose_Goal__Sequence__init(arm_control__action__GoToNamedPose_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_Goal * data = NULL;

  if (size) {
    data = (arm_control__action__GoToNamedPose_Goal *)allocator.zero_allocate(size, sizeof(arm_control__action__GoToNamedPose_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__GoToNamedPose_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__GoToNamedPose_Goal__fini(&data[i - 1]);
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
arm_control__action__GoToNamedPose_Goal__Sequence__fini(arm_control__action__GoToNamedPose_Goal__Sequence * array)
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
      arm_control__action__GoToNamedPose_Goal__fini(&array->data[i]);
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

arm_control__action__GoToNamedPose_Goal__Sequence *
arm_control__action__GoToNamedPose_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_Goal__Sequence * array = (arm_control__action__GoToNamedPose_Goal__Sequence *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__GoToNamedPose_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__GoToNamedPose_Goal__Sequence__destroy(arm_control__action__GoToNamedPose_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__GoToNamedPose_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__GoToNamedPose_Goal__Sequence__are_equal(const arm_control__action__GoToNamedPose_Goal__Sequence * lhs, const arm_control__action__GoToNamedPose_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__GoToNamedPose_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__GoToNamedPose_Goal__Sequence__copy(
  const arm_control__action__GoToNamedPose_Goal__Sequence * input,
  arm_control__action__GoToNamedPose_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__GoToNamedPose_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__GoToNamedPose_Goal * data =
      (arm_control__action__GoToNamedPose_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__GoToNamedPose_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__GoToNamedPose_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__GoToNamedPose_Goal__copy(
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
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
arm_control__action__GoToNamedPose_Result__init(arm_control__action__GoToNamedPose_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    arm_control__action__GoToNamedPose_Result__fini(msg);
    return false;
  }
  // error_code
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->final_pose)) {
    arm_control__action__GoToNamedPose_Result__fini(msg);
    return false;
  }
  // planning_time
  // execution_time
  // final_position_error
  // final_orientation_error
  return true;
}

void
arm_control__action__GoToNamedPose_Result__fini(arm_control__action__GoToNamedPose_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
  // error_code
  // final_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->final_pose);
  // planning_time
  // execution_time
  // final_position_error
  // final_orientation_error
}

bool
arm_control__action__GoToNamedPose_Result__are_equal(const arm_control__action__GoToNamedPose_Result * lhs, const arm_control__action__GoToNamedPose_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_message), &(rhs->error_message)))
  {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->final_pose), &(rhs->final_pose)))
  {
    return false;
  }
  // planning_time
  if (lhs->planning_time != rhs->planning_time) {
    return false;
  }
  // execution_time
  if (lhs->execution_time != rhs->execution_time) {
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
  return true;
}

bool
arm_control__action__GoToNamedPose_Result__copy(
  const arm_control__action__GoToNamedPose_Result * input,
  arm_control__action__GoToNamedPose_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // error_message
  if (!rosidl_runtime_c__String__copy(
      &(input->error_message), &(output->error_message)))
  {
    return false;
  }
  // error_code
  output->error_code = input->error_code;
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->final_pose), &(output->final_pose)))
  {
    return false;
  }
  // planning_time
  output->planning_time = input->planning_time;
  // execution_time
  output->execution_time = input->execution_time;
  // final_position_error
  output->final_position_error = input->final_position_error;
  // final_orientation_error
  output->final_orientation_error = input->final_orientation_error;
  return true;
}

arm_control__action__GoToNamedPose_Result *
arm_control__action__GoToNamedPose_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_Result * msg = (arm_control__action__GoToNamedPose_Result *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__GoToNamedPose_Result));
  bool success = arm_control__action__GoToNamedPose_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__GoToNamedPose_Result__destroy(arm_control__action__GoToNamedPose_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__GoToNamedPose_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__GoToNamedPose_Result__Sequence__init(arm_control__action__GoToNamedPose_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_Result * data = NULL;

  if (size) {
    data = (arm_control__action__GoToNamedPose_Result *)allocator.zero_allocate(size, sizeof(arm_control__action__GoToNamedPose_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__GoToNamedPose_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__GoToNamedPose_Result__fini(&data[i - 1]);
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
arm_control__action__GoToNamedPose_Result__Sequence__fini(arm_control__action__GoToNamedPose_Result__Sequence * array)
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
      arm_control__action__GoToNamedPose_Result__fini(&array->data[i]);
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

arm_control__action__GoToNamedPose_Result__Sequence *
arm_control__action__GoToNamedPose_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_Result__Sequence * array = (arm_control__action__GoToNamedPose_Result__Sequence *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__GoToNamedPose_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__GoToNamedPose_Result__Sequence__destroy(arm_control__action__GoToNamedPose_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__GoToNamedPose_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__GoToNamedPose_Result__Sequence__are_equal(const arm_control__action__GoToNamedPose_Result__Sequence * lhs, const arm_control__action__GoToNamedPose_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__GoToNamedPose_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__GoToNamedPose_Result__Sequence__copy(
  const arm_control__action__GoToNamedPose_Result__Sequence * input,
  arm_control__action__GoToNamedPose_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__GoToNamedPose_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__GoToNamedPose_Result * data =
      (arm_control__action__GoToNamedPose_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__GoToNamedPose_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__GoToNamedPose_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__GoToNamedPose_Result__copy(
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
// Member `current_joint_state`
#include "sensor_msgs/msg/detail/joint_state__functions.h"

bool
arm_control__action__GoToNamedPose_Feedback__init(arm_control__action__GoToNamedPose_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // current_state
  if (!rosidl_runtime_c__String__init(&msg->current_state)) {
    arm_control__action__GoToNamedPose_Feedback__fini(msg);
    return false;
  }
  // progress_percentage
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->current_pose)) {
    arm_control__action__GoToNamedPose_Feedback__fini(msg);
    return false;
  }
  // estimated_time_remaining
  // current_joint_state
  if (!sensor_msgs__msg__JointState__init(&msg->current_joint_state)) {
    arm_control__action__GoToNamedPose_Feedback__fini(msg);
    return false;
  }
  // collision_detected
  // status_message
  if (!rosidl_runtime_c__String__init(&msg->status_message)) {
    arm_control__action__GoToNamedPose_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
arm_control__action__GoToNamedPose_Feedback__fini(arm_control__action__GoToNamedPose_Feedback * msg)
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
  // current_joint_state
  sensor_msgs__msg__JointState__fini(&msg->current_joint_state);
  // collision_detected
  // status_message
  rosidl_runtime_c__String__fini(&msg->status_message);
}

bool
arm_control__action__GoToNamedPose_Feedback__are_equal(const arm_control__action__GoToNamedPose_Feedback * lhs, const arm_control__action__GoToNamedPose_Feedback * rhs)
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
  // current_joint_state
  if (!sensor_msgs__msg__JointState__are_equal(
      &(lhs->current_joint_state), &(rhs->current_joint_state)))
  {
    return false;
  }
  // collision_detected
  if (lhs->collision_detected != rhs->collision_detected) {
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
arm_control__action__GoToNamedPose_Feedback__copy(
  const arm_control__action__GoToNamedPose_Feedback * input,
  arm_control__action__GoToNamedPose_Feedback * output)
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
  // current_joint_state
  if (!sensor_msgs__msg__JointState__copy(
      &(input->current_joint_state), &(output->current_joint_state)))
  {
    return false;
  }
  // collision_detected
  output->collision_detected = input->collision_detected;
  // status_message
  if (!rosidl_runtime_c__String__copy(
      &(input->status_message), &(output->status_message)))
  {
    return false;
  }
  return true;
}

arm_control__action__GoToNamedPose_Feedback *
arm_control__action__GoToNamedPose_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_Feedback * msg = (arm_control__action__GoToNamedPose_Feedback *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__GoToNamedPose_Feedback));
  bool success = arm_control__action__GoToNamedPose_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__GoToNamedPose_Feedback__destroy(arm_control__action__GoToNamedPose_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__GoToNamedPose_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__GoToNamedPose_Feedback__Sequence__init(arm_control__action__GoToNamedPose_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_Feedback * data = NULL;

  if (size) {
    data = (arm_control__action__GoToNamedPose_Feedback *)allocator.zero_allocate(size, sizeof(arm_control__action__GoToNamedPose_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__GoToNamedPose_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__GoToNamedPose_Feedback__fini(&data[i - 1]);
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
arm_control__action__GoToNamedPose_Feedback__Sequence__fini(arm_control__action__GoToNamedPose_Feedback__Sequence * array)
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
      arm_control__action__GoToNamedPose_Feedback__fini(&array->data[i]);
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

arm_control__action__GoToNamedPose_Feedback__Sequence *
arm_control__action__GoToNamedPose_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_Feedback__Sequence * array = (arm_control__action__GoToNamedPose_Feedback__Sequence *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__GoToNamedPose_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__GoToNamedPose_Feedback__Sequence__destroy(arm_control__action__GoToNamedPose_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__GoToNamedPose_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__GoToNamedPose_Feedback__Sequence__are_equal(const arm_control__action__GoToNamedPose_Feedback__Sequence * lhs, const arm_control__action__GoToNamedPose_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__GoToNamedPose_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__GoToNamedPose_Feedback__Sequence__copy(
  const arm_control__action__GoToNamedPose_Feedback__Sequence * input,
  arm_control__action__GoToNamedPose_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__GoToNamedPose_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__GoToNamedPose_Feedback * data =
      (arm_control__action__GoToNamedPose_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__GoToNamedPose_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__GoToNamedPose_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__GoToNamedPose_Feedback__copy(
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
// #include "arm_control/action/detail/go_to_named_pose__functions.h"

bool
arm_control__action__GoToNamedPose_SendGoal_Request__init(arm_control__action__GoToNamedPose_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    arm_control__action__GoToNamedPose_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!arm_control__action__GoToNamedPose_Goal__init(&msg->goal)) {
    arm_control__action__GoToNamedPose_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
arm_control__action__GoToNamedPose_SendGoal_Request__fini(arm_control__action__GoToNamedPose_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  arm_control__action__GoToNamedPose_Goal__fini(&msg->goal);
}

bool
arm_control__action__GoToNamedPose_SendGoal_Request__are_equal(const arm_control__action__GoToNamedPose_SendGoal_Request * lhs, const arm_control__action__GoToNamedPose_SendGoal_Request * rhs)
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
  if (!arm_control__action__GoToNamedPose_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
arm_control__action__GoToNamedPose_SendGoal_Request__copy(
  const arm_control__action__GoToNamedPose_SendGoal_Request * input,
  arm_control__action__GoToNamedPose_SendGoal_Request * output)
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
  if (!arm_control__action__GoToNamedPose_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

arm_control__action__GoToNamedPose_SendGoal_Request *
arm_control__action__GoToNamedPose_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_SendGoal_Request * msg = (arm_control__action__GoToNamedPose_SendGoal_Request *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__GoToNamedPose_SendGoal_Request));
  bool success = arm_control__action__GoToNamedPose_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__GoToNamedPose_SendGoal_Request__destroy(arm_control__action__GoToNamedPose_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__GoToNamedPose_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__GoToNamedPose_SendGoal_Request__Sequence__init(arm_control__action__GoToNamedPose_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_SendGoal_Request * data = NULL;

  if (size) {
    data = (arm_control__action__GoToNamedPose_SendGoal_Request *)allocator.zero_allocate(size, sizeof(arm_control__action__GoToNamedPose_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__GoToNamedPose_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__GoToNamedPose_SendGoal_Request__fini(&data[i - 1]);
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
arm_control__action__GoToNamedPose_SendGoal_Request__Sequence__fini(arm_control__action__GoToNamedPose_SendGoal_Request__Sequence * array)
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
      arm_control__action__GoToNamedPose_SendGoal_Request__fini(&array->data[i]);
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

arm_control__action__GoToNamedPose_SendGoal_Request__Sequence *
arm_control__action__GoToNamedPose_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_SendGoal_Request__Sequence * array = (arm_control__action__GoToNamedPose_SendGoal_Request__Sequence *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__GoToNamedPose_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__GoToNamedPose_SendGoal_Request__Sequence__destroy(arm_control__action__GoToNamedPose_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__GoToNamedPose_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__GoToNamedPose_SendGoal_Request__Sequence__are_equal(const arm_control__action__GoToNamedPose_SendGoal_Request__Sequence * lhs, const arm_control__action__GoToNamedPose_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__GoToNamedPose_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__GoToNamedPose_SendGoal_Request__Sequence__copy(
  const arm_control__action__GoToNamedPose_SendGoal_Request__Sequence * input,
  arm_control__action__GoToNamedPose_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__GoToNamedPose_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__GoToNamedPose_SendGoal_Request * data =
      (arm_control__action__GoToNamedPose_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__GoToNamedPose_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__GoToNamedPose_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__GoToNamedPose_SendGoal_Request__copy(
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
arm_control__action__GoToNamedPose_SendGoal_Response__init(arm_control__action__GoToNamedPose_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    arm_control__action__GoToNamedPose_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
arm_control__action__GoToNamedPose_SendGoal_Response__fini(arm_control__action__GoToNamedPose_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
arm_control__action__GoToNamedPose_SendGoal_Response__are_equal(const arm_control__action__GoToNamedPose_SendGoal_Response * lhs, const arm_control__action__GoToNamedPose_SendGoal_Response * rhs)
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
arm_control__action__GoToNamedPose_SendGoal_Response__copy(
  const arm_control__action__GoToNamedPose_SendGoal_Response * input,
  arm_control__action__GoToNamedPose_SendGoal_Response * output)
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

arm_control__action__GoToNamedPose_SendGoal_Response *
arm_control__action__GoToNamedPose_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_SendGoal_Response * msg = (arm_control__action__GoToNamedPose_SendGoal_Response *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__GoToNamedPose_SendGoal_Response));
  bool success = arm_control__action__GoToNamedPose_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__GoToNamedPose_SendGoal_Response__destroy(arm_control__action__GoToNamedPose_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__GoToNamedPose_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__GoToNamedPose_SendGoal_Response__Sequence__init(arm_control__action__GoToNamedPose_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_SendGoal_Response * data = NULL;

  if (size) {
    data = (arm_control__action__GoToNamedPose_SendGoal_Response *)allocator.zero_allocate(size, sizeof(arm_control__action__GoToNamedPose_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__GoToNamedPose_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__GoToNamedPose_SendGoal_Response__fini(&data[i - 1]);
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
arm_control__action__GoToNamedPose_SendGoal_Response__Sequence__fini(arm_control__action__GoToNamedPose_SendGoal_Response__Sequence * array)
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
      arm_control__action__GoToNamedPose_SendGoal_Response__fini(&array->data[i]);
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

arm_control__action__GoToNamedPose_SendGoal_Response__Sequence *
arm_control__action__GoToNamedPose_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_SendGoal_Response__Sequence * array = (arm_control__action__GoToNamedPose_SendGoal_Response__Sequence *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__GoToNamedPose_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__GoToNamedPose_SendGoal_Response__Sequence__destroy(arm_control__action__GoToNamedPose_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__GoToNamedPose_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__GoToNamedPose_SendGoal_Response__Sequence__are_equal(const arm_control__action__GoToNamedPose_SendGoal_Response__Sequence * lhs, const arm_control__action__GoToNamedPose_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__GoToNamedPose_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__GoToNamedPose_SendGoal_Response__Sequence__copy(
  const arm_control__action__GoToNamedPose_SendGoal_Response__Sequence * input,
  arm_control__action__GoToNamedPose_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__GoToNamedPose_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__GoToNamedPose_SendGoal_Response * data =
      (arm_control__action__GoToNamedPose_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__GoToNamedPose_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__GoToNamedPose_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__GoToNamedPose_SendGoal_Response__copy(
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
arm_control__action__GoToNamedPose_GetResult_Request__init(arm_control__action__GoToNamedPose_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    arm_control__action__GoToNamedPose_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
arm_control__action__GoToNamedPose_GetResult_Request__fini(arm_control__action__GoToNamedPose_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
arm_control__action__GoToNamedPose_GetResult_Request__are_equal(const arm_control__action__GoToNamedPose_GetResult_Request * lhs, const arm_control__action__GoToNamedPose_GetResult_Request * rhs)
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
arm_control__action__GoToNamedPose_GetResult_Request__copy(
  const arm_control__action__GoToNamedPose_GetResult_Request * input,
  arm_control__action__GoToNamedPose_GetResult_Request * output)
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

arm_control__action__GoToNamedPose_GetResult_Request *
arm_control__action__GoToNamedPose_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_GetResult_Request * msg = (arm_control__action__GoToNamedPose_GetResult_Request *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__GoToNamedPose_GetResult_Request));
  bool success = arm_control__action__GoToNamedPose_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__GoToNamedPose_GetResult_Request__destroy(arm_control__action__GoToNamedPose_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__GoToNamedPose_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__GoToNamedPose_GetResult_Request__Sequence__init(arm_control__action__GoToNamedPose_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_GetResult_Request * data = NULL;

  if (size) {
    data = (arm_control__action__GoToNamedPose_GetResult_Request *)allocator.zero_allocate(size, sizeof(arm_control__action__GoToNamedPose_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__GoToNamedPose_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__GoToNamedPose_GetResult_Request__fini(&data[i - 1]);
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
arm_control__action__GoToNamedPose_GetResult_Request__Sequence__fini(arm_control__action__GoToNamedPose_GetResult_Request__Sequence * array)
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
      arm_control__action__GoToNamedPose_GetResult_Request__fini(&array->data[i]);
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

arm_control__action__GoToNamedPose_GetResult_Request__Sequence *
arm_control__action__GoToNamedPose_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_GetResult_Request__Sequence * array = (arm_control__action__GoToNamedPose_GetResult_Request__Sequence *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__GoToNamedPose_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__GoToNamedPose_GetResult_Request__Sequence__destroy(arm_control__action__GoToNamedPose_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__GoToNamedPose_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__GoToNamedPose_GetResult_Request__Sequence__are_equal(const arm_control__action__GoToNamedPose_GetResult_Request__Sequence * lhs, const arm_control__action__GoToNamedPose_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__GoToNamedPose_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__GoToNamedPose_GetResult_Request__Sequence__copy(
  const arm_control__action__GoToNamedPose_GetResult_Request__Sequence * input,
  arm_control__action__GoToNamedPose_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__GoToNamedPose_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__GoToNamedPose_GetResult_Request * data =
      (arm_control__action__GoToNamedPose_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__GoToNamedPose_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__GoToNamedPose_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__GoToNamedPose_GetResult_Request__copy(
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
// #include "arm_control/action/detail/go_to_named_pose__functions.h"

bool
arm_control__action__GoToNamedPose_GetResult_Response__init(arm_control__action__GoToNamedPose_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!arm_control__action__GoToNamedPose_Result__init(&msg->result)) {
    arm_control__action__GoToNamedPose_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
arm_control__action__GoToNamedPose_GetResult_Response__fini(arm_control__action__GoToNamedPose_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  arm_control__action__GoToNamedPose_Result__fini(&msg->result);
}

bool
arm_control__action__GoToNamedPose_GetResult_Response__are_equal(const arm_control__action__GoToNamedPose_GetResult_Response * lhs, const arm_control__action__GoToNamedPose_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!arm_control__action__GoToNamedPose_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
arm_control__action__GoToNamedPose_GetResult_Response__copy(
  const arm_control__action__GoToNamedPose_GetResult_Response * input,
  arm_control__action__GoToNamedPose_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!arm_control__action__GoToNamedPose_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

arm_control__action__GoToNamedPose_GetResult_Response *
arm_control__action__GoToNamedPose_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_GetResult_Response * msg = (arm_control__action__GoToNamedPose_GetResult_Response *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__GoToNamedPose_GetResult_Response));
  bool success = arm_control__action__GoToNamedPose_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__GoToNamedPose_GetResult_Response__destroy(arm_control__action__GoToNamedPose_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__GoToNamedPose_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__GoToNamedPose_GetResult_Response__Sequence__init(arm_control__action__GoToNamedPose_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_GetResult_Response * data = NULL;

  if (size) {
    data = (arm_control__action__GoToNamedPose_GetResult_Response *)allocator.zero_allocate(size, sizeof(arm_control__action__GoToNamedPose_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__GoToNamedPose_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__GoToNamedPose_GetResult_Response__fini(&data[i - 1]);
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
arm_control__action__GoToNamedPose_GetResult_Response__Sequence__fini(arm_control__action__GoToNamedPose_GetResult_Response__Sequence * array)
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
      arm_control__action__GoToNamedPose_GetResult_Response__fini(&array->data[i]);
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

arm_control__action__GoToNamedPose_GetResult_Response__Sequence *
arm_control__action__GoToNamedPose_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_GetResult_Response__Sequence * array = (arm_control__action__GoToNamedPose_GetResult_Response__Sequence *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__GoToNamedPose_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__GoToNamedPose_GetResult_Response__Sequence__destroy(arm_control__action__GoToNamedPose_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__GoToNamedPose_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__GoToNamedPose_GetResult_Response__Sequence__are_equal(const arm_control__action__GoToNamedPose_GetResult_Response__Sequence * lhs, const arm_control__action__GoToNamedPose_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__GoToNamedPose_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__GoToNamedPose_GetResult_Response__Sequence__copy(
  const arm_control__action__GoToNamedPose_GetResult_Response__Sequence * input,
  arm_control__action__GoToNamedPose_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__GoToNamedPose_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__GoToNamedPose_GetResult_Response * data =
      (arm_control__action__GoToNamedPose_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__GoToNamedPose_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__GoToNamedPose_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__GoToNamedPose_GetResult_Response__copy(
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
// #include "arm_control/action/detail/go_to_named_pose__functions.h"

bool
arm_control__action__GoToNamedPose_FeedbackMessage__init(arm_control__action__GoToNamedPose_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    arm_control__action__GoToNamedPose_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!arm_control__action__GoToNamedPose_Feedback__init(&msg->feedback)) {
    arm_control__action__GoToNamedPose_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
arm_control__action__GoToNamedPose_FeedbackMessage__fini(arm_control__action__GoToNamedPose_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  arm_control__action__GoToNamedPose_Feedback__fini(&msg->feedback);
}

bool
arm_control__action__GoToNamedPose_FeedbackMessage__are_equal(const arm_control__action__GoToNamedPose_FeedbackMessage * lhs, const arm_control__action__GoToNamedPose_FeedbackMessage * rhs)
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
  if (!arm_control__action__GoToNamedPose_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
arm_control__action__GoToNamedPose_FeedbackMessage__copy(
  const arm_control__action__GoToNamedPose_FeedbackMessage * input,
  arm_control__action__GoToNamedPose_FeedbackMessage * output)
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
  if (!arm_control__action__GoToNamedPose_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

arm_control__action__GoToNamedPose_FeedbackMessage *
arm_control__action__GoToNamedPose_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_FeedbackMessage * msg = (arm_control__action__GoToNamedPose_FeedbackMessage *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__GoToNamedPose_FeedbackMessage));
  bool success = arm_control__action__GoToNamedPose_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__GoToNamedPose_FeedbackMessage__destroy(arm_control__action__GoToNamedPose_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__GoToNamedPose_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__GoToNamedPose_FeedbackMessage__Sequence__init(arm_control__action__GoToNamedPose_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_FeedbackMessage * data = NULL;

  if (size) {
    data = (arm_control__action__GoToNamedPose_FeedbackMessage *)allocator.zero_allocate(size, sizeof(arm_control__action__GoToNamedPose_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__GoToNamedPose_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__GoToNamedPose_FeedbackMessage__fini(&data[i - 1]);
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
arm_control__action__GoToNamedPose_FeedbackMessage__Sequence__fini(arm_control__action__GoToNamedPose_FeedbackMessage__Sequence * array)
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
      arm_control__action__GoToNamedPose_FeedbackMessage__fini(&array->data[i]);
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

arm_control__action__GoToNamedPose_FeedbackMessage__Sequence *
arm_control__action__GoToNamedPose_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__GoToNamedPose_FeedbackMessage__Sequence * array = (arm_control__action__GoToNamedPose_FeedbackMessage__Sequence *)allocator.allocate(sizeof(arm_control__action__GoToNamedPose_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__GoToNamedPose_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__GoToNamedPose_FeedbackMessage__Sequence__destroy(arm_control__action__GoToNamedPose_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__GoToNamedPose_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__GoToNamedPose_FeedbackMessage__Sequence__are_equal(const arm_control__action__GoToNamedPose_FeedbackMessage__Sequence * lhs, const arm_control__action__GoToNamedPose_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__GoToNamedPose_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__GoToNamedPose_FeedbackMessage__Sequence__copy(
  const arm_control__action__GoToNamedPose_FeedbackMessage__Sequence * input,
  arm_control__action__GoToNamedPose_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__GoToNamedPose_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__GoToNamedPose_FeedbackMessage * data =
      (arm_control__action__GoToNamedPose_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__GoToNamedPose_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__GoToNamedPose_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__GoToNamedPose_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
