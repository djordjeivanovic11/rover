// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from arm_control:action/ToolChange.idl
// generated code does not contain a copyright notice
#include "arm_control/action/detail/tool_change__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `new_tool_name`
// Member `current_tool_name`
// Member `change_strategy`
#include "rosidl_runtime_c/string_functions.h"
// Member `tool_dock_pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
arm_control__action__ToolChange_Goal__init(arm_control__action__ToolChange_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // new_tool_name
  if (!rosidl_runtime_c__String__init(&msg->new_tool_name)) {
    arm_control__action__ToolChange_Goal__fini(msg);
    return false;
  }
  // current_tool_name
  if (!rosidl_runtime_c__String__init(&msg->current_tool_name)) {
    arm_control__action__ToolChange_Goal__fini(msg);
    return false;
  }
  // auto_detect_current
  msg->auto_detect_current = false;
  // tool_dock_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->tool_dock_pose)) {
    arm_control__action__ToolChange_Goal__fini(msg);
    return false;
  }
  // approach_distance
  msg->approach_distance = 0.1f;
  // coupling_timeout
  msg->coupling_timeout = 10.0f;
  // verify_tool_change
  msg->verify_tool_change = true;
  // update_kinematics
  msg->update_kinematics = true;
  // velocity_scaling
  msg->velocity_scaling = 0.05f;
  // change_strategy
  if (!rosidl_runtime_c__String__init(&msg->change_strategy)) {
    arm_control__action__ToolChange_Goal__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->change_strategy, "auto");
    if (!success) {
      goto abort_init_0;
    }
  }
  return true;
abort_init_0:
  return false;
}

void
arm_control__action__ToolChange_Goal__fini(arm_control__action__ToolChange_Goal * msg)
{
  if (!msg) {
    return;
  }
  // new_tool_name
  rosidl_runtime_c__String__fini(&msg->new_tool_name);
  // current_tool_name
  rosidl_runtime_c__String__fini(&msg->current_tool_name);
  // auto_detect_current
  // tool_dock_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->tool_dock_pose);
  // approach_distance
  // coupling_timeout
  // verify_tool_change
  // update_kinematics
  // velocity_scaling
  // change_strategy
  rosidl_runtime_c__String__fini(&msg->change_strategy);
}

bool
arm_control__action__ToolChange_Goal__are_equal(const arm_control__action__ToolChange_Goal * lhs, const arm_control__action__ToolChange_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // new_tool_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->new_tool_name), &(rhs->new_tool_name)))
  {
    return false;
  }
  // current_tool_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_tool_name), &(rhs->current_tool_name)))
  {
    return false;
  }
  // auto_detect_current
  if (lhs->auto_detect_current != rhs->auto_detect_current) {
    return false;
  }
  // tool_dock_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->tool_dock_pose), &(rhs->tool_dock_pose)))
  {
    return false;
  }
  // approach_distance
  if (lhs->approach_distance != rhs->approach_distance) {
    return false;
  }
  // coupling_timeout
  if (lhs->coupling_timeout != rhs->coupling_timeout) {
    return false;
  }
  // verify_tool_change
  if (lhs->verify_tool_change != rhs->verify_tool_change) {
    return false;
  }
  // update_kinematics
  if (lhs->update_kinematics != rhs->update_kinematics) {
    return false;
  }
  // velocity_scaling
  if (lhs->velocity_scaling != rhs->velocity_scaling) {
    return false;
  }
  // change_strategy
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->change_strategy), &(rhs->change_strategy)))
  {
    return false;
  }
  return true;
}

bool
arm_control__action__ToolChange_Goal__copy(
  const arm_control__action__ToolChange_Goal * input,
  arm_control__action__ToolChange_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // new_tool_name
  if (!rosidl_runtime_c__String__copy(
      &(input->new_tool_name), &(output->new_tool_name)))
  {
    return false;
  }
  // current_tool_name
  if (!rosidl_runtime_c__String__copy(
      &(input->current_tool_name), &(output->current_tool_name)))
  {
    return false;
  }
  // auto_detect_current
  output->auto_detect_current = input->auto_detect_current;
  // tool_dock_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->tool_dock_pose), &(output->tool_dock_pose)))
  {
    return false;
  }
  // approach_distance
  output->approach_distance = input->approach_distance;
  // coupling_timeout
  output->coupling_timeout = input->coupling_timeout;
  // verify_tool_change
  output->verify_tool_change = input->verify_tool_change;
  // update_kinematics
  output->update_kinematics = input->update_kinematics;
  // velocity_scaling
  output->velocity_scaling = input->velocity_scaling;
  // change_strategy
  if (!rosidl_runtime_c__String__copy(
      &(input->change_strategy), &(output->change_strategy)))
  {
    return false;
  }
  return true;
}

arm_control__action__ToolChange_Goal *
arm_control__action__ToolChange_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_Goal * msg = (arm_control__action__ToolChange_Goal *)allocator.allocate(sizeof(arm_control__action__ToolChange_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__ToolChange_Goal));
  bool success = arm_control__action__ToolChange_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__ToolChange_Goal__destroy(arm_control__action__ToolChange_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__ToolChange_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__ToolChange_Goal__Sequence__init(arm_control__action__ToolChange_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_Goal * data = NULL;

  if (size) {
    data = (arm_control__action__ToolChange_Goal *)allocator.zero_allocate(size, sizeof(arm_control__action__ToolChange_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__ToolChange_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__ToolChange_Goal__fini(&data[i - 1]);
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
arm_control__action__ToolChange_Goal__Sequence__fini(arm_control__action__ToolChange_Goal__Sequence * array)
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
      arm_control__action__ToolChange_Goal__fini(&array->data[i]);
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

arm_control__action__ToolChange_Goal__Sequence *
arm_control__action__ToolChange_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_Goal__Sequence * array = (arm_control__action__ToolChange_Goal__Sequence *)allocator.allocate(sizeof(arm_control__action__ToolChange_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__ToolChange_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__ToolChange_Goal__Sequence__destroy(arm_control__action__ToolChange_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__ToolChange_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__ToolChange_Goal__Sequence__are_equal(const arm_control__action__ToolChange_Goal__Sequence * lhs, const arm_control__action__ToolChange_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__ToolChange_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__ToolChange_Goal__Sequence__copy(
  const arm_control__action__ToolChange_Goal__Sequence * input,
  arm_control__action__ToolChange_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__ToolChange_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__ToolChange_Goal * data =
      (arm_control__action__ToolChange_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__ToolChange_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__ToolChange_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__ToolChange_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `error_message`
// Member `old_tool_name`
// Member `new_tool_name`
// Member `tool_configuration`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `final_tool_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
arm_control__action__ToolChange_Result__init(arm_control__action__ToolChange_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    arm_control__action__ToolChange_Result__fini(msg);
    return false;
  }
  // error_code
  // old_tool_name
  if (!rosidl_runtime_c__String__init(&msg->old_tool_name)) {
    arm_control__action__ToolChange_Result__fini(msg);
    return false;
  }
  // new_tool_name
  if (!rosidl_runtime_c__String__init(&msg->new_tool_name)) {
    arm_control__action__ToolChange_Result__fini(msg);
    return false;
  }
  // coupling_verified
  // kinematics_updated
  // final_tool_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->final_tool_pose)) {
    arm_control__action__ToolChange_Result__fini(msg);
    return false;
  }
  // total_execution_time
  // tool_configuration
  if (!rosidl_runtime_c__String__init(&msg->tool_configuration)) {
    arm_control__action__ToolChange_Result__fini(msg);
    return false;
  }
  return true;
}

void
arm_control__action__ToolChange_Result__fini(arm_control__action__ToolChange_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
  // error_code
  // old_tool_name
  rosidl_runtime_c__String__fini(&msg->old_tool_name);
  // new_tool_name
  rosidl_runtime_c__String__fini(&msg->new_tool_name);
  // coupling_verified
  // kinematics_updated
  // final_tool_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->final_tool_pose);
  // total_execution_time
  // tool_configuration
  rosidl_runtime_c__String__fini(&msg->tool_configuration);
}

bool
arm_control__action__ToolChange_Result__are_equal(const arm_control__action__ToolChange_Result * lhs, const arm_control__action__ToolChange_Result * rhs)
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
  // old_tool_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->old_tool_name), &(rhs->old_tool_name)))
  {
    return false;
  }
  // new_tool_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->new_tool_name), &(rhs->new_tool_name)))
  {
    return false;
  }
  // coupling_verified
  if (lhs->coupling_verified != rhs->coupling_verified) {
    return false;
  }
  // kinematics_updated
  if (lhs->kinematics_updated != rhs->kinematics_updated) {
    return false;
  }
  // final_tool_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->final_tool_pose), &(rhs->final_tool_pose)))
  {
    return false;
  }
  // total_execution_time
  if (lhs->total_execution_time != rhs->total_execution_time) {
    return false;
  }
  // tool_configuration
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->tool_configuration), &(rhs->tool_configuration)))
  {
    return false;
  }
  return true;
}

bool
arm_control__action__ToolChange_Result__copy(
  const arm_control__action__ToolChange_Result * input,
  arm_control__action__ToolChange_Result * output)
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
  // old_tool_name
  if (!rosidl_runtime_c__String__copy(
      &(input->old_tool_name), &(output->old_tool_name)))
  {
    return false;
  }
  // new_tool_name
  if (!rosidl_runtime_c__String__copy(
      &(input->new_tool_name), &(output->new_tool_name)))
  {
    return false;
  }
  // coupling_verified
  output->coupling_verified = input->coupling_verified;
  // kinematics_updated
  output->kinematics_updated = input->kinematics_updated;
  // final_tool_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->final_tool_pose), &(output->final_tool_pose)))
  {
    return false;
  }
  // total_execution_time
  output->total_execution_time = input->total_execution_time;
  // tool_configuration
  if (!rosidl_runtime_c__String__copy(
      &(input->tool_configuration), &(output->tool_configuration)))
  {
    return false;
  }
  return true;
}

arm_control__action__ToolChange_Result *
arm_control__action__ToolChange_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_Result * msg = (arm_control__action__ToolChange_Result *)allocator.allocate(sizeof(arm_control__action__ToolChange_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__ToolChange_Result));
  bool success = arm_control__action__ToolChange_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__ToolChange_Result__destroy(arm_control__action__ToolChange_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__ToolChange_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__ToolChange_Result__Sequence__init(arm_control__action__ToolChange_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_Result * data = NULL;

  if (size) {
    data = (arm_control__action__ToolChange_Result *)allocator.zero_allocate(size, sizeof(arm_control__action__ToolChange_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__ToolChange_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__ToolChange_Result__fini(&data[i - 1]);
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
arm_control__action__ToolChange_Result__Sequence__fini(arm_control__action__ToolChange_Result__Sequence * array)
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
      arm_control__action__ToolChange_Result__fini(&array->data[i]);
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

arm_control__action__ToolChange_Result__Sequence *
arm_control__action__ToolChange_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_Result__Sequence * array = (arm_control__action__ToolChange_Result__Sequence *)allocator.allocate(sizeof(arm_control__action__ToolChange_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__ToolChange_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__ToolChange_Result__Sequence__destroy(arm_control__action__ToolChange_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__ToolChange_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__ToolChange_Result__Sequence__are_equal(const arm_control__action__ToolChange_Result__Sequence * lhs, const arm_control__action__ToolChange_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__ToolChange_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__ToolChange_Result__Sequence__copy(
  const arm_control__action__ToolChange_Result__Sequence * input,
  arm_control__action__ToolChange_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__ToolChange_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__ToolChange_Result * data =
      (arm_control__action__ToolChange_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__ToolChange_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__ToolChange_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__ToolChange_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `current_phase`
// Member `coupling_status`
// Member `kinematics_status`
// Member `status_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
arm_control__action__ToolChange_Feedback__init(arm_control__action__ToolChange_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // current_phase
  if (!rosidl_runtime_c__String__init(&msg->current_phase)) {
    arm_control__action__ToolChange_Feedback__fini(msg);
    return false;
  }
  // phase_progress
  // overall_progress
  // tool_detected
  // coupling_engaged
  // coupling_status
  if (!rosidl_runtime_c__String__init(&msg->coupling_status)) {
    arm_control__action__ToolChange_Feedback__fini(msg);
    return false;
  }
  // kinematics_status
  if (!rosidl_runtime_c__String__init(&msg->kinematics_status)) {
    arm_control__action__ToolChange_Feedback__fini(msg);
    return false;
  }
  // status_message
  if (!rosidl_runtime_c__String__init(&msg->status_message)) {
    arm_control__action__ToolChange_Feedback__fini(msg);
    return false;
  }
  // estimated_time_remaining
  return true;
}

void
arm_control__action__ToolChange_Feedback__fini(arm_control__action__ToolChange_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // current_phase
  rosidl_runtime_c__String__fini(&msg->current_phase);
  // phase_progress
  // overall_progress
  // tool_detected
  // coupling_engaged
  // coupling_status
  rosidl_runtime_c__String__fini(&msg->coupling_status);
  // kinematics_status
  rosidl_runtime_c__String__fini(&msg->kinematics_status);
  // status_message
  rosidl_runtime_c__String__fini(&msg->status_message);
  // estimated_time_remaining
}

bool
arm_control__action__ToolChange_Feedback__are_equal(const arm_control__action__ToolChange_Feedback * lhs, const arm_control__action__ToolChange_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current_phase
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_phase), &(rhs->current_phase)))
  {
    return false;
  }
  // phase_progress
  if (lhs->phase_progress != rhs->phase_progress) {
    return false;
  }
  // overall_progress
  if (lhs->overall_progress != rhs->overall_progress) {
    return false;
  }
  // tool_detected
  if (lhs->tool_detected != rhs->tool_detected) {
    return false;
  }
  // coupling_engaged
  if (lhs->coupling_engaged != rhs->coupling_engaged) {
    return false;
  }
  // coupling_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->coupling_status), &(rhs->coupling_status)))
  {
    return false;
  }
  // kinematics_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->kinematics_status), &(rhs->kinematics_status)))
  {
    return false;
  }
  // status_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status_message), &(rhs->status_message)))
  {
    return false;
  }
  // estimated_time_remaining
  if (lhs->estimated_time_remaining != rhs->estimated_time_remaining) {
    return false;
  }
  return true;
}

bool
arm_control__action__ToolChange_Feedback__copy(
  const arm_control__action__ToolChange_Feedback * input,
  arm_control__action__ToolChange_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // current_phase
  if (!rosidl_runtime_c__String__copy(
      &(input->current_phase), &(output->current_phase)))
  {
    return false;
  }
  // phase_progress
  output->phase_progress = input->phase_progress;
  // overall_progress
  output->overall_progress = input->overall_progress;
  // tool_detected
  output->tool_detected = input->tool_detected;
  // coupling_engaged
  output->coupling_engaged = input->coupling_engaged;
  // coupling_status
  if (!rosidl_runtime_c__String__copy(
      &(input->coupling_status), &(output->coupling_status)))
  {
    return false;
  }
  // kinematics_status
  if (!rosidl_runtime_c__String__copy(
      &(input->kinematics_status), &(output->kinematics_status)))
  {
    return false;
  }
  // status_message
  if (!rosidl_runtime_c__String__copy(
      &(input->status_message), &(output->status_message)))
  {
    return false;
  }
  // estimated_time_remaining
  output->estimated_time_remaining = input->estimated_time_remaining;
  return true;
}

arm_control__action__ToolChange_Feedback *
arm_control__action__ToolChange_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_Feedback * msg = (arm_control__action__ToolChange_Feedback *)allocator.allocate(sizeof(arm_control__action__ToolChange_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__ToolChange_Feedback));
  bool success = arm_control__action__ToolChange_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__ToolChange_Feedback__destroy(arm_control__action__ToolChange_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__ToolChange_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__ToolChange_Feedback__Sequence__init(arm_control__action__ToolChange_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_Feedback * data = NULL;

  if (size) {
    data = (arm_control__action__ToolChange_Feedback *)allocator.zero_allocate(size, sizeof(arm_control__action__ToolChange_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__ToolChange_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__ToolChange_Feedback__fini(&data[i - 1]);
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
arm_control__action__ToolChange_Feedback__Sequence__fini(arm_control__action__ToolChange_Feedback__Sequence * array)
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
      arm_control__action__ToolChange_Feedback__fini(&array->data[i]);
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

arm_control__action__ToolChange_Feedback__Sequence *
arm_control__action__ToolChange_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_Feedback__Sequence * array = (arm_control__action__ToolChange_Feedback__Sequence *)allocator.allocate(sizeof(arm_control__action__ToolChange_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__ToolChange_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__ToolChange_Feedback__Sequence__destroy(arm_control__action__ToolChange_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__ToolChange_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__ToolChange_Feedback__Sequence__are_equal(const arm_control__action__ToolChange_Feedback__Sequence * lhs, const arm_control__action__ToolChange_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__ToolChange_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__ToolChange_Feedback__Sequence__copy(
  const arm_control__action__ToolChange_Feedback__Sequence * input,
  arm_control__action__ToolChange_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__ToolChange_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__ToolChange_Feedback * data =
      (arm_control__action__ToolChange_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__ToolChange_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__ToolChange_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__ToolChange_Feedback__copy(
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
// #include "arm_control/action/detail/tool_change__functions.h"

bool
arm_control__action__ToolChange_SendGoal_Request__init(arm_control__action__ToolChange_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    arm_control__action__ToolChange_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!arm_control__action__ToolChange_Goal__init(&msg->goal)) {
    arm_control__action__ToolChange_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
arm_control__action__ToolChange_SendGoal_Request__fini(arm_control__action__ToolChange_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  arm_control__action__ToolChange_Goal__fini(&msg->goal);
}

bool
arm_control__action__ToolChange_SendGoal_Request__are_equal(const arm_control__action__ToolChange_SendGoal_Request * lhs, const arm_control__action__ToolChange_SendGoal_Request * rhs)
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
  if (!arm_control__action__ToolChange_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
arm_control__action__ToolChange_SendGoal_Request__copy(
  const arm_control__action__ToolChange_SendGoal_Request * input,
  arm_control__action__ToolChange_SendGoal_Request * output)
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
  if (!arm_control__action__ToolChange_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

arm_control__action__ToolChange_SendGoal_Request *
arm_control__action__ToolChange_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_SendGoal_Request * msg = (arm_control__action__ToolChange_SendGoal_Request *)allocator.allocate(sizeof(arm_control__action__ToolChange_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__ToolChange_SendGoal_Request));
  bool success = arm_control__action__ToolChange_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__ToolChange_SendGoal_Request__destroy(arm_control__action__ToolChange_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__ToolChange_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__ToolChange_SendGoal_Request__Sequence__init(arm_control__action__ToolChange_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_SendGoal_Request * data = NULL;

  if (size) {
    data = (arm_control__action__ToolChange_SendGoal_Request *)allocator.zero_allocate(size, sizeof(arm_control__action__ToolChange_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__ToolChange_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__ToolChange_SendGoal_Request__fini(&data[i - 1]);
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
arm_control__action__ToolChange_SendGoal_Request__Sequence__fini(arm_control__action__ToolChange_SendGoal_Request__Sequence * array)
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
      arm_control__action__ToolChange_SendGoal_Request__fini(&array->data[i]);
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

arm_control__action__ToolChange_SendGoal_Request__Sequence *
arm_control__action__ToolChange_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_SendGoal_Request__Sequence * array = (arm_control__action__ToolChange_SendGoal_Request__Sequence *)allocator.allocate(sizeof(arm_control__action__ToolChange_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__ToolChange_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__ToolChange_SendGoal_Request__Sequence__destroy(arm_control__action__ToolChange_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__ToolChange_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__ToolChange_SendGoal_Request__Sequence__are_equal(const arm_control__action__ToolChange_SendGoal_Request__Sequence * lhs, const arm_control__action__ToolChange_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__ToolChange_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__ToolChange_SendGoal_Request__Sequence__copy(
  const arm_control__action__ToolChange_SendGoal_Request__Sequence * input,
  arm_control__action__ToolChange_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__ToolChange_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__ToolChange_SendGoal_Request * data =
      (arm_control__action__ToolChange_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__ToolChange_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__ToolChange_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__ToolChange_SendGoal_Request__copy(
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
arm_control__action__ToolChange_SendGoal_Response__init(arm_control__action__ToolChange_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    arm_control__action__ToolChange_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
arm_control__action__ToolChange_SendGoal_Response__fini(arm_control__action__ToolChange_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
arm_control__action__ToolChange_SendGoal_Response__are_equal(const arm_control__action__ToolChange_SendGoal_Response * lhs, const arm_control__action__ToolChange_SendGoal_Response * rhs)
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
arm_control__action__ToolChange_SendGoal_Response__copy(
  const arm_control__action__ToolChange_SendGoal_Response * input,
  arm_control__action__ToolChange_SendGoal_Response * output)
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

arm_control__action__ToolChange_SendGoal_Response *
arm_control__action__ToolChange_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_SendGoal_Response * msg = (arm_control__action__ToolChange_SendGoal_Response *)allocator.allocate(sizeof(arm_control__action__ToolChange_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__ToolChange_SendGoal_Response));
  bool success = arm_control__action__ToolChange_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__ToolChange_SendGoal_Response__destroy(arm_control__action__ToolChange_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__ToolChange_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__ToolChange_SendGoal_Response__Sequence__init(arm_control__action__ToolChange_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_SendGoal_Response * data = NULL;

  if (size) {
    data = (arm_control__action__ToolChange_SendGoal_Response *)allocator.zero_allocate(size, sizeof(arm_control__action__ToolChange_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__ToolChange_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__ToolChange_SendGoal_Response__fini(&data[i - 1]);
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
arm_control__action__ToolChange_SendGoal_Response__Sequence__fini(arm_control__action__ToolChange_SendGoal_Response__Sequence * array)
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
      arm_control__action__ToolChange_SendGoal_Response__fini(&array->data[i]);
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

arm_control__action__ToolChange_SendGoal_Response__Sequence *
arm_control__action__ToolChange_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_SendGoal_Response__Sequence * array = (arm_control__action__ToolChange_SendGoal_Response__Sequence *)allocator.allocate(sizeof(arm_control__action__ToolChange_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__ToolChange_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__ToolChange_SendGoal_Response__Sequence__destroy(arm_control__action__ToolChange_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__ToolChange_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__ToolChange_SendGoal_Response__Sequence__are_equal(const arm_control__action__ToolChange_SendGoal_Response__Sequence * lhs, const arm_control__action__ToolChange_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__ToolChange_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__ToolChange_SendGoal_Response__Sequence__copy(
  const arm_control__action__ToolChange_SendGoal_Response__Sequence * input,
  arm_control__action__ToolChange_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__ToolChange_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__ToolChange_SendGoal_Response * data =
      (arm_control__action__ToolChange_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__ToolChange_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__ToolChange_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__ToolChange_SendGoal_Response__copy(
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
arm_control__action__ToolChange_GetResult_Request__init(arm_control__action__ToolChange_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    arm_control__action__ToolChange_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
arm_control__action__ToolChange_GetResult_Request__fini(arm_control__action__ToolChange_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
arm_control__action__ToolChange_GetResult_Request__are_equal(const arm_control__action__ToolChange_GetResult_Request * lhs, const arm_control__action__ToolChange_GetResult_Request * rhs)
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
arm_control__action__ToolChange_GetResult_Request__copy(
  const arm_control__action__ToolChange_GetResult_Request * input,
  arm_control__action__ToolChange_GetResult_Request * output)
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

arm_control__action__ToolChange_GetResult_Request *
arm_control__action__ToolChange_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_GetResult_Request * msg = (arm_control__action__ToolChange_GetResult_Request *)allocator.allocate(sizeof(arm_control__action__ToolChange_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__ToolChange_GetResult_Request));
  bool success = arm_control__action__ToolChange_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__ToolChange_GetResult_Request__destroy(arm_control__action__ToolChange_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__ToolChange_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__ToolChange_GetResult_Request__Sequence__init(arm_control__action__ToolChange_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_GetResult_Request * data = NULL;

  if (size) {
    data = (arm_control__action__ToolChange_GetResult_Request *)allocator.zero_allocate(size, sizeof(arm_control__action__ToolChange_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__ToolChange_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__ToolChange_GetResult_Request__fini(&data[i - 1]);
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
arm_control__action__ToolChange_GetResult_Request__Sequence__fini(arm_control__action__ToolChange_GetResult_Request__Sequence * array)
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
      arm_control__action__ToolChange_GetResult_Request__fini(&array->data[i]);
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

arm_control__action__ToolChange_GetResult_Request__Sequence *
arm_control__action__ToolChange_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_GetResult_Request__Sequence * array = (arm_control__action__ToolChange_GetResult_Request__Sequence *)allocator.allocate(sizeof(arm_control__action__ToolChange_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__ToolChange_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__ToolChange_GetResult_Request__Sequence__destroy(arm_control__action__ToolChange_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__ToolChange_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__ToolChange_GetResult_Request__Sequence__are_equal(const arm_control__action__ToolChange_GetResult_Request__Sequence * lhs, const arm_control__action__ToolChange_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__ToolChange_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__ToolChange_GetResult_Request__Sequence__copy(
  const arm_control__action__ToolChange_GetResult_Request__Sequence * input,
  arm_control__action__ToolChange_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__ToolChange_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__ToolChange_GetResult_Request * data =
      (arm_control__action__ToolChange_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__ToolChange_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__ToolChange_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__ToolChange_GetResult_Request__copy(
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
// #include "arm_control/action/detail/tool_change__functions.h"

bool
arm_control__action__ToolChange_GetResult_Response__init(arm_control__action__ToolChange_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!arm_control__action__ToolChange_Result__init(&msg->result)) {
    arm_control__action__ToolChange_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
arm_control__action__ToolChange_GetResult_Response__fini(arm_control__action__ToolChange_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  arm_control__action__ToolChange_Result__fini(&msg->result);
}

bool
arm_control__action__ToolChange_GetResult_Response__are_equal(const arm_control__action__ToolChange_GetResult_Response * lhs, const arm_control__action__ToolChange_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!arm_control__action__ToolChange_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
arm_control__action__ToolChange_GetResult_Response__copy(
  const arm_control__action__ToolChange_GetResult_Response * input,
  arm_control__action__ToolChange_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!arm_control__action__ToolChange_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

arm_control__action__ToolChange_GetResult_Response *
arm_control__action__ToolChange_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_GetResult_Response * msg = (arm_control__action__ToolChange_GetResult_Response *)allocator.allocate(sizeof(arm_control__action__ToolChange_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__ToolChange_GetResult_Response));
  bool success = arm_control__action__ToolChange_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__ToolChange_GetResult_Response__destroy(arm_control__action__ToolChange_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__ToolChange_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__ToolChange_GetResult_Response__Sequence__init(arm_control__action__ToolChange_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_GetResult_Response * data = NULL;

  if (size) {
    data = (arm_control__action__ToolChange_GetResult_Response *)allocator.zero_allocate(size, sizeof(arm_control__action__ToolChange_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__ToolChange_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__ToolChange_GetResult_Response__fini(&data[i - 1]);
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
arm_control__action__ToolChange_GetResult_Response__Sequence__fini(arm_control__action__ToolChange_GetResult_Response__Sequence * array)
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
      arm_control__action__ToolChange_GetResult_Response__fini(&array->data[i]);
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

arm_control__action__ToolChange_GetResult_Response__Sequence *
arm_control__action__ToolChange_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_GetResult_Response__Sequence * array = (arm_control__action__ToolChange_GetResult_Response__Sequence *)allocator.allocate(sizeof(arm_control__action__ToolChange_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__ToolChange_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__ToolChange_GetResult_Response__Sequence__destroy(arm_control__action__ToolChange_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__ToolChange_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__ToolChange_GetResult_Response__Sequence__are_equal(const arm_control__action__ToolChange_GetResult_Response__Sequence * lhs, const arm_control__action__ToolChange_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__ToolChange_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__ToolChange_GetResult_Response__Sequence__copy(
  const arm_control__action__ToolChange_GetResult_Response__Sequence * input,
  arm_control__action__ToolChange_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__ToolChange_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__ToolChange_GetResult_Response * data =
      (arm_control__action__ToolChange_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__ToolChange_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__ToolChange_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__ToolChange_GetResult_Response__copy(
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
// #include "arm_control/action/detail/tool_change__functions.h"

bool
arm_control__action__ToolChange_FeedbackMessage__init(arm_control__action__ToolChange_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    arm_control__action__ToolChange_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!arm_control__action__ToolChange_Feedback__init(&msg->feedback)) {
    arm_control__action__ToolChange_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
arm_control__action__ToolChange_FeedbackMessage__fini(arm_control__action__ToolChange_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  arm_control__action__ToolChange_Feedback__fini(&msg->feedback);
}

bool
arm_control__action__ToolChange_FeedbackMessage__are_equal(const arm_control__action__ToolChange_FeedbackMessage * lhs, const arm_control__action__ToolChange_FeedbackMessage * rhs)
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
  if (!arm_control__action__ToolChange_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
arm_control__action__ToolChange_FeedbackMessage__copy(
  const arm_control__action__ToolChange_FeedbackMessage * input,
  arm_control__action__ToolChange_FeedbackMessage * output)
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
  if (!arm_control__action__ToolChange_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

arm_control__action__ToolChange_FeedbackMessage *
arm_control__action__ToolChange_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_FeedbackMessage * msg = (arm_control__action__ToolChange_FeedbackMessage *)allocator.allocate(sizeof(arm_control__action__ToolChange_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__action__ToolChange_FeedbackMessage));
  bool success = arm_control__action__ToolChange_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__action__ToolChange_FeedbackMessage__destroy(arm_control__action__ToolChange_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__action__ToolChange_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__action__ToolChange_FeedbackMessage__Sequence__init(arm_control__action__ToolChange_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_FeedbackMessage * data = NULL;

  if (size) {
    data = (arm_control__action__ToolChange_FeedbackMessage *)allocator.zero_allocate(size, sizeof(arm_control__action__ToolChange_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__action__ToolChange_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__action__ToolChange_FeedbackMessage__fini(&data[i - 1]);
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
arm_control__action__ToolChange_FeedbackMessage__Sequence__fini(arm_control__action__ToolChange_FeedbackMessage__Sequence * array)
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
      arm_control__action__ToolChange_FeedbackMessage__fini(&array->data[i]);
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

arm_control__action__ToolChange_FeedbackMessage__Sequence *
arm_control__action__ToolChange_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__action__ToolChange_FeedbackMessage__Sequence * array = (arm_control__action__ToolChange_FeedbackMessage__Sequence *)allocator.allocate(sizeof(arm_control__action__ToolChange_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__action__ToolChange_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__action__ToolChange_FeedbackMessage__Sequence__destroy(arm_control__action__ToolChange_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__action__ToolChange_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__action__ToolChange_FeedbackMessage__Sequence__are_equal(const arm_control__action__ToolChange_FeedbackMessage__Sequence * lhs, const arm_control__action__ToolChange_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__action__ToolChange_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__action__ToolChange_FeedbackMessage__Sequence__copy(
  const arm_control__action__ToolChange_FeedbackMessage__Sequence * input,
  arm_control__action__ToolChange_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__action__ToolChange_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__action__ToolChange_FeedbackMessage * data =
      (arm_control__action__ToolChange_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__action__ToolChange_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__action__ToolChange_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__action__ToolChange_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
