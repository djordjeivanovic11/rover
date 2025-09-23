// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from arm_control:msg/Fault.idl
// generated code does not contain a copyright notice
#include "arm_control/msg/detail/fault__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `fault_category`
// Member `fault_description`
// Member `component_name`
// Member `joint_name`
// Member `fault_units`
// Member `reference_frame`
// Member `recovery_action`
// Member `current_action`
// Member `related_faults`
// Member `recommended_action`
// Member `troubleshooting_info`
#include "rosidl_runtime_c/string_functions.h"
// Member `fault_location`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `time_since_last`
#include "builtin_interfaces/msg/detail/duration__functions.h"

bool
arm_control__msg__Fault__init(arm_control__msg__Fault * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    arm_control__msg__Fault__fini(msg);
    return false;
  }
  // severity
  // fault_code
  // fault_category
  if (!rosidl_runtime_c__String__init(&msg->fault_category)) {
    arm_control__msg__Fault__fini(msg);
    return false;
  }
  // fault_description
  if (!rosidl_runtime_c__String__init(&msg->fault_description)) {
    arm_control__msg__Fault__fini(msg);
    return false;
  }
  // component_name
  if (!rosidl_runtime_c__String__init(&msg->component_name)) {
    arm_control__msg__Fault__fini(msg);
    return false;
  }
  // joint_name
  if (!rosidl_runtime_c__String__init(&msg->joint_name)) {
    arm_control__msg__Fault__fini(msg);
    return false;
  }
  // fault_value
  // fault_threshold
  // fault_units
  if (!rosidl_runtime_c__String__init(&msg->fault_units)) {
    arm_control__msg__Fault__fini(msg);
    return false;
  }
  // fault_location
  if (!geometry_msgs__msg__Point__init(&msg->fault_location)) {
    arm_control__msg__Fault__fini(msg);
    return false;
  }
  // reference_frame
  if (!rosidl_runtime_c__String__init(&msg->reference_frame)) {
    arm_control__msg__Fault__fini(msg);
    return false;
  }
  // auto_recoverable
  // recovery_attempted
  // recovery_successful
  // recovery_action
  if (!rosidl_runtime_c__String__init(&msg->recovery_action)) {
    arm_control__msg__Fault__fini(msg);
    return false;
  }
  // system_state
  // current_action
  if (!rosidl_runtime_c__String__init(&msg->current_action)) {
    arm_control__msg__Fault__fini(msg);
    return false;
  }
  // action_progress
  // related_faults
  if (!rosidl_runtime_c__String__Sequence__init(&msg->related_faults, 0)) {
    arm_control__msg__Fault__fini(msg);
    return false;
  }
  // occurrence_count
  // time_since_last
  if (!builtin_interfaces__msg__Duration__init(&msg->time_since_last)) {
    arm_control__msg__Fault__fini(msg);
    return false;
  }
  // recommended_action
  if (!rosidl_runtime_c__String__init(&msg->recommended_action)) {
    arm_control__msg__Fault__fini(msg);
    return false;
  }
  // operator_intervention_required
  // troubleshooting_info
  if (!rosidl_runtime_c__String__init(&msg->troubleshooting_info)) {
    arm_control__msg__Fault__fini(msg);
    return false;
  }
  return true;
}

void
arm_control__msg__Fault__fini(arm_control__msg__Fault * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // severity
  // fault_code
  // fault_category
  rosidl_runtime_c__String__fini(&msg->fault_category);
  // fault_description
  rosidl_runtime_c__String__fini(&msg->fault_description);
  // component_name
  rosidl_runtime_c__String__fini(&msg->component_name);
  // joint_name
  rosidl_runtime_c__String__fini(&msg->joint_name);
  // fault_value
  // fault_threshold
  // fault_units
  rosidl_runtime_c__String__fini(&msg->fault_units);
  // fault_location
  geometry_msgs__msg__Point__fini(&msg->fault_location);
  // reference_frame
  rosidl_runtime_c__String__fini(&msg->reference_frame);
  // auto_recoverable
  // recovery_attempted
  // recovery_successful
  // recovery_action
  rosidl_runtime_c__String__fini(&msg->recovery_action);
  // system_state
  // current_action
  rosidl_runtime_c__String__fini(&msg->current_action);
  // action_progress
  // related_faults
  rosidl_runtime_c__String__Sequence__fini(&msg->related_faults);
  // occurrence_count
  // time_since_last
  builtin_interfaces__msg__Duration__fini(&msg->time_since_last);
  // recommended_action
  rosidl_runtime_c__String__fini(&msg->recommended_action);
  // operator_intervention_required
  // troubleshooting_info
  rosidl_runtime_c__String__fini(&msg->troubleshooting_info);
}

bool
arm_control__msg__Fault__are_equal(const arm_control__msg__Fault * lhs, const arm_control__msg__Fault * rhs)
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
  // severity
  if (lhs->severity != rhs->severity) {
    return false;
  }
  // fault_code
  if (lhs->fault_code != rhs->fault_code) {
    return false;
  }
  // fault_category
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->fault_category), &(rhs->fault_category)))
  {
    return false;
  }
  // fault_description
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->fault_description), &(rhs->fault_description)))
  {
    return false;
  }
  // component_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->component_name), &(rhs->component_name)))
  {
    return false;
  }
  // joint_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->joint_name), &(rhs->joint_name)))
  {
    return false;
  }
  // fault_value
  if (lhs->fault_value != rhs->fault_value) {
    return false;
  }
  // fault_threshold
  if (lhs->fault_threshold != rhs->fault_threshold) {
    return false;
  }
  // fault_units
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->fault_units), &(rhs->fault_units)))
  {
    return false;
  }
  // fault_location
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->fault_location), &(rhs->fault_location)))
  {
    return false;
  }
  // reference_frame
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reference_frame), &(rhs->reference_frame)))
  {
    return false;
  }
  // auto_recoverable
  if (lhs->auto_recoverable != rhs->auto_recoverable) {
    return false;
  }
  // recovery_attempted
  if (lhs->recovery_attempted != rhs->recovery_attempted) {
    return false;
  }
  // recovery_successful
  if (lhs->recovery_successful != rhs->recovery_successful) {
    return false;
  }
  // recovery_action
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->recovery_action), &(rhs->recovery_action)))
  {
    return false;
  }
  // system_state
  if (lhs->system_state != rhs->system_state) {
    return false;
  }
  // current_action
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_action), &(rhs->current_action)))
  {
    return false;
  }
  // action_progress
  if (lhs->action_progress != rhs->action_progress) {
    return false;
  }
  // related_faults
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->related_faults), &(rhs->related_faults)))
  {
    return false;
  }
  // occurrence_count
  if (lhs->occurrence_count != rhs->occurrence_count) {
    return false;
  }
  // time_since_last
  if (!builtin_interfaces__msg__Duration__are_equal(
      &(lhs->time_since_last), &(rhs->time_since_last)))
  {
    return false;
  }
  // recommended_action
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->recommended_action), &(rhs->recommended_action)))
  {
    return false;
  }
  // operator_intervention_required
  if (lhs->operator_intervention_required != rhs->operator_intervention_required) {
    return false;
  }
  // troubleshooting_info
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->troubleshooting_info), &(rhs->troubleshooting_info)))
  {
    return false;
  }
  return true;
}

bool
arm_control__msg__Fault__copy(
  const arm_control__msg__Fault * input,
  arm_control__msg__Fault * output)
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
  // severity
  output->severity = input->severity;
  // fault_code
  output->fault_code = input->fault_code;
  // fault_category
  if (!rosidl_runtime_c__String__copy(
      &(input->fault_category), &(output->fault_category)))
  {
    return false;
  }
  // fault_description
  if (!rosidl_runtime_c__String__copy(
      &(input->fault_description), &(output->fault_description)))
  {
    return false;
  }
  // component_name
  if (!rosidl_runtime_c__String__copy(
      &(input->component_name), &(output->component_name)))
  {
    return false;
  }
  // joint_name
  if (!rosidl_runtime_c__String__copy(
      &(input->joint_name), &(output->joint_name)))
  {
    return false;
  }
  // fault_value
  output->fault_value = input->fault_value;
  // fault_threshold
  output->fault_threshold = input->fault_threshold;
  // fault_units
  if (!rosidl_runtime_c__String__copy(
      &(input->fault_units), &(output->fault_units)))
  {
    return false;
  }
  // fault_location
  if (!geometry_msgs__msg__Point__copy(
      &(input->fault_location), &(output->fault_location)))
  {
    return false;
  }
  // reference_frame
  if (!rosidl_runtime_c__String__copy(
      &(input->reference_frame), &(output->reference_frame)))
  {
    return false;
  }
  // auto_recoverable
  output->auto_recoverable = input->auto_recoverable;
  // recovery_attempted
  output->recovery_attempted = input->recovery_attempted;
  // recovery_successful
  output->recovery_successful = input->recovery_successful;
  // recovery_action
  if (!rosidl_runtime_c__String__copy(
      &(input->recovery_action), &(output->recovery_action)))
  {
    return false;
  }
  // system_state
  output->system_state = input->system_state;
  // current_action
  if (!rosidl_runtime_c__String__copy(
      &(input->current_action), &(output->current_action)))
  {
    return false;
  }
  // action_progress
  output->action_progress = input->action_progress;
  // related_faults
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->related_faults), &(output->related_faults)))
  {
    return false;
  }
  // occurrence_count
  output->occurrence_count = input->occurrence_count;
  // time_since_last
  if (!builtin_interfaces__msg__Duration__copy(
      &(input->time_since_last), &(output->time_since_last)))
  {
    return false;
  }
  // recommended_action
  if (!rosidl_runtime_c__String__copy(
      &(input->recommended_action), &(output->recommended_action)))
  {
    return false;
  }
  // operator_intervention_required
  output->operator_intervention_required = input->operator_intervention_required;
  // troubleshooting_info
  if (!rosidl_runtime_c__String__copy(
      &(input->troubleshooting_info), &(output->troubleshooting_info)))
  {
    return false;
  }
  return true;
}

arm_control__msg__Fault *
arm_control__msg__Fault__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__msg__Fault * msg = (arm_control__msg__Fault *)allocator.allocate(sizeof(arm_control__msg__Fault), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__msg__Fault));
  bool success = arm_control__msg__Fault__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__msg__Fault__destroy(arm_control__msg__Fault * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__msg__Fault__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__msg__Fault__Sequence__init(arm_control__msg__Fault__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__msg__Fault * data = NULL;

  if (size) {
    data = (arm_control__msg__Fault *)allocator.zero_allocate(size, sizeof(arm_control__msg__Fault), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__msg__Fault__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__msg__Fault__fini(&data[i - 1]);
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
arm_control__msg__Fault__Sequence__fini(arm_control__msg__Fault__Sequence * array)
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
      arm_control__msg__Fault__fini(&array->data[i]);
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

arm_control__msg__Fault__Sequence *
arm_control__msg__Fault__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__msg__Fault__Sequence * array = (arm_control__msg__Fault__Sequence *)allocator.allocate(sizeof(arm_control__msg__Fault__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__msg__Fault__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__msg__Fault__Sequence__destroy(arm_control__msg__Fault__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__msg__Fault__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__msg__Fault__Sequence__are_equal(const arm_control__msg__Fault__Sequence * lhs, const arm_control__msg__Fault__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__msg__Fault__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__msg__Fault__Sequence__copy(
  const arm_control__msg__Fault__Sequence * input,
  arm_control__msg__Fault__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__msg__Fault);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__msg__Fault * data =
      (arm_control__msg__Fault *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__msg__Fault__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__msg__Fault__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__msg__Fault__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
