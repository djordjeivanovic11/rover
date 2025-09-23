// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from urc_msgs:msg/ScienceData.idl
// generated code does not contain a copyright notice
#include "urc_msgs/msg/detail/science_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `sample_id`
// Member `detected_compounds`
#include "rosidl_runtime_c/string_functions.h"
// Member `sample_location`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
urc_msgs__msg__ScienceData__init(urc_msgs__msg__ScienceData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    urc_msgs__msg__ScienceData__fini(msg);
    return false;
  }
  // sample_id
  if (!rosidl_runtime_c__String__init(&msg->sample_id)) {
    urc_msgs__msg__ScienceData__fini(msg);
    return false;
  }
  // sample_location
  if (!geometry_msgs__msg__Point__init(&msg->sample_location)) {
    urc_msgs__msg__ScienceData__fini(msg);
    return false;
  }
  // ph_level
  // temperature
  // moisture
  // detected_compounds
  if (!rosidl_runtime_c__String__Sequence__init(&msg->detected_compounds, 0)) {
    urc_msgs__msg__ScienceData__fini(msg);
    return false;
  }
  return true;
}

void
urc_msgs__msg__ScienceData__fini(urc_msgs__msg__ScienceData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // sample_id
  rosidl_runtime_c__String__fini(&msg->sample_id);
  // sample_location
  geometry_msgs__msg__Point__fini(&msg->sample_location);
  // ph_level
  // temperature
  // moisture
  // detected_compounds
  rosidl_runtime_c__String__Sequence__fini(&msg->detected_compounds);
}

bool
urc_msgs__msg__ScienceData__are_equal(const urc_msgs__msg__ScienceData * lhs, const urc_msgs__msg__ScienceData * rhs)
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
  // sample_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sample_id), &(rhs->sample_id)))
  {
    return false;
  }
  // sample_location
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->sample_location), &(rhs->sample_location)))
  {
    return false;
  }
  // ph_level
  if (lhs->ph_level != rhs->ph_level) {
    return false;
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // moisture
  if (lhs->moisture != rhs->moisture) {
    return false;
  }
  // detected_compounds
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->detected_compounds), &(rhs->detected_compounds)))
  {
    return false;
  }
  return true;
}

bool
urc_msgs__msg__ScienceData__copy(
  const urc_msgs__msg__ScienceData * input,
  urc_msgs__msg__ScienceData * output)
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
  // sample_id
  if (!rosidl_runtime_c__String__copy(
      &(input->sample_id), &(output->sample_id)))
  {
    return false;
  }
  // sample_location
  if (!geometry_msgs__msg__Point__copy(
      &(input->sample_location), &(output->sample_location)))
  {
    return false;
  }
  // ph_level
  output->ph_level = input->ph_level;
  // temperature
  output->temperature = input->temperature;
  // moisture
  output->moisture = input->moisture;
  // detected_compounds
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->detected_compounds), &(output->detected_compounds)))
  {
    return false;
  }
  return true;
}

urc_msgs__msg__ScienceData *
urc_msgs__msg__ScienceData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  urc_msgs__msg__ScienceData * msg = (urc_msgs__msg__ScienceData *)allocator.allocate(sizeof(urc_msgs__msg__ScienceData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(urc_msgs__msg__ScienceData));
  bool success = urc_msgs__msg__ScienceData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
urc_msgs__msg__ScienceData__destroy(urc_msgs__msg__ScienceData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    urc_msgs__msg__ScienceData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
urc_msgs__msg__ScienceData__Sequence__init(urc_msgs__msg__ScienceData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  urc_msgs__msg__ScienceData * data = NULL;

  if (size) {
    data = (urc_msgs__msg__ScienceData *)allocator.zero_allocate(size, sizeof(urc_msgs__msg__ScienceData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = urc_msgs__msg__ScienceData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        urc_msgs__msg__ScienceData__fini(&data[i - 1]);
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
urc_msgs__msg__ScienceData__Sequence__fini(urc_msgs__msg__ScienceData__Sequence * array)
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
      urc_msgs__msg__ScienceData__fini(&array->data[i]);
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

urc_msgs__msg__ScienceData__Sequence *
urc_msgs__msg__ScienceData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  urc_msgs__msg__ScienceData__Sequence * array = (urc_msgs__msg__ScienceData__Sequence *)allocator.allocate(sizeof(urc_msgs__msg__ScienceData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = urc_msgs__msg__ScienceData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
urc_msgs__msg__ScienceData__Sequence__destroy(urc_msgs__msg__ScienceData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    urc_msgs__msg__ScienceData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
urc_msgs__msg__ScienceData__Sequence__are_equal(const urc_msgs__msg__ScienceData__Sequence * lhs, const urc_msgs__msg__ScienceData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!urc_msgs__msg__ScienceData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
urc_msgs__msg__ScienceData__Sequence__copy(
  const urc_msgs__msg__ScienceData__Sequence * input,
  urc_msgs__msg__ScienceData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(urc_msgs__msg__ScienceData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    urc_msgs__msg__ScienceData * data =
      (urc_msgs__msg__ScienceData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!urc_msgs__msg__ScienceData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          urc_msgs__msg__ScienceData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!urc_msgs__msg__ScienceData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
