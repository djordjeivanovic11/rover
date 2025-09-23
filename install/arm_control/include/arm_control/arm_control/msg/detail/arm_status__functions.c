// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from arm_control:msg/ArmStatus.idl
// generated code does not contain a copyright notice
#include "arm_control/msg/detail/arm_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `current_pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `joint_state`
#include "sensor_msgs/msg/detail/joint_state__functions.h"
// Member `current_tool_name`
// Member `error_message`
// Member `current_mission`
// Member `current_action`
#include "rosidl_runtime_c/string_functions.h"
// Member `tcp_velocity`
// Member `tcp_angular_velocity`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `current_wrench`
#include "geometry_msgs/msg/detail/wrench_stamped__functions.h"
// Member `joint_temperatures`
// Member `joint_currents`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
arm_control__msg__ArmStatus__init(arm_control__msg__ArmStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    arm_control__msg__ArmStatus__fini(msg);
    return false;
  }
  // state
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->current_pose)) {
    arm_control__msg__ArmStatus__fini(msg);
    return false;
  }
  // joint_state
  if (!sensor_msgs__msg__JointState__init(&msg->joint_state)) {
    arm_control__msg__ArmStatus__fini(msg);
    return false;
  }
  // current_tool_name
  if (!rosidl_runtime_c__String__init(&msg->current_tool_name)) {
    arm_control__msg__ArmStatus__fini(msg);
    return false;
  }
  // tool_attached
  // gripper_opening
  // gripper_force
  // safety_ok
  // estop_active
  // error_code
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    arm_control__msg__ArmStatus__fini(msg);
    return false;
  }
  // in_motion
  // velocity_norm
  // tcp_velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->tcp_velocity)) {
    arm_control__msg__ArmStatus__fini(msg);
    return false;
  }
  // tcp_angular_velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->tcp_angular_velocity)) {
    arm_control__msg__ArmStatus__fini(msg);
    return false;
  }
  // current_wrench
  if (!geometry_msgs__msg__WrenchStamped__init(&msg->current_wrench)) {
    arm_control__msg__ArmStatus__fini(msg);
    return false;
  }
  // force_limit_active
  // joint_temperatures
  if (!rosidl_runtime_c__float__Sequence__init(&msg->joint_temperatures, 0)) {
    arm_control__msg__ArmStatus__fini(msg);
    return false;
  }
  // max_temperature
  // temperature_warning
  // joint_currents
  if (!rosidl_runtime_c__float__Sequence__init(&msg->joint_currents, 0)) {
    arm_control__msg__ArmStatus__fini(msg);
    return false;
  }
  // max_current
  // current_warning
  // workspace_valid
  // workspace_margin
  // current_mission
  if (!rosidl_runtime_c__String__init(&msg->current_mission)) {
    arm_control__msg__ArmStatus__fini(msg);
    return false;
  }
  // current_action
  if (!rosidl_runtime_c__String__init(&msg->current_action)) {
    arm_control__msg__ArmStatus__fini(msg);
    return false;
  }
  // action_progress
  // system_uptime
  // command_count
  // fault_count
  // success_rate
  return true;
}

void
arm_control__msg__ArmStatus__fini(arm_control__msg__ArmStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
  // current_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->current_pose);
  // joint_state
  sensor_msgs__msg__JointState__fini(&msg->joint_state);
  // current_tool_name
  rosidl_runtime_c__String__fini(&msg->current_tool_name);
  // tool_attached
  // gripper_opening
  // gripper_force
  // safety_ok
  // estop_active
  // error_code
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
  // in_motion
  // velocity_norm
  // tcp_velocity
  geometry_msgs__msg__Vector3__fini(&msg->tcp_velocity);
  // tcp_angular_velocity
  geometry_msgs__msg__Vector3__fini(&msg->tcp_angular_velocity);
  // current_wrench
  geometry_msgs__msg__WrenchStamped__fini(&msg->current_wrench);
  // force_limit_active
  // joint_temperatures
  rosidl_runtime_c__float__Sequence__fini(&msg->joint_temperatures);
  // max_temperature
  // temperature_warning
  // joint_currents
  rosidl_runtime_c__float__Sequence__fini(&msg->joint_currents);
  // max_current
  // current_warning
  // workspace_valid
  // workspace_margin
  // current_mission
  rosidl_runtime_c__String__fini(&msg->current_mission);
  // current_action
  rosidl_runtime_c__String__fini(&msg->current_action);
  // action_progress
  // system_uptime
  // command_count
  // fault_count
  // success_rate
}

bool
arm_control__msg__ArmStatus__are_equal(const arm_control__msg__ArmStatus * lhs, const arm_control__msg__ArmStatus * rhs)
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
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->current_pose), &(rhs->current_pose)))
  {
    return false;
  }
  // joint_state
  if (!sensor_msgs__msg__JointState__are_equal(
      &(lhs->joint_state), &(rhs->joint_state)))
  {
    return false;
  }
  // current_tool_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_tool_name), &(rhs->current_tool_name)))
  {
    return false;
  }
  // tool_attached
  if (lhs->tool_attached != rhs->tool_attached) {
    return false;
  }
  // gripper_opening
  if (lhs->gripper_opening != rhs->gripper_opening) {
    return false;
  }
  // gripper_force
  if (lhs->gripper_force != rhs->gripper_force) {
    return false;
  }
  // safety_ok
  if (lhs->safety_ok != rhs->safety_ok) {
    return false;
  }
  // estop_active
  if (lhs->estop_active != rhs->estop_active) {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_message), &(rhs->error_message)))
  {
    return false;
  }
  // in_motion
  if (lhs->in_motion != rhs->in_motion) {
    return false;
  }
  // velocity_norm
  if (lhs->velocity_norm != rhs->velocity_norm) {
    return false;
  }
  // tcp_velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->tcp_velocity), &(rhs->tcp_velocity)))
  {
    return false;
  }
  // tcp_angular_velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->tcp_angular_velocity), &(rhs->tcp_angular_velocity)))
  {
    return false;
  }
  // current_wrench
  if (!geometry_msgs__msg__WrenchStamped__are_equal(
      &(lhs->current_wrench), &(rhs->current_wrench)))
  {
    return false;
  }
  // force_limit_active
  if (lhs->force_limit_active != rhs->force_limit_active) {
    return false;
  }
  // joint_temperatures
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->joint_temperatures), &(rhs->joint_temperatures)))
  {
    return false;
  }
  // max_temperature
  if (lhs->max_temperature != rhs->max_temperature) {
    return false;
  }
  // temperature_warning
  if (lhs->temperature_warning != rhs->temperature_warning) {
    return false;
  }
  // joint_currents
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->joint_currents), &(rhs->joint_currents)))
  {
    return false;
  }
  // max_current
  if (lhs->max_current != rhs->max_current) {
    return false;
  }
  // current_warning
  if (lhs->current_warning != rhs->current_warning) {
    return false;
  }
  // workspace_valid
  if (lhs->workspace_valid != rhs->workspace_valid) {
    return false;
  }
  // workspace_margin
  if (lhs->workspace_margin != rhs->workspace_margin) {
    return false;
  }
  // current_mission
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_mission), &(rhs->current_mission)))
  {
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
  // system_uptime
  if (lhs->system_uptime != rhs->system_uptime) {
    return false;
  }
  // command_count
  if (lhs->command_count != rhs->command_count) {
    return false;
  }
  // fault_count
  if (lhs->fault_count != rhs->fault_count) {
    return false;
  }
  // success_rate
  if (lhs->success_rate != rhs->success_rate) {
    return false;
  }
  return true;
}

bool
arm_control__msg__ArmStatus__copy(
  const arm_control__msg__ArmStatus * input,
  arm_control__msg__ArmStatus * output)
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
  // state
  output->state = input->state;
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->current_pose), &(output->current_pose)))
  {
    return false;
  }
  // joint_state
  if (!sensor_msgs__msg__JointState__copy(
      &(input->joint_state), &(output->joint_state)))
  {
    return false;
  }
  // current_tool_name
  if (!rosidl_runtime_c__String__copy(
      &(input->current_tool_name), &(output->current_tool_name)))
  {
    return false;
  }
  // tool_attached
  output->tool_attached = input->tool_attached;
  // gripper_opening
  output->gripper_opening = input->gripper_opening;
  // gripper_force
  output->gripper_force = input->gripper_force;
  // safety_ok
  output->safety_ok = input->safety_ok;
  // estop_active
  output->estop_active = input->estop_active;
  // error_code
  output->error_code = input->error_code;
  // error_message
  if (!rosidl_runtime_c__String__copy(
      &(input->error_message), &(output->error_message)))
  {
    return false;
  }
  // in_motion
  output->in_motion = input->in_motion;
  // velocity_norm
  output->velocity_norm = input->velocity_norm;
  // tcp_velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->tcp_velocity), &(output->tcp_velocity)))
  {
    return false;
  }
  // tcp_angular_velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->tcp_angular_velocity), &(output->tcp_angular_velocity)))
  {
    return false;
  }
  // current_wrench
  if (!geometry_msgs__msg__WrenchStamped__copy(
      &(input->current_wrench), &(output->current_wrench)))
  {
    return false;
  }
  // force_limit_active
  output->force_limit_active = input->force_limit_active;
  // joint_temperatures
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->joint_temperatures), &(output->joint_temperatures)))
  {
    return false;
  }
  // max_temperature
  output->max_temperature = input->max_temperature;
  // temperature_warning
  output->temperature_warning = input->temperature_warning;
  // joint_currents
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->joint_currents), &(output->joint_currents)))
  {
    return false;
  }
  // max_current
  output->max_current = input->max_current;
  // current_warning
  output->current_warning = input->current_warning;
  // workspace_valid
  output->workspace_valid = input->workspace_valid;
  // workspace_margin
  output->workspace_margin = input->workspace_margin;
  // current_mission
  if (!rosidl_runtime_c__String__copy(
      &(input->current_mission), &(output->current_mission)))
  {
    return false;
  }
  // current_action
  if (!rosidl_runtime_c__String__copy(
      &(input->current_action), &(output->current_action)))
  {
    return false;
  }
  // action_progress
  output->action_progress = input->action_progress;
  // system_uptime
  output->system_uptime = input->system_uptime;
  // command_count
  output->command_count = input->command_count;
  // fault_count
  output->fault_count = input->fault_count;
  // success_rate
  output->success_rate = input->success_rate;
  return true;
}

arm_control__msg__ArmStatus *
arm_control__msg__ArmStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__msg__ArmStatus * msg = (arm_control__msg__ArmStatus *)allocator.allocate(sizeof(arm_control__msg__ArmStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arm_control__msg__ArmStatus));
  bool success = arm_control__msg__ArmStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arm_control__msg__ArmStatus__destroy(arm_control__msg__ArmStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arm_control__msg__ArmStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arm_control__msg__ArmStatus__Sequence__init(arm_control__msg__ArmStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__msg__ArmStatus * data = NULL;

  if (size) {
    data = (arm_control__msg__ArmStatus *)allocator.zero_allocate(size, sizeof(arm_control__msg__ArmStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arm_control__msg__ArmStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arm_control__msg__ArmStatus__fini(&data[i - 1]);
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
arm_control__msg__ArmStatus__Sequence__fini(arm_control__msg__ArmStatus__Sequence * array)
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
      arm_control__msg__ArmStatus__fini(&array->data[i]);
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

arm_control__msg__ArmStatus__Sequence *
arm_control__msg__ArmStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arm_control__msg__ArmStatus__Sequence * array = (arm_control__msg__ArmStatus__Sequence *)allocator.allocate(sizeof(arm_control__msg__ArmStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arm_control__msg__ArmStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arm_control__msg__ArmStatus__Sequence__destroy(arm_control__msg__ArmStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arm_control__msg__ArmStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arm_control__msg__ArmStatus__Sequence__are_equal(const arm_control__msg__ArmStatus__Sequence * lhs, const arm_control__msg__ArmStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arm_control__msg__ArmStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arm_control__msg__ArmStatus__Sequence__copy(
  const arm_control__msg__ArmStatus__Sequence * input,
  arm_control__msg__ArmStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arm_control__msg__ArmStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arm_control__msg__ArmStatus * data =
      (arm_control__msg__ArmStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arm_control__msg__ArmStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arm_control__msg__ArmStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arm_control__msg__ArmStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
