// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_control:msg/RoverStatus.idl
// generated code does not contain a copyright notice
#include "rover_control/msg/detail/rover_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `current_action`
// Member `mission_mode`
// Member `fault_messages`
// Member `competition_mode`
#include "rosidl_runtime_c/string_functions.h"
// Member `current_pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `current_velocity`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `odometry`
#include "nav_msgs/msg/detail/odometry__functions.h"
// Member `motor_currents`
// Member `motor_temperatures`
// Member `active_faults`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `acceleration`
// Member `angular_velocity`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `sun_direction`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
rover_control__msg__RoverStatus__init(rover_control__msg__RoverStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rover_control__msg__RoverStatus__fini(msg);
    return false;
  }
  // current_action
  if (!rosidl_runtime_c__String__init(&msg->current_action)) {
    rover_control__msg__RoverStatus__fini(msg);
    return false;
  }
  // mission_mode
  if (!rosidl_runtime_c__String__init(&msg->mission_mode)) {
    rover_control__msg__RoverStatus__fini(msg);
    return false;
  }
  // is_autonomous
  // safety_ok
  // emergency_stop_active
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->current_pose)) {
    rover_control__msg__RoverStatus__fini(msg);
    return false;
  }
  // current_velocity
  if (!geometry_msgs__msg__Twist__init(&msg->current_velocity)) {
    rover_control__msg__RoverStatus__fini(msg);
    return false;
  }
  // odometry
  if (!nav_msgs__msg__Odometry__init(&msg->odometry)) {
    rover_control__msg__RoverStatus__fini(msg);
    return false;
  }
  // mission_progress_percentage
  // autonomy_time_remaining
  // waypoints_completed
  // total_waypoints
  // battery_voltage
  // battery_current
  // battery_soc
  // battery_temperature
  // motor_currents
  if (!rosidl_runtime_c__float__Sequence__init(&msg->motor_currents, 0)) {
    rover_control__msg__RoverStatus__fini(msg);
    return false;
  }
  // motor_temperatures
  if (!rosidl_runtime_c__float__Sequence__init(&msg->motor_temperatures, 0)) {
    rover_control__msg__RoverStatus__fini(msg);
    return false;
  }
  // communication_quality
  // bandwidth_usage
  // cpu_usage
  // memory_usage
  // active_faults
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->active_faults, 0)) {
    rover_control__msg__RoverStatus__fini(msg);
    return false;
  }
  // fault_messages
  if (!rosidl_runtime_c__String__Sequence__init(&msg->fault_messages, 0)) {
    rover_control__msg__RoverStatus__fini(msg);
    return false;
  }
  // stuck_detected
  // slip_ratio
  // collision_detected
  // ambient_temperature
  // internal_temperature
  // humidity
  // acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->acceleration)) {
    rover_control__msg__RoverStatus__fini(msg);
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->angular_velocity)) {
    rover_control__msg__RoverStatus__fini(msg);
    return false;
  }
  // competition_mode
  if (!rosidl_runtime_c__String__init(&msg->competition_mode)) {
    rover_control__msg__RoverStatus__fini(msg);
    return false;
  }
  // judge_score_estimate
  // visual_servoing_active
  // tennis_balls_detected
  // sun_direction
  if (!geometry_msgs__msg__Point__init(&msg->sun_direction)) {
    rover_control__msg__RoverStatus__fini(msg);
    return false;
  }
  return true;
}

void
rover_control__msg__RoverStatus__fini(rover_control__msg__RoverStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // current_action
  rosidl_runtime_c__String__fini(&msg->current_action);
  // mission_mode
  rosidl_runtime_c__String__fini(&msg->mission_mode);
  // is_autonomous
  // safety_ok
  // emergency_stop_active
  // current_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->current_pose);
  // current_velocity
  geometry_msgs__msg__Twist__fini(&msg->current_velocity);
  // odometry
  nav_msgs__msg__Odometry__fini(&msg->odometry);
  // mission_progress_percentage
  // autonomy_time_remaining
  // waypoints_completed
  // total_waypoints
  // battery_voltage
  // battery_current
  // battery_soc
  // battery_temperature
  // motor_currents
  rosidl_runtime_c__float__Sequence__fini(&msg->motor_currents);
  // motor_temperatures
  rosidl_runtime_c__float__Sequence__fini(&msg->motor_temperatures);
  // communication_quality
  // bandwidth_usage
  // cpu_usage
  // memory_usage
  // active_faults
  rosidl_runtime_c__int32__Sequence__fini(&msg->active_faults);
  // fault_messages
  rosidl_runtime_c__String__Sequence__fini(&msg->fault_messages);
  // stuck_detected
  // slip_ratio
  // collision_detected
  // ambient_temperature
  // internal_temperature
  // humidity
  // acceleration
  geometry_msgs__msg__Vector3__fini(&msg->acceleration);
  // angular_velocity
  geometry_msgs__msg__Vector3__fini(&msg->angular_velocity);
  // competition_mode
  rosidl_runtime_c__String__fini(&msg->competition_mode);
  // judge_score_estimate
  // visual_servoing_active
  // tennis_balls_detected
  // sun_direction
  geometry_msgs__msg__Point__fini(&msg->sun_direction);
}

bool
rover_control__msg__RoverStatus__are_equal(const rover_control__msg__RoverStatus * lhs, const rover_control__msg__RoverStatus * rhs)
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
  // current_action
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_action), &(rhs->current_action)))
  {
    return false;
  }
  // mission_mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission_mode), &(rhs->mission_mode)))
  {
    return false;
  }
  // is_autonomous
  if (lhs->is_autonomous != rhs->is_autonomous) {
    return false;
  }
  // safety_ok
  if (lhs->safety_ok != rhs->safety_ok) {
    return false;
  }
  // emergency_stop_active
  if (lhs->emergency_stop_active != rhs->emergency_stop_active) {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
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
  // odometry
  if (!nav_msgs__msg__Odometry__are_equal(
      &(lhs->odometry), &(rhs->odometry)))
  {
    return false;
  }
  // mission_progress_percentage
  if (lhs->mission_progress_percentage != rhs->mission_progress_percentage) {
    return false;
  }
  // autonomy_time_remaining
  if (lhs->autonomy_time_remaining != rhs->autonomy_time_remaining) {
    return false;
  }
  // waypoints_completed
  if (lhs->waypoints_completed != rhs->waypoints_completed) {
    return false;
  }
  // total_waypoints
  if (lhs->total_waypoints != rhs->total_waypoints) {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // battery_current
  if (lhs->battery_current != rhs->battery_current) {
    return false;
  }
  // battery_soc
  if (lhs->battery_soc != rhs->battery_soc) {
    return false;
  }
  // battery_temperature
  if (lhs->battery_temperature != rhs->battery_temperature) {
    return false;
  }
  // motor_currents
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->motor_currents), &(rhs->motor_currents)))
  {
    return false;
  }
  // motor_temperatures
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->motor_temperatures), &(rhs->motor_temperatures)))
  {
    return false;
  }
  // communication_quality
  if (lhs->communication_quality != rhs->communication_quality) {
    return false;
  }
  // bandwidth_usage
  if (lhs->bandwidth_usage != rhs->bandwidth_usage) {
    return false;
  }
  // cpu_usage
  if (lhs->cpu_usage != rhs->cpu_usage) {
    return false;
  }
  // memory_usage
  if (lhs->memory_usage != rhs->memory_usage) {
    return false;
  }
  // active_faults
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->active_faults), &(rhs->active_faults)))
  {
    return false;
  }
  // fault_messages
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->fault_messages), &(rhs->fault_messages)))
  {
    return false;
  }
  // stuck_detected
  if (lhs->stuck_detected != rhs->stuck_detected) {
    return false;
  }
  // slip_ratio
  if (lhs->slip_ratio != rhs->slip_ratio) {
    return false;
  }
  // collision_detected
  if (lhs->collision_detected != rhs->collision_detected) {
    return false;
  }
  // ambient_temperature
  if (lhs->ambient_temperature != rhs->ambient_temperature) {
    return false;
  }
  // internal_temperature
  if (lhs->internal_temperature != rhs->internal_temperature) {
    return false;
  }
  // humidity
  if (lhs->humidity != rhs->humidity) {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->acceleration), &(rhs->acceleration)))
  {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->angular_velocity), &(rhs->angular_velocity)))
  {
    return false;
  }
  // competition_mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->competition_mode), &(rhs->competition_mode)))
  {
    return false;
  }
  // judge_score_estimate
  if (lhs->judge_score_estimate != rhs->judge_score_estimate) {
    return false;
  }
  // visual_servoing_active
  if (lhs->visual_servoing_active != rhs->visual_servoing_active) {
    return false;
  }
  // tennis_balls_detected
  if (lhs->tennis_balls_detected != rhs->tennis_balls_detected) {
    return false;
  }
  // sun_direction
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->sun_direction), &(rhs->sun_direction)))
  {
    return false;
  }
  return true;
}

bool
rover_control__msg__RoverStatus__copy(
  const rover_control__msg__RoverStatus * input,
  rover_control__msg__RoverStatus * output)
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
  // current_action
  if (!rosidl_runtime_c__String__copy(
      &(input->current_action), &(output->current_action)))
  {
    return false;
  }
  // mission_mode
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_mode), &(output->mission_mode)))
  {
    return false;
  }
  // is_autonomous
  output->is_autonomous = input->is_autonomous;
  // safety_ok
  output->safety_ok = input->safety_ok;
  // emergency_stop_active
  output->emergency_stop_active = input->emergency_stop_active;
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
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
  // odometry
  if (!nav_msgs__msg__Odometry__copy(
      &(input->odometry), &(output->odometry)))
  {
    return false;
  }
  // mission_progress_percentage
  output->mission_progress_percentage = input->mission_progress_percentage;
  // autonomy_time_remaining
  output->autonomy_time_remaining = input->autonomy_time_remaining;
  // waypoints_completed
  output->waypoints_completed = input->waypoints_completed;
  // total_waypoints
  output->total_waypoints = input->total_waypoints;
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // battery_current
  output->battery_current = input->battery_current;
  // battery_soc
  output->battery_soc = input->battery_soc;
  // battery_temperature
  output->battery_temperature = input->battery_temperature;
  // motor_currents
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->motor_currents), &(output->motor_currents)))
  {
    return false;
  }
  // motor_temperatures
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->motor_temperatures), &(output->motor_temperatures)))
  {
    return false;
  }
  // communication_quality
  output->communication_quality = input->communication_quality;
  // bandwidth_usage
  output->bandwidth_usage = input->bandwidth_usage;
  // cpu_usage
  output->cpu_usage = input->cpu_usage;
  // memory_usage
  output->memory_usage = input->memory_usage;
  // active_faults
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->active_faults), &(output->active_faults)))
  {
    return false;
  }
  // fault_messages
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->fault_messages), &(output->fault_messages)))
  {
    return false;
  }
  // stuck_detected
  output->stuck_detected = input->stuck_detected;
  // slip_ratio
  output->slip_ratio = input->slip_ratio;
  // collision_detected
  output->collision_detected = input->collision_detected;
  // ambient_temperature
  output->ambient_temperature = input->ambient_temperature;
  // internal_temperature
  output->internal_temperature = input->internal_temperature;
  // humidity
  output->humidity = input->humidity;
  // acceleration
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->acceleration), &(output->acceleration)))
  {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->angular_velocity), &(output->angular_velocity)))
  {
    return false;
  }
  // competition_mode
  if (!rosidl_runtime_c__String__copy(
      &(input->competition_mode), &(output->competition_mode)))
  {
    return false;
  }
  // judge_score_estimate
  output->judge_score_estimate = input->judge_score_estimate;
  // visual_servoing_active
  output->visual_servoing_active = input->visual_servoing_active;
  // tennis_balls_detected
  output->tennis_balls_detected = input->tennis_balls_detected;
  // sun_direction
  if (!geometry_msgs__msg__Point__copy(
      &(input->sun_direction), &(output->sun_direction)))
  {
    return false;
  }
  return true;
}

rover_control__msg__RoverStatus *
rover_control__msg__RoverStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__msg__RoverStatus * msg = (rover_control__msg__RoverStatus *)allocator.allocate(sizeof(rover_control__msg__RoverStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_control__msg__RoverStatus));
  bool success = rover_control__msg__RoverStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_control__msg__RoverStatus__destroy(rover_control__msg__RoverStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_control__msg__RoverStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_control__msg__RoverStatus__Sequence__init(rover_control__msg__RoverStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__msg__RoverStatus * data = NULL;

  if (size) {
    data = (rover_control__msg__RoverStatus *)allocator.zero_allocate(size, sizeof(rover_control__msg__RoverStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_control__msg__RoverStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_control__msg__RoverStatus__fini(&data[i - 1]);
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
rover_control__msg__RoverStatus__Sequence__fini(rover_control__msg__RoverStatus__Sequence * array)
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
      rover_control__msg__RoverStatus__fini(&array->data[i]);
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

rover_control__msg__RoverStatus__Sequence *
rover_control__msg__RoverStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_control__msg__RoverStatus__Sequence * array = (rover_control__msg__RoverStatus__Sequence *)allocator.allocate(sizeof(rover_control__msg__RoverStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_control__msg__RoverStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_control__msg__RoverStatus__Sequence__destroy(rover_control__msg__RoverStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_control__msg__RoverStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_control__msg__RoverStatus__Sequence__are_equal(const rover_control__msg__RoverStatus__Sequence * lhs, const rover_control__msg__RoverStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_control__msg__RoverStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_control__msg__RoverStatus__Sequence__copy(
  const rover_control__msg__RoverStatus__Sequence * input,
  rover_control__msg__RoverStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_control__msg__RoverStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_control__msg__RoverStatus * data =
      (rover_control__msg__RoverStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_control__msg__RoverStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_control__msg__RoverStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_control__msg__RoverStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
