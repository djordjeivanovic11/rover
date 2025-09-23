// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arm_control:action/PickAndPlace.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__ACTION__DETAIL__PICK_AND_PLACE__TRAITS_HPP_
#define ARM_CONTROL__ACTION__DETAIL__PICK_AND_PLACE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arm_control/action/detail/pick_and_place__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pick_pose'
// Member 'place_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'approach_offset'
// Member 'retreat_offset'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const PickAndPlace_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: pick_pose
  {
    out << "pick_pose: ";
    to_flow_style_yaml(msg.pick_pose, out);
    out << ", ";
  }

  // member: place_pose
  {
    out << "place_pose: ";
    to_flow_style_yaml(msg.place_pose, out);
    out << ", ";
  }

  // member: approach_offset
  {
    out << "approach_offset: ";
    to_flow_style_yaml(msg.approach_offset, out);
    out << ", ";
  }

  // member: retreat_offset
  {
    out << "retreat_offset: ";
    to_flow_style_yaml(msg.retreat_offset, out);
    out << ", ";
  }

  // member: grasp_force
  {
    out << "grasp_force: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_force, out);
    out << ", ";
  }

  // member: grasp_timeout
  {
    out << "grasp_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_timeout, out);
    out << ", ";
  }

  // member: grasp_strategy
  {
    out << "grasp_strategy: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_strategy, out);
    out << ", ";
  }

  // member: verify_grasp
  {
    out << "verify_grasp: ";
    rosidl_generator_traits::value_to_yaml(msg.verify_grasp, out);
    out << ", ";
  }

  // member: lift_height
  {
    out << "lift_height: ";
    rosidl_generator_traits::value_to_yaml(msg.lift_height, out);
    out << ", ";
  }

  // member: place_force
  {
    out << "place_force: ";
    rosidl_generator_traits::value_to_yaml(msg.place_force, out);
    out << ", ";
  }

  // member: gentle_place
  {
    out << "gentle_place: ";
    rosidl_generator_traits::value_to_yaml(msg.gentle_place, out);
    out << ", ";
  }

  // member: velocity_scaling
  {
    out << "velocity_scaling: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_scaling, out);
    out << ", ";
  }

  // member: acceleration_scaling
  {
    out << "acceleration_scaling: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration_scaling, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickAndPlace_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pick_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pick_pose:\n";
    to_block_style_yaml(msg.pick_pose, out, indentation + 2);
  }

  // member: place_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "place_pose:\n";
    to_block_style_yaml(msg.place_pose, out, indentation + 2);
  }

  // member: approach_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "approach_offset:\n";
    to_block_style_yaml(msg.approach_offset, out, indentation + 2);
  }

  // member: retreat_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "retreat_offset:\n";
    to_block_style_yaml(msg.retreat_offset, out, indentation + 2);
  }

  // member: grasp_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grasp_force: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_force, out);
    out << "\n";
  }

  // member: grasp_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grasp_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_timeout, out);
    out << "\n";
  }

  // member: grasp_strategy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grasp_strategy: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_strategy, out);
    out << "\n";
  }

  // member: verify_grasp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "verify_grasp: ";
    rosidl_generator_traits::value_to_yaml(msg.verify_grasp, out);
    out << "\n";
  }

  // member: lift_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lift_height: ";
    rosidl_generator_traits::value_to_yaml(msg.lift_height, out);
    out << "\n";
  }

  // member: place_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "place_force: ";
    rosidl_generator_traits::value_to_yaml(msg.place_force, out);
    out << "\n";
  }

  // member: gentle_place
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gentle_place: ";
    rosidl_generator_traits::value_to_yaml(msg.gentle_place, out);
    out << "\n";
  }

  // member: velocity_scaling
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_scaling: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_scaling, out);
    out << "\n";
  }

  // member: acceleration_scaling
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration_scaling: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration_scaling, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickAndPlace_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_control

namespace rosidl_generator_traits
{

[[deprecated("use arm_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_control::action::PickAndPlace_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::PickAndPlace_Goal & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::PickAndPlace_Goal>()
{
  return "arm_control::action::PickAndPlace_Goal";
}

template<>
inline const char * name<arm_control::action::PickAndPlace_Goal>()
{
  return "arm_control/action/PickAndPlace_Goal";
}

template<>
struct has_fixed_size<arm_control::action::PickAndPlace_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arm_control::action::PickAndPlace_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arm_control::action::PickAndPlace_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'final_object_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'final_joint_state'
#include "sensor_msgs/msg/detail/joint_state__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const PickAndPlace_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: error_message
  {
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << ", ";
  }

  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << ", ";
  }

  // member: failure_phase
  {
    out << "failure_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.failure_phase, out);
    out << ", ";
  }

  // member: grasp_successful
  {
    out << "grasp_successful: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_successful, out);
    out << ", ";
  }

  // member: place_successful
  {
    out << "place_successful: ";
    rosidl_generator_traits::value_to_yaml(msg.place_successful, out);
    out << ", ";
  }

  // member: grasp_force_achieved
  {
    out << "grasp_force_achieved: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_force_achieved, out);
    out << ", ";
  }

  // member: final_object_pose
  {
    out << "final_object_pose: ";
    to_flow_style_yaml(msg.final_object_pose, out);
    out << ", ";
  }

  // member: total_execution_time
  {
    out << "total_execution_time: ";
    rosidl_generator_traits::value_to_yaml(msg.total_execution_time, out);
    out << ", ";
  }

  // member: final_joint_state
  {
    out << "final_joint_state: ";
    to_flow_style_yaml(msg.final_joint_state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickAndPlace_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: error_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << "\n";
  }

  // member: error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << "\n";
  }

  // member: failure_phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "failure_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.failure_phase, out);
    out << "\n";
  }

  // member: grasp_successful
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grasp_successful: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_successful, out);
    out << "\n";
  }

  // member: place_successful
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "place_successful: ";
    rosidl_generator_traits::value_to_yaml(msg.place_successful, out);
    out << "\n";
  }

  // member: grasp_force_achieved
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grasp_force_achieved: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_force_achieved, out);
    out << "\n";
  }

  // member: final_object_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_object_pose:\n";
    to_block_style_yaml(msg.final_object_pose, out, indentation + 2);
  }

  // member: total_execution_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_execution_time: ";
    rosidl_generator_traits::value_to_yaml(msg.total_execution_time, out);
    out << "\n";
  }

  // member: final_joint_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_joint_state:\n";
    to_block_style_yaml(msg.final_joint_state, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickAndPlace_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_control

namespace rosidl_generator_traits
{

[[deprecated("use arm_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_control::action::PickAndPlace_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::PickAndPlace_Result & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::PickAndPlace_Result>()
{
  return "arm_control::action::PickAndPlace_Result";
}

template<>
inline const char * name<arm_control::action::PickAndPlace_Result>()
{
  return "arm_control/action/PickAndPlace_Result";
}

template<>
struct has_fixed_size<arm_control::action::PickAndPlace_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arm_control::action::PickAndPlace_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arm_control::action::PickAndPlace_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const PickAndPlace_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_phase
  {
    out << "current_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.current_phase, out);
    out << ", ";
  }

  // member: phase_progress
  {
    out << "phase_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.phase_progress, out);
    out << ", ";
  }

  // member: overall_progress
  {
    out << "overall_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.overall_progress, out);
    out << ", ";
  }

  // member: current_pose
  {
    out << "current_pose: ";
    to_flow_style_yaml(msg.current_pose, out);
    out << ", ";
  }

  // member: current_grasp_force
  {
    out << "current_grasp_force: ";
    rosidl_generator_traits::value_to_yaml(msg.current_grasp_force, out);
    out << ", ";
  }

  // member: object_detected
  {
    out << "object_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.object_detected, out);
    out << ", ";
  }

  // member: grasp_status
  {
    out << "grasp_status: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_status, out);
    out << ", ";
  }

  // member: status_message
  {
    out << "status_message: ";
    rosidl_generator_traits::value_to_yaml(msg.status_message, out);
    out << ", ";
  }

  // member: estimated_time_remaining
  {
    out << "estimated_time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_time_remaining, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickAndPlace_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_phase: ";
    rosidl_generator_traits::value_to_yaml(msg.current_phase, out);
    out << "\n";
  }

  // member: phase_progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "phase_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.phase_progress, out);
    out << "\n";
  }

  // member: overall_progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "overall_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.overall_progress, out);
    out << "\n";
  }

  // member: current_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_pose:\n";
    to_block_style_yaml(msg.current_pose, out, indentation + 2);
  }

  // member: current_grasp_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_grasp_force: ";
    rosidl_generator_traits::value_to_yaml(msg.current_grasp_force, out);
    out << "\n";
  }

  // member: object_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "object_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.object_detected, out);
    out << "\n";
  }

  // member: grasp_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grasp_status: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_status, out);
    out << "\n";
  }

  // member: status_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status_message: ";
    rosidl_generator_traits::value_to_yaml(msg.status_message, out);
    out << "\n";
  }

  // member: estimated_time_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estimated_time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_time_remaining, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickAndPlace_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_control

namespace rosidl_generator_traits
{

[[deprecated("use arm_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_control::action::PickAndPlace_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::PickAndPlace_Feedback & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::PickAndPlace_Feedback>()
{
  return "arm_control::action::PickAndPlace_Feedback";
}

template<>
inline const char * name<arm_control::action::PickAndPlace_Feedback>()
{
  return "arm_control/action/PickAndPlace_Feedback";
}

template<>
struct has_fixed_size<arm_control::action::PickAndPlace_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arm_control::action::PickAndPlace_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arm_control::action::PickAndPlace_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "arm_control/action/detail/pick_and_place__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const PickAndPlace_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickAndPlace_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickAndPlace_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_control

namespace rosidl_generator_traits
{

[[deprecated("use arm_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_control::action::PickAndPlace_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::PickAndPlace_SendGoal_Request & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::PickAndPlace_SendGoal_Request>()
{
  return "arm_control::action::PickAndPlace_SendGoal_Request";
}

template<>
inline const char * name<arm_control::action::PickAndPlace_SendGoal_Request>()
{
  return "arm_control/action/PickAndPlace_SendGoal_Request";
}

template<>
struct has_fixed_size<arm_control::action::PickAndPlace_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<arm_control::action::PickAndPlace_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<arm_control::action::PickAndPlace_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<arm_control::action::PickAndPlace_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<arm_control::action::PickAndPlace_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const PickAndPlace_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickAndPlace_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickAndPlace_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_control

namespace rosidl_generator_traits
{

[[deprecated("use arm_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_control::action::PickAndPlace_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::PickAndPlace_SendGoal_Response & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::PickAndPlace_SendGoal_Response>()
{
  return "arm_control::action::PickAndPlace_SendGoal_Response";
}

template<>
inline const char * name<arm_control::action::PickAndPlace_SendGoal_Response>()
{
  return "arm_control/action/PickAndPlace_SendGoal_Response";
}

template<>
struct has_fixed_size<arm_control::action::PickAndPlace_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<arm_control::action::PickAndPlace_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<arm_control::action::PickAndPlace_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<arm_control::action::PickAndPlace_SendGoal>()
{
  return "arm_control::action::PickAndPlace_SendGoal";
}

template<>
inline const char * name<arm_control::action::PickAndPlace_SendGoal>()
{
  return "arm_control/action/PickAndPlace_SendGoal";
}

template<>
struct has_fixed_size<arm_control::action::PickAndPlace_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<arm_control::action::PickAndPlace_SendGoal_Request>::value &&
    has_fixed_size<arm_control::action::PickAndPlace_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<arm_control::action::PickAndPlace_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<arm_control::action::PickAndPlace_SendGoal_Request>::value &&
    has_bounded_size<arm_control::action::PickAndPlace_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<arm_control::action::PickAndPlace_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<arm_control::action::PickAndPlace_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<arm_control::action::PickAndPlace_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const PickAndPlace_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickAndPlace_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickAndPlace_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_control

namespace rosidl_generator_traits
{

[[deprecated("use arm_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_control::action::PickAndPlace_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::PickAndPlace_GetResult_Request & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::PickAndPlace_GetResult_Request>()
{
  return "arm_control::action::PickAndPlace_GetResult_Request";
}

template<>
inline const char * name<arm_control::action::PickAndPlace_GetResult_Request>()
{
  return "arm_control/action/PickAndPlace_GetResult_Request";
}

template<>
struct has_fixed_size<arm_control::action::PickAndPlace_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<arm_control::action::PickAndPlace_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<arm_control::action::PickAndPlace_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "arm_control/action/detail/pick_and_place__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const PickAndPlace_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickAndPlace_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickAndPlace_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_control

namespace rosidl_generator_traits
{

[[deprecated("use arm_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_control::action::PickAndPlace_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::PickAndPlace_GetResult_Response & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::PickAndPlace_GetResult_Response>()
{
  return "arm_control::action::PickAndPlace_GetResult_Response";
}

template<>
inline const char * name<arm_control::action::PickAndPlace_GetResult_Response>()
{
  return "arm_control/action/PickAndPlace_GetResult_Response";
}

template<>
struct has_fixed_size<arm_control::action::PickAndPlace_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<arm_control::action::PickAndPlace_Result>::value> {};

template<>
struct has_bounded_size<arm_control::action::PickAndPlace_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<arm_control::action::PickAndPlace_Result>::value> {};

template<>
struct is_message<arm_control::action::PickAndPlace_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<arm_control::action::PickAndPlace_GetResult>()
{
  return "arm_control::action::PickAndPlace_GetResult";
}

template<>
inline const char * name<arm_control::action::PickAndPlace_GetResult>()
{
  return "arm_control/action/PickAndPlace_GetResult";
}

template<>
struct has_fixed_size<arm_control::action::PickAndPlace_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<arm_control::action::PickAndPlace_GetResult_Request>::value &&
    has_fixed_size<arm_control::action::PickAndPlace_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<arm_control::action::PickAndPlace_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<arm_control::action::PickAndPlace_GetResult_Request>::value &&
    has_bounded_size<arm_control::action::PickAndPlace_GetResult_Response>::value
  >
{
};

template<>
struct is_service<arm_control::action::PickAndPlace_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<arm_control::action::PickAndPlace_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<arm_control::action::PickAndPlace_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "arm_control/action/detail/pick_and_place__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const PickAndPlace_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickAndPlace_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickAndPlace_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace arm_control

namespace rosidl_generator_traits
{

[[deprecated("use arm_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_control::action::PickAndPlace_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::PickAndPlace_FeedbackMessage & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::PickAndPlace_FeedbackMessage>()
{
  return "arm_control::action::PickAndPlace_FeedbackMessage";
}

template<>
inline const char * name<arm_control::action::PickAndPlace_FeedbackMessage>()
{
  return "arm_control/action/PickAndPlace_FeedbackMessage";
}

template<>
struct has_fixed_size<arm_control::action::PickAndPlace_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<arm_control::action::PickAndPlace_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<arm_control::action::PickAndPlace_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<arm_control::action::PickAndPlace_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<arm_control::action::PickAndPlace_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<arm_control::action::PickAndPlace>
  : std::true_type
{
};

template<>
struct is_action_goal<arm_control::action::PickAndPlace_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<arm_control::action::PickAndPlace_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<arm_control::action::PickAndPlace_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ARM_CONTROL__ACTION__DETAIL__PICK_AND_PLACE__TRAITS_HPP_
