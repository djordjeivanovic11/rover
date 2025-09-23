// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arm_control:action/ToolChange.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__ACTION__DETAIL__TOOL_CHANGE__TRAITS_HPP_
#define ARM_CONTROL__ACTION__DETAIL__TOOL_CHANGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arm_control/action/detail/tool_change__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'tool_dock_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const ToolChange_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: new_tool_name
  {
    out << "new_tool_name: ";
    rosidl_generator_traits::value_to_yaml(msg.new_tool_name, out);
    out << ", ";
  }

  // member: current_tool_name
  {
    out << "current_tool_name: ";
    rosidl_generator_traits::value_to_yaml(msg.current_tool_name, out);
    out << ", ";
  }

  // member: auto_detect_current
  {
    out << "auto_detect_current: ";
    rosidl_generator_traits::value_to_yaml(msg.auto_detect_current, out);
    out << ", ";
  }

  // member: tool_dock_pose
  {
    out << "tool_dock_pose: ";
    to_flow_style_yaml(msg.tool_dock_pose, out);
    out << ", ";
  }

  // member: approach_distance
  {
    out << "approach_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.approach_distance, out);
    out << ", ";
  }

  // member: coupling_timeout
  {
    out << "coupling_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.coupling_timeout, out);
    out << ", ";
  }

  // member: verify_tool_change
  {
    out << "verify_tool_change: ";
    rosidl_generator_traits::value_to_yaml(msg.verify_tool_change, out);
    out << ", ";
  }

  // member: update_kinematics
  {
    out << "update_kinematics: ";
    rosidl_generator_traits::value_to_yaml(msg.update_kinematics, out);
    out << ", ";
  }

  // member: velocity_scaling
  {
    out << "velocity_scaling: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_scaling, out);
    out << ", ";
  }

  // member: change_strategy
  {
    out << "change_strategy: ";
    rosidl_generator_traits::value_to_yaml(msg.change_strategy, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ToolChange_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: new_tool_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "new_tool_name: ";
    rosidl_generator_traits::value_to_yaml(msg.new_tool_name, out);
    out << "\n";
  }

  // member: current_tool_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_tool_name: ";
    rosidl_generator_traits::value_to_yaml(msg.current_tool_name, out);
    out << "\n";
  }

  // member: auto_detect_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "auto_detect_current: ";
    rosidl_generator_traits::value_to_yaml(msg.auto_detect_current, out);
    out << "\n";
  }

  // member: tool_dock_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tool_dock_pose:\n";
    to_block_style_yaml(msg.tool_dock_pose, out, indentation + 2);
  }

  // member: approach_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "approach_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.approach_distance, out);
    out << "\n";
  }

  // member: coupling_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "coupling_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.coupling_timeout, out);
    out << "\n";
  }

  // member: verify_tool_change
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "verify_tool_change: ";
    rosidl_generator_traits::value_to_yaml(msg.verify_tool_change, out);
    out << "\n";
  }

  // member: update_kinematics
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "update_kinematics: ";
    rosidl_generator_traits::value_to_yaml(msg.update_kinematics, out);
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

  // member: change_strategy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "change_strategy: ";
    rosidl_generator_traits::value_to_yaml(msg.change_strategy, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ToolChange_Goal & msg, bool use_flow_style = false)
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
  const arm_control::action::ToolChange_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::ToolChange_Goal & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::ToolChange_Goal>()
{
  return "arm_control::action::ToolChange_Goal";
}

template<>
inline const char * name<arm_control::action::ToolChange_Goal>()
{
  return "arm_control/action/ToolChange_Goal";
}

template<>
struct has_fixed_size<arm_control::action::ToolChange_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arm_control::action::ToolChange_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arm_control::action::ToolChange_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'final_tool_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const ToolChange_Result & msg,
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

  // member: old_tool_name
  {
    out << "old_tool_name: ";
    rosidl_generator_traits::value_to_yaml(msg.old_tool_name, out);
    out << ", ";
  }

  // member: new_tool_name
  {
    out << "new_tool_name: ";
    rosidl_generator_traits::value_to_yaml(msg.new_tool_name, out);
    out << ", ";
  }

  // member: coupling_verified
  {
    out << "coupling_verified: ";
    rosidl_generator_traits::value_to_yaml(msg.coupling_verified, out);
    out << ", ";
  }

  // member: kinematics_updated
  {
    out << "kinematics_updated: ";
    rosidl_generator_traits::value_to_yaml(msg.kinematics_updated, out);
    out << ", ";
  }

  // member: final_tool_pose
  {
    out << "final_tool_pose: ";
    to_flow_style_yaml(msg.final_tool_pose, out);
    out << ", ";
  }

  // member: total_execution_time
  {
    out << "total_execution_time: ";
    rosidl_generator_traits::value_to_yaml(msg.total_execution_time, out);
    out << ", ";
  }

  // member: tool_configuration
  {
    out << "tool_configuration: ";
    rosidl_generator_traits::value_to_yaml(msg.tool_configuration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ToolChange_Result & msg,
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

  // member: old_tool_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "old_tool_name: ";
    rosidl_generator_traits::value_to_yaml(msg.old_tool_name, out);
    out << "\n";
  }

  // member: new_tool_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "new_tool_name: ";
    rosidl_generator_traits::value_to_yaml(msg.new_tool_name, out);
    out << "\n";
  }

  // member: coupling_verified
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "coupling_verified: ";
    rosidl_generator_traits::value_to_yaml(msg.coupling_verified, out);
    out << "\n";
  }

  // member: kinematics_updated
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kinematics_updated: ";
    rosidl_generator_traits::value_to_yaml(msg.kinematics_updated, out);
    out << "\n";
  }

  // member: final_tool_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_tool_pose:\n";
    to_block_style_yaml(msg.final_tool_pose, out, indentation + 2);
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

  // member: tool_configuration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tool_configuration: ";
    rosidl_generator_traits::value_to_yaml(msg.tool_configuration, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ToolChange_Result & msg, bool use_flow_style = false)
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
  const arm_control::action::ToolChange_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::ToolChange_Result & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::ToolChange_Result>()
{
  return "arm_control::action::ToolChange_Result";
}

template<>
inline const char * name<arm_control::action::ToolChange_Result>()
{
  return "arm_control/action/ToolChange_Result";
}

template<>
struct has_fixed_size<arm_control::action::ToolChange_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arm_control::action::ToolChange_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arm_control::action::ToolChange_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const ToolChange_Feedback & msg,
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

  // member: tool_detected
  {
    out << "tool_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.tool_detected, out);
    out << ", ";
  }

  // member: coupling_engaged
  {
    out << "coupling_engaged: ";
    rosidl_generator_traits::value_to_yaml(msg.coupling_engaged, out);
    out << ", ";
  }

  // member: coupling_status
  {
    out << "coupling_status: ";
    rosidl_generator_traits::value_to_yaml(msg.coupling_status, out);
    out << ", ";
  }

  // member: kinematics_status
  {
    out << "kinematics_status: ";
    rosidl_generator_traits::value_to_yaml(msg.kinematics_status, out);
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
  const ToolChange_Feedback & msg,
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

  // member: tool_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tool_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.tool_detected, out);
    out << "\n";
  }

  // member: coupling_engaged
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "coupling_engaged: ";
    rosidl_generator_traits::value_to_yaml(msg.coupling_engaged, out);
    out << "\n";
  }

  // member: coupling_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "coupling_status: ";
    rosidl_generator_traits::value_to_yaml(msg.coupling_status, out);
    out << "\n";
  }

  // member: kinematics_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kinematics_status: ";
    rosidl_generator_traits::value_to_yaml(msg.kinematics_status, out);
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

inline std::string to_yaml(const ToolChange_Feedback & msg, bool use_flow_style = false)
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
  const arm_control::action::ToolChange_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::ToolChange_Feedback & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::ToolChange_Feedback>()
{
  return "arm_control::action::ToolChange_Feedback";
}

template<>
inline const char * name<arm_control::action::ToolChange_Feedback>()
{
  return "arm_control/action/ToolChange_Feedback";
}

template<>
struct has_fixed_size<arm_control::action::ToolChange_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arm_control::action::ToolChange_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arm_control::action::ToolChange_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "arm_control/action/detail/tool_change__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const ToolChange_SendGoal_Request & msg,
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
  const ToolChange_SendGoal_Request & msg,
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

inline std::string to_yaml(const ToolChange_SendGoal_Request & msg, bool use_flow_style = false)
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
  const arm_control::action::ToolChange_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::ToolChange_SendGoal_Request & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::ToolChange_SendGoal_Request>()
{
  return "arm_control::action::ToolChange_SendGoal_Request";
}

template<>
inline const char * name<arm_control::action::ToolChange_SendGoal_Request>()
{
  return "arm_control/action/ToolChange_SendGoal_Request";
}

template<>
struct has_fixed_size<arm_control::action::ToolChange_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<arm_control::action::ToolChange_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<arm_control::action::ToolChange_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<arm_control::action::ToolChange_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<arm_control::action::ToolChange_SendGoal_Request>
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
  const ToolChange_SendGoal_Response & msg,
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
  const ToolChange_SendGoal_Response & msg,
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

inline std::string to_yaml(const ToolChange_SendGoal_Response & msg, bool use_flow_style = false)
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
  const arm_control::action::ToolChange_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::ToolChange_SendGoal_Response & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::ToolChange_SendGoal_Response>()
{
  return "arm_control::action::ToolChange_SendGoal_Response";
}

template<>
inline const char * name<arm_control::action::ToolChange_SendGoal_Response>()
{
  return "arm_control/action/ToolChange_SendGoal_Response";
}

template<>
struct has_fixed_size<arm_control::action::ToolChange_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<arm_control::action::ToolChange_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<arm_control::action::ToolChange_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<arm_control::action::ToolChange_SendGoal>()
{
  return "arm_control::action::ToolChange_SendGoal";
}

template<>
inline const char * name<arm_control::action::ToolChange_SendGoal>()
{
  return "arm_control/action/ToolChange_SendGoal";
}

template<>
struct has_fixed_size<arm_control::action::ToolChange_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<arm_control::action::ToolChange_SendGoal_Request>::value &&
    has_fixed_size<arm_control::action::ToolChange_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<arm_control::action::ToolChange_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<arm_control::action::ToolChange_SendGoal_Request>::value &&
    has_bounded_size<arm_control::action::ToolChange_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<arm_control::action::ToolChange_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<arm_control::action::ToolChange_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<arm_control::action::ToolChange_SendGoal_Response>
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
  const ToolChange_GetResult_Request & msg,
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
  const ToolChange_GetResult_Request & msg,
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

inline std::string to_yaml(const ToolChange_GetResult_Request & msg, bool use_flow_style = false)
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
  const arm_control::action::ToolChange_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::ToolChange_GetResult_Request & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::ToolChange_GetResult_Request>()
{
  return "arm_control::action::ToolChange_GetResult_Request";
}

template<>
inline const char * name<arm_control::action::ToolChange_GetResult_Request>()
{
  return "arm_control/action/ToolChange_GetResult_Request";
}

template<>
struct has_fixed_size<arm_control::action::ToolChange_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<arm_control::action::ToolChange_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<arm_control::action::ToolChange_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "arm_control/action/detail/tool_change__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const ToolChange_GetResult_Response & msg,
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
  const ToolChange_GetResult_Response & msg,
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

inline std::string to_yaml(const ToolChange_GetResult_Response & msg, bool use_flow_style = false)
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
  const arm_control::action::ToolChange_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::ToolChange_GetResult_Response & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::ToolChange_GetResult_Response>()
{
  return "arm_control::action::ToolChange_GetResult_Response";
}

template<>
inline const char * name<arm_control::action::ToolChange_GetResult_Response>()
{
  return "arm_control/action/ToolChange_GetResult_Response";
}

template<>
struct has_fixed_size<arm_control::action::ToolChange_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<arm_control::action::ToolChange_Result>::value> {};

template<>
struct has_bounded_size<arm_control::action::ToolChange_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<arm_control::action::ToolChange_Result>::value> {};

template<>
struct is_message<arm_control::action::ToolChange_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<arm_control::action::ToolChange_GetResult>()
{
  return "arm_control::action::ToolChange_GetResult";
}

template<>
inline const char * name<arm_control::action::ToolChange_GetResult>()
{
  return "arm_control/action/ToolChange_GetResult";
}

template<>
struct has_fixed_size<arm_control::action::ToolChange_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<arm_control::action::ToolChange_GetResult_Request>::value &&
    has_fixed_size<arm_control::action::ToolChange_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<arm_control::action::ToolChange_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<arm_control::action::ToolChange_GetResult_Request>::value &&
    has_bounded_size<arm_control::action::ToolChange_GetResult_Response>::value
  >
{
};

template<>
struct is_service<arm_control::action::ToolChange_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<arm_control::action::ToolChange_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<arm_control::action::ToolChange_GetResult_Response>
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
// #include "arm_control/action/detail/tool_change__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const ToolChange_FeedbackMessage & msg,
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
  const ToolChange_FeedbackMessage & msg,
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

inline std::string to_yaml(const ToolChange_FeedbackMessage & msg, bool use_flow_style = false)
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
  const arm_control::action::ToolChange_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::ToolChange_FeedbackMessage & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::ToolChange_FeedbackMessage>()
{
  return "arm_control::action::ToolChange_FeedbackMessage";
}

template<>
inline const char * name<arm_control::action::ToolChange_FeedbackMessage>()
{
  return "arm_control/action/ToolChange_FeedbackMessage";
}

template<>
struct has_fixed_size<arm_control::action::ToolChange_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<arm_control::action::ToolChange_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<arm_control::action::ToolChange_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<arm_control::action::ToolChange_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<arm_control::action::ToolChange_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<arm_control::action::ToolChange>
  : std::true_type
{
};

template<>
struct is_action_goal<arm_control::action::ToolChange_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<arm_control::action::ToolChange_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<arm_control::action::ToolChange_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ARM_CONTROL__ACTION__DETAIL__TOOL_CHANGE__TRAITS_HPP_
