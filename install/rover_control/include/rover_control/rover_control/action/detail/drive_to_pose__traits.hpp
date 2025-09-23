// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_control:action/DriveToPose.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__ACTION__DETAIL__DRIVE_TO_POSE__TRAITS_HPP_
#define ROVER_CONTROL__ACTION__DETAIL__DRIVE_TO_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_control/action/detail/drive_to_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const DriveToPose_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_pose
  {
    out << "target_pose: ";
    to_flow_style_yaml(msg.target_pose, out);
    out << ", ";
  }

  // member: max_velocity
  {
    out << "max_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.max_velocity, out);
    out << ", ";
  }

  // member: position_tolerance
  {
    out << "position_tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.position_tolerance, out);
    out << ", ";
  }

  // member: orientation_tolerance
  {
    out << "orientation_tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.orientation_tolerance, out);
    out << ", ";
  }

  // member: timeout
  {
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << ", ";
  }

  // member: use_visual_servoing
  {
    out << "use_visual_servoing: ";
    rosidl_generator_traits::value_to_yaml(msg.use_visual_servoing, out);
    out << ", ";
  }

  // member: mission_mode
  {
    out << "mission_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_mode, out);
    out << ", ";
  }

  // member: reverse_allowed
  {
    out << "reverse_allowed: ";
    rosidl_generator_traits::value_to_yaml(msg.reverse_allowed, out);
    out << ", ";
  }

  // member: approach_velocity
  {
    out << "approach_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.approach_velocity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DriveToPose_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_pose:\n";
    to_block_style_yaml(msg.target_pose, out, indentation + 2);
  }

  // member: max_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.max_velocity, out);
    out << "\n";
  }

  // member: position_tolerance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.position_tolerance, out);
    out << "\n";
  }

  // member: orientation_tolerance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation_tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.orientation_tolerance, out);
    out << "\n";
  }

  // member: timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << "\n";
  }

  // member: use_visual_servoing
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_visual_servoing: ";
    rosidl_generator_traits::value_to_yaml(msg.use_visual_servoing, out);
    out << "\n";
  }

  // member: mission_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_mode, out);
    out << "\n";
  }

  // member: reverse_allowed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reverse_allowed: ";
    rosidl_generator_traits::value_to_yaml(msg.reverse_allowed, out);
    out << "\n";
  }

  // member: approach_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "approach_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.approach_velocity, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DriveToPose_Goal & msg, bool use_flow_style = false)
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

}  // namespace rover_control

namespace rosidl_generator_traits
{

[[deprecated("use rover_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_control::action::DriveToPose_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::DriveToPose_Goal & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::DriveToPose_Goal>()
{
  return "rover_control::action::DriveToPose_Goal";
}

template<>
inline const char * name<rover_control::action::DriveToPose_Goal>()
{
  return "rover_control/action/DriveToPose_Goal";
}

template<>
struct has_fixed_size<rover_control::action::DriveToPose_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_control::action::DriveToPose_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_control::action::DriveToPose_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'final_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'executed_path'
#include "nav_msgs/msg/detail/path__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const DriveToPose_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: result_code
  {
    out << "result_code: ";
    rosidl_generator_traits::value_to_yaml(msg.result_code, out);
    out << ", ";
  }

  // member: error_message
  {
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << ", ";
  }

  // member: final_position_error
  {
    out << "final_position_error: ";
    rosidl_generator_traits::value_to_yaml(msg.final_position_error, out);
    out << ", ";
  }

  // member: final_orientation_error
  {
    out << "final_orientation_error: ";
    rosidl_generator_traits::value_to_yaml(msg.final_orientation_error, out);
    out << ", ";
  }

  // member: execution_time
  {
    out << "execution_time: ";
    rosidl_generator_traits::value_to_yaml(msg.execution_time, out);
    out << ", ";
  }

  // member: distance_traveled
  {
    out << "distance_traveled: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_traveled, out);
    out << ", ";
  }

  // member: final_pose
  {
    out << "final_pose: ";
    to_flow_style_yaml(msg.final_pose, out);
    out << ", ";
  }

  // member: executed_path
  {
    out << "executed_path: ";
    to_flow_style_yaml(msg.executed_path, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DriveToPose_Result & msg,
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

  // member: result_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result_code: ";
    rosidl_generator_traits::value_to_yaml(msg.result_code, out);
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

  // member: final_position_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_position_error: ";
    rosidl_generator_traits::value_to_yaml(msg.final_position_error, out);
    out << "\n";
  }

  // member: final_orientation_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_orientation_error: ";
    rosidl_generator_traits::value_to_yaml(msg.final_orientation_error, out);
    out << "\n";
  }

  // member: execution_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "execution_time: ";
    rosidl_generator_traits::value_to_yaml(msg.execution_time, out);
    out << "\n";
  }

  // member: distance_traveled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_traveled: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_traveled, out);
    out << "\n";
  }

  // member: final_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_pose:\n";
    to_block_style_yaml(msg.final_pose, out, indentation + 2);
  }

  // member: executed_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "executed_path:\n";
    to_block_style_yaml(msg.executed_path, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DriveToPose_Result & msg, bool use_flow_style = false)
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

}  // namespace rover_control

namespace rosidl_generator_traits
{

[[deprecated("use rover_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_control::action::DriveToPose_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::DriveToPose_Result & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::DriveToPose_Result>()
{
  return "rover_control::action::DriveToPose_Result";
}

template<>
inline const char * name<rover_control::action::DriveToPose_Result>()
{
  return "rover_control/action/DriveToPose_Result";
}

template<>
struct has_fixed_size<rover_control::action::DriveToPose_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_control::action::DriveToPose_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_control::action::DriveToPose_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const DriveToPose_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_state
  {
    out << "current_state: ";
    rosidl_generator_traits::value_to_yaml(msg.current_state, out);
    out << ", ";
  }

  // member: progress_percentage
  {
    out << "progress_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_percentage, out);
    out << ", ";
  }

  // member: current_pose
  {
    out << "current_pose: ";
    to_flow_style_yaml(msg.current_pose, out);
    out << ", ";
  }

  // member: estimated_time_remaining
  {
    out << "estimated_time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_time_remaining, out);
    out << ", ";
  }

  // member: distance_remaining
  {
    out << "distance_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_remaining, out);
    out << ", ";
  }

  // member: current_velocity
  {
    out << "current_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.current_velocity, out);
    out << ", ";
  }

  // member: status_message
  {
    out << "status_message: ";
    rosidl_generator_traits::value_to_yaml(msg.status_message, out);
    out << ", ";
  }

  // member: stuck_detected
  {
    out << "stuck_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.stuck_detected, out);
    out << ", ";
  }

  // member: autonomy_time_remaining
  {
    out << "autonomy_time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.autonomy_time_remaining, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DriveToPose_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_state: ";
    rosidl_generator_traits::value_to_yaml(msg.current_state, out);
    out << "\n";
  }

  // member: progress_percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "progress_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_percentage, out);
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

  // member: estimated_time_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estimated_time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_time_remaining, out);
    out << "\n";
  }

  // member: distance_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_remaining, out);
    out << "\n";
  }

  // member: current_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.current_velocity, out);
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

  // member: stuck_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stuck_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.stuck_detected, out);
    out << "\n";
  }

  // member: autonomy_time_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "autonomy_time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.autonomy_time_remaining, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DriveToPose_Feedback & msg, bool use_flow_style = false)
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

}  // namespace rover_control

namespace rosidl_generator_traits
{

[[deprecated("use rover_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_control::action::DriveToPose_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::DriveToPose_Feedback & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::DriveToPose_Feedback>()
{
  return "rover_control::action::DriveToPose_Feedback";
}

template<>
inline const char * name<rover_control::action::DriveToPose_Feedback>()
{
  return "rover_control/action/DriveToPose_Feedback";
}

template<>
struct has_fixed_size<rover_control::action::DriveToPose_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_control::action::DriveToPose_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_control::action::DriveToPose_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "rover_control/action/detail/drive_to_pose__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const DriveToPose_SendGoal_Request & msg,
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
  const DriveToPose_SendGoal_Request & msg,
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

inline std::string to_yaml(const DriveToPose_SendGoal_Request & msg, bool use_flow_style = false)
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

}  // namespace rover_control

namespace rosidl_generator_traits
{

[[deprecated("use rover_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_control::action::DriveToPose_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::DriveToPose_SendGoal_Request & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::DriveToPose_SendGoal_Request>()
{
  return "rover_control::action::DriveToPose_SendGoal_Request";
}

template<>
inline const char * name<rover_control::action::DriveToPose_SendGoal_Request>()
{
  return "rover_control/action/DriveToPose_SendGoal_Request";
}

template<>
struct has_fixed_size<rover_control::action::DriveToPose_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<rover_control::action::DriveToPose_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rover_control::action::DriveToPose_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<rover_control::action::DriveToPose_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rover_control::action::DriveToPose_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const DriveToPose_SendGoal_Response & msg,
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
  const DriveToPose_SendGoal_Response & msg,
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

inline std::string to_yaml(const DriveToPose_SendGoal_Response & msg, bool use_flow_style = false)
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

}  // namespace rover_control

namespace rosidl_generator_traits
{

[[deprecated("use rover_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_control::action::DriveToPose_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::DriveToPose_SendGoal_Response & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::DriveToPose_SendGoal_Response>()
{
  return "rover_control::action::DriveToPose_SendGoal_Response";
}

template<>
inline const char * name<rover_control::action::DriveToPose_SendGoal_Response>()
{
  return "rover_control/action/DriveToPose_SendGoal_Response";
}

template<>
struct has_fixed_size<rover_control::action::DriveToPose_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<rover_control::action::DriveToPose_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<rover_control::action::DriveToPose_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rover_control::action::DriveToPose_SendGoal>()
{
  return "rover_control::action::DriveToPose_SendGoal";
}

template<>
inline const char * name<rover_control::action::DriveToPose_SendGoal>()
{
  return "rover_control/action/DriveToPose_SendGoal";
}

template<>
struct has_fixed_size<rover_control::action::DriveToPose_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<rover_control::action::DriveToPose_SendGoal_Request>::value &&
    has_fixed_size<rover_control::action::DriveToPose_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<rover_control::action::DriveToPose_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<rover_control::action::DriveToPose_SendGoal_Request>::value &&
    has_bounded_size<rover_control::action::DriveToPose_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<rover_control::action::DriveToPose_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<rover_control::action::DriveToPose_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rover_control::action::DriveToPose_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const DriveToPose_GetResult_Request & msg,
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
  const DriveToPose_GetResult_Request & msg,
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

inline std::string to_yaml(const DriveToPose_GetResult_Request & msg, bool use_flow_style = false)
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

}  // namespace rover_control

namespace rosidl_generator_traits
{

[[deprecated("use rover_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_control::action::DriveToPose_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::DriveToPose_GetResult_Request & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::DriveToPose_GetResult_Request>()
{
  return "rover_control::action::DriveToPose_GetResult_Request";
}

template<>
inline const char * name<rover_control::action::DriveToPose_GetResult_Request>()
{
  return "rover_control/action/DriveToPose_GetResult_Request";
}

template<>
struct has_fixed_size<rover_control::action::DriveToPose_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rover_control::action::DriveToPose_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rover_control::action::DriveToPose_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "rover_control/action/detail/drive_to_pose__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const DriveToPose_GetResult_Response & msg,
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
  const DriveToPose_GetResult_Response & msg,
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

inline std::string to_yaml(const DriveToPose_GetResult_Response & msg, bool use_flow_style = false)
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

}  // namespace rover_control

namespace rosidl_generator_traits
{

[[deprecated("use rover_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_control::action::DriveToPose_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::DriveToPose_GetResult_Response & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::DriveToPose_GetResult_Response>()
{
  return "rover_control::action::DriveToPose_GetResult_Response";
}

template<>
inline const char * name<rover_control::action::DriveToPose_GetResult_Response>()
{
  return "rover_control/action/DriveToPose_GetResult_Response";
}

template<>
struct has_fixed_size<rover_control::action::DriveToPose_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<rover_control::action::DriveToPose_Result>::value> {};

template<>
struct has_bounded_size<rover_control::action::DriveToPose_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<rover_control::action::DriveToPose_Result>::value> {};

template<>
struct is_message<rover_control::action::DriveToPose_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rover_control::action::DriveToPose_GetResult>()
{
  return "rover_control::action::DriveToPose_GetResult";
}

template<>
inline const char * name<rover_control::action::DriveToPose_GetResult>()
{
  return "rover_control/action/DriveToPose_GetResult";
}

template<>
struct has_fixed_size<rover_control::action::DriveToPose_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<rover_control::action::DriveToPose_GetResult_Request>::value &&
    has_fixed_size<rover_control::action::DriveToPose_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<rover_control::action::DriveToPose_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<rover_control::action::DriveToPose_GetResult_Request>::value &&
    has_bounded_size<rover_control::action::DriveToPose_GetResult_Response>::value
  >
{
};

template<>
struct is_service<rover_control::action::DriveToPose_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<rover_control::action::DriveToPose_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rover_control::action::DriveToPose_GetResult_Response>
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
// #include "rover_control/action/detail/drive_to_pose__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const DriveToPose_FeedbackMessage & msg,
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
  const DriveToPose_FeedbackMessage & msg,
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

inline std::string to_yaml(const DriveToPose_FeedbackMessage & msg, bool use_flow_style = false)
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

}  // namespace rover_control

namespace rosidl_generator_traits
{

[[deprecated("use rover_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_control::action::DriveToPose_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::DriveToPose_FeedbackMessage & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::DriveToPose_FeedbackMessage>()
{
  return "rover_control::action::DriveToPose_FeedbackMessage";
}

template<>
inline const char * name<rover_control::action::DriveToPose_FeedbackMessage>()
{
  return "rover_control/action/DriveToPose_FeedbackMessage";
}

template<>
struct has_fixed_size<rover_control::action::DriveToPose_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<rover_control::action::DriveToPose_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rover_control::action::DriveToPose_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<rover_control::action::DriveToPose_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rover_control::action::DriveToPose_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<rover_control::action::DriveToPose>
  : std::true_type
{
};

template<>
struct is_action_goal<rover_control::action::DriveToPose_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<rover_control::action::DriveToPose_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<rover_control::action::DriveToPose_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ROVER_CONTROL__ACTION__DETAIL__DRIVE_TO_POSE__TRAITS_HPP_
