// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_control:action/CancelMotion.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__ACTION__DETAIL__CANCEL_MOTION__TRAITS_HPP_
#define ROVER_CONTROL__ACTION__DETAIL__CANCEL_MOTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_control/action/detail/cancel_motion__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const CancelMotion_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: emergency_stop
  {
    out << "emergency_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop, out);
    out << ", ";
  }

  // member: deceleration_time
  {
    out << "deceleration_time: ";
    rosidl_generator_traits::value_to_yaml(msg.deceleration_time, out);
    out << ", ";
  }

  // member: cancel_all_actions
  {
    out << "cancel_all_actions: ";
    rosidl_generator_traits::value_to_yaml(msg.cancel_all_actions, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CancelMotion_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: emergency_stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "emergency_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop, out);
    out << "\n";
  }

  // member: deceleration_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "deceleration_time: ";
    rosidl_generator_traits::value_to_yaml(msg.deceleration_time, out);
    out << "\n";
  }

  // member: cancel_all_actions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cancel_all_actions: ";
    rosidl_generator_traits::value_to_yaml(msg.cancel_all_actions, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CancelMotion_Goal & msg, bool use_flow_style = false)
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
  const rover_control::action::CancelMotion_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::CancelMotion_Goal & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::CancelMotion_Goal>()
{
  return "rover_control::action::CancelMotion_Goal";
}

template<>
inline const char * name<rover_control::action::CancelMotion_Goal>()
{
  return "rover_control/action/CancelMotion_Goal";
}

template<>
struct has_fixed_size<rover_control::action::CancelMotion_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_control::action::CancelMotion_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_control::action::CancelMotion_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'velocity_at_stop'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'final_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const CancelMotion_Result & msg,
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

  // member: stop_time
  {
    out << "stop_time: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_time, out);
    out << ", ";
  }

  // member: velocity_at_stop
  {
    out << "velocity_at_stop: ";
    to_flow_style_yaml(msg.velocity_at_stop, out);
    out << ", ";
  }

  // member: final_pose
  {
    out << "final_pose: ";
    to_flow_style_yaml(msg.final_pose, out);
    out << ", ";
  }

  // member: actions_cancelled
  {
    out << "actions_cancelled: ";
    rosidl_generator_traits::value_to_yaml(msg.actions_cancelled, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CancelMotion_Result & msg,
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

  // member: stop_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stop_time: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_time, out);
    out << "\n";
  }

  // member: velocity_at_stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_at_stop:\n";
    to_block_style_yaml(msg.velocity_at_stop, out, indentation + 2);
  }

  // member: final_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_pose:\n";
    to_block_style_yaml(msg.final_pose, out, indentation + 2);
  }

  // member: actions_cancelled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "actions_cancelled: ";
    rosidl_generator_traits::value_to_yaml(msg.actions_cancelled, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CancelMotion_Result & msg, bool use_flow_style = false)
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
  const rover_control::action::CancelMotion_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::CancelMotion_Result & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::CancelMotion_Result>()
{
  return "rover_control::action::CancelMotion_Result";
}

template<>
inline const char * name<rover_control::action::CancelMotion_Result>()
{
  return "rover_control/action/CancelMotion_Result";
}

template<>
struct has_fixed_size<rover_control::action::CancelMotion_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_control::action::CancelMotion_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_control::action::CancelMotion_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'current_velocity'
// already included above
// #include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const CancelMotion_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_state
  {
    out << "current_state: ";
    rosidl_generator_traits::value_to_yaml(msg.current_state, out);
    out << ", ";
  }

  // member: current_velocity
  {
    out << "current_velocity: ";
    to_flow_style_yaml(msg.current_velocity, out);
    out << ", ";
  }

  // member: progress_percentage
  {
    out << "progress_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_percentage, out);
    out << ", ";
  }

  // member: status_message
  {
    out << "status_message: ";
    rosidl_generator_traits::value_to_yaml(msg.status_message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CancelMotion_Feedback & msg,
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

  // member: current_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_velocity:\n";
    to_block_style_yaml(msg.current_velocity, out, indentation + 2);
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

  // member: status_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status_message: ";
    rosidl_generator_traits::value_to_yaml(msg.status_message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CancelMotion_Feedback & msg, bool use_flow_style = false)
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
  const rover_control::action::CancelMotion_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::CancelMotion_Feedback & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::CancelMotion_Feedback>()
{
  return "rover_control::action::CancelMotion_Feedback";
}

template<>
inline const char * name<rover_control::action::CancelMotion_Feedback>()
{
  return "rover_control/action/CancelMotion_Feedback";
}

template<>
struct has_fixed_size<rover_control::action::CancelMotion_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_control::action::CancelMotion_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_control::action::CancelMotion_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "rover_control/action/detail/cancel_motion__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const CancelMotion_SendGoal_Request & msg,
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
  const CancelMotion_SendGoal_Request & msg,
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

inline std::string to_yaml(const CancelMotion_SendGoal_Request & msg, bool use_flow_style = false)
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
  const rover_control::action::CancelMotion_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::CancelMotion_SendGoal_Request & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::CancelMotion_SendGoal_Request>()
{
  return "rover_control::action::CancelMotion_SendGoal_Request";
}

template<>
inline const char * name<rover_control::action::CancelMotion_SendGoal_Request>()
{
  return "rover_control/action/CancelMotion_SendGoal_Request";
}

template<>
struct has_fixed_size<rover_control::action::CancelMotion_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<rover_control::action::CancelMotion_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rover_control::action::CancelMotion_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<rover_control::action::CancelMotion_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rover_control::action::CancelMotion_SendGoal_Request>
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
  const CancelMotion_SendGoal_Response & msg,
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
  const CancelMotion_SendGoal_Response & msg,
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

inline std::string to_yaml(const CancelMotion_SendGoal_Response & msg, bool use_flow_style = false)
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
  const rover_control::action::CancelMotion_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::CancelMotion_SendGoal_Response & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::CancelMotion_SendGoal_Response>()
{
  return "rover_control::action::CancelMotion_SendGoal_Response";
}

template<>
inline const char * name<rover_control::action::CancelMotion_SendGoal_Response>()
{
  return "rover_control/action/CancelMotion_SendGoal_Response";
}

template<>
struct has_fixed_size<rover_control::action::CancelMotion_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<rover_control::action::CancelMotion_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<rover_control::action::CancelMotion_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rover_control::action::CancelMotion_SendGoal>()
{
  return "rover_control::action::CancelMotion_SendGoal";
}

template<>
inline const char * name<rover_control::action::CancelMotion_SendGoal>()
{
  return "rover_control/action/CancelMotion_SendGoal";
}

template<>
struct has_fixed_size<rover_control::action::CancelMotion_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<rover_control::action::CancelMotion_SendGoal_Request>::value &&
    has_fixed_size<rover_control::action::CancelMotion_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<rover_control::action::CancelMotion_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<rover_control::action::CancelMotion_SendGoal_Request>::value &&
    has_bounded_size<rover_control::action::CancelMotion_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<rover_control::action::CancelMotion_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<rover_control::action::CancelMotion_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rover_control::action::CancelMotion_SendGoal_Response>
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
  const CancelMotion_GetResult_Request & msg,
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
  const CancelMotion_GetResult_Request & msg,
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

inline std::string to_yaml(const CancelMotion_GetResult_Request & msg, bool use_flow_style = false)
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
  const rover_control::action::CancelMotion_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::CancelMotion_GetResult_Request & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::CancelMotion_GetResult_Request>()
{
  return "rover_control::action::CancelMotion_GetResult_Request";
}

template<>
inline const char * name<rover_control::action::CancelMotion_GetResult_Request>()
{
  return "rover_control/action/CancelMotion_GetResult_Request";
}

template<>
struct has_fixed_size<rover_control::action::CancelMotion_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rover_control::action::CancelMotion_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rover_control::action::CancelMotion_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "rover_control/action/detail/cancel_motion__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const CancelMotion_GetResult_Response & msg,
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
  const CancelMotion_GetResult_Response & msg,
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

inline std::string to_yaml(const CancelMotion_GetResult_Response & msg, bool use_flow_style = false)
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
  const rover_control::action::CancelMotion_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::CancelMotion_GetResult_Response & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::CancelMotion_GetResult_Response>()
{
  return "rover_control::action::CancelMotion_GetResult_Response";
}

template<>
inline const char * name<rover_control::action::CancelMotion_GetResult_Response>()
{
  return "rover_control/action/CancelMotion_GetResult_Response";
}

template<>
struct has_fixed_size<rover_control::action::CancelMotion_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<rover_control::action::CancelMotion_Result>::value> {};

template<>
struct has_bounded_size<rover_control::action::CancelMotion_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<rover_control::action::CancelMotion_Result>::value> {};

template<>
struct is_message<rover_control::action::CancelMotion_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rover_control::action::CancelMotion_GetResult>()
{
  return "rover_control::action::CancelMotion_GetResult";
}

template<>
inline const char * name<rover_control::action::CancelMotion_GetResult>()
{
  return "rover_control/action/CancelMotion_GetResult";
}

template<>
struct has_fixed_size<rover_control::action::CancelMotion_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<rover_control::action::CancelMotion_GetResult_Request>::value &&
    has_fixed_size<rover_control::action::CancelMotion_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<rover_control::action::CancelMotion_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<rover_control::action::CancelMotion_GetResult_Request>::value &&
    has_bounded_size<rover_control::action::CancelMotion_GetResult_Response>::value
  >
{
};

template<>
struct is_service<rover_control::action::CancelMotion_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<rover_control::action::CancelMotion_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rover_control::action::CancelMotion_GetResult_Response>
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
// #include "rover_control/action/detail/cancel_motion__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const CancelMotion_FeedbackMessage & msg,
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
  const CancelMotion_FeedbackMessage & msg,
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

inline std::string to_yaml(const CancelMotion_FeedbackMessage & msg, bool use_flow_style = false)
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
  const rover_control::action::CancelMotion_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::CancelMotion_FeedbackMessage & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::CancelMotion_FeedbackMessage>()
{
  return "rover_control::action::CancelMotion_FeedbackMessage";
}

template<>
inline const char * name<rover_control::action::CancelMotion_FeedbackMessage>()
{
  return "rover_control/action/CancelMotion_FeedbackMessage";
}

template<>
struct has_fixed_size<rover_control::action::CancelMotion_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<rover_control::action::CancelMotion_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rover_control::action::CancelMotion_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<rover_control::action::CancelMotion_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rover_control::action::CancelMotion_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<rover_control::action::CancelMotion>
  : std::true_type
{
};

template<>
struct is_action_goal<rover_control::action::CancelMotion_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<rover_control::action::CancelMotion_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<rover_control::action::CancelMotion_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ROVER_CONTROL__ACTION__DETAIL__CANCEL_MOTION__TRAITS_HPP_
