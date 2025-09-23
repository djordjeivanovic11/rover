// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_control:action/SetVelocity.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__ACTION__DETAIL__SET_VELOCITY__TRAITS_HPP_
#define ROVER_CONTROL__ACTION__DETAIL__SET_VELOCITY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_control/action/detail/set_velocity__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'target_velocity'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const SetVelocity_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_velocity
  {
    out << "target_velocity: ";
    to_flow_style_yaml(msg.target_velocity, out);
    out << ", ";
  }

  // member: duration
  {
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
    out << ", ";
  }

  // member: acceleration_limit
  {
    out << "acceleration_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration_limit, out);
    out << ", ";
  }

  // member: timeout
  {
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << ", ";
  }

  // member: mission_mode
  {
    out << "mission_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_mode, out);
    out << ", ";
  }

  // member: enforce_safety_limits
  {
    out << "enforce_safety_limits: ";
    rosidl_generator_traits::value_to_yaml(msg.enforce_safety_limits, out);
    out << ", ";
  }

  // member: ramp_to_velocity
  {
    out << "ramp_to_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.ramp_to_velocity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetVelocity_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_velocity:\n";
    to_block_style_yaml(msg.target_velocity, out, indentation + 2);
  }

  // member: duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
    out << "\n";
  }

  // member: acceleration_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration_limit, out);
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

  // member: mission_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_mode, out);
    out << "\n";
  }

  // member: enforce_safety_limits
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enforce_safety_limits: ";
    rosidl_generator_traits::value_to_yaml(msg.enforce_safety_limits, out);
    out << "\n";
  }

  // member: ramp_to_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ramp_to_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.ramp_to_velocity, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetVelocity_Goal & msg, bool use_flow_style = false)
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
  const rover_control::action::SetVelocity_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::SetVelocity_Goal & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::SetVelocity_Goal>()
{
  return "rover_control::action::SetVelocity_Goal";
}

template<>
inline const char * name<rover_control::action::SetVelocity_Goal>()
{
  return "rover_control/action/SetVelocity_Goal";
}

template<>
struct has_fixed_size<rover_control::action::SetVelocity_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_control::action::SetVelocity_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_control::action::SetVelocity_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'final_velocity'
// Member 'max_velocity_reached'
// already included above
// #include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const SetVelocity_Result & msg,
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

  // member: execution_time
  {
    out << "execution_time: ";
    rosidl_generator_traits::value_to_yaml(msg.execution_time, out);
    out << ", ";
  }

  // member: final_velocity
  {
    out << "final_velocity: ";
    to_flow_style_yaml(msg.final_velocity, out);
    out << ", ";
  }

  // member: max_velocity_reached
  {
    out << "max_velocity_reached: ";
    to_flow_style_yaml(msg.max_velocity_reached, out);
    out << ", ";
  }

  // member: distance_traveled
  {
    out << "distance_traveled: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_traveled, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetVelocity_Result & msg,
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

  // member: execution_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "execution_time: ";
    rosidl_generator_traits::value_to_yaml(msg.execution_time, out);
    out << "\n";
  }

  // member: final_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_velocity:\n";
    to_block_style_yaml(msg.final_velocity, out, indentation + 2);
  }

  // member: max_velocity_reached
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_velocity_reached:\n";
    to_block_style_yaml(msg.max_velocity_reached, out, indentation + 2);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetVelocity_Result & msg, bool use_flow_style = false)
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
  const rover_control::action::SetVelocity_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::SetVelocity_Result & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::SetVelocity_Result>()
{
  return "rover_control::action::SetVelocity_Result";
}

template<>
inline const char * name<rover_control::action::SetVelocity_Result>()
{
  return "rover_control/action/SetVelocity_Result";
}

template<>
struct has_fixed_size<rover_control::action::SetVelocity_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_control::action::SetVelocity_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_control::action::SetVelocity_Result>
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
  const SetVelocity_Feedback & msg,
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

  // member: time_remaining
  {
    out << "time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.time_remaining, out);
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
    out << ", ";
  }

  // member: safety_limit_active
  {
    out << "safety_limit_active: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_limit_active, out);
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
  const SetVelocity_Feedback & msg,
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

  // member: time_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.time_remaining, out);
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

  // member: status_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status_message: ";
    rosidl_generator_traits::value_to_yaml(msg.status_message, out);
    out << "\n";
  }

  // member: safety_limit_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_limit_active: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_limit_active, out);
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

inline std::string to_yaml(const SetVelocity_Feedback & msg, bool use_flow_style = false)
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
  const rover_control::action::SetVelocity_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::SetVelocity_Feedback & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::SetVelocity_Feedback>()
{
  return "rover_control::action::SetVelocity_Feedback";
}

template<>
inline const char * name<rover_control::action::SetVelocity_Feedback>()
{
  return "rover_control/action/SetVelocity_Feedback";
}

template<>
struct has_fixed_size<rover_control::action::SetVelocity_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_control::action::SetVelocity_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_control::action::SetVelocity_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "rover_control/action/detail/set_velocity__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const SetVelocity_SendGoal_Request & msg,
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
  const SetVelocity_SendGoal_Request & msg,
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

inline std::string to_yaml(const SetVelocity_SendGoal_Request & msg, bool use_flow_style = false)
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
  const rover_control::action::SetVelocity_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::SetVelocity_SendGoal_Request & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::SetVelocity_SendGoal_Request>()
{
  return "rover_control::action::SetVelocity_SendGoal_Request";
}

template<>
inline const char * name<rover_control::action::SetVelocity_SendGoal_Request>()
{
  return "rover_control/action/SetVelocity_SendGoal_Request";
}

template<>
struct has_fixed_size<rover_control::action::SetVelocity_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<rover_control::action::SetVelocity_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rover_control::action::SetVelocity_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<rover_control::action::SetVelocity_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rover_control::action::SetVelocity_SendGoal_Request>
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
  const SetVelocity_SendGoal_Response & msg,
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
  const SetVelocity_SendGoal_Response & msg,
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

inline std::string to_yaml(const SetVelocity_SendGoal_Response & msg, bool use_flow_style = false)
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
  const rover_control::action::SetVelocity_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::SetVelocity_SendGoal_Response & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::SetVelocity_SendGoal_Response>()
{
  return "rover_control::action::SetVelocity_SendGoal_Response";
}

template<>
inline const char * name<rover_control::action::SetVelocity_SendGoal_Response>()
{
  return "rover_control/action/SetVelocity_SendGoal_Response";
}

template<>
struct has_fixed_size<rover_control::action::SetVelocity_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<rover_control::action::SetVelocity_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<rover_control::action::SetVelocity_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rover_control::action::SetVelocity_SendGoal>()
{
  return "rover_control::action::SetVelocity_SendGoal";
}

template<>
inline const char * name<rover_control::action::SetVelocity_SendGoal>()
{
  return "rover_control/action/SetVelocity_SendGoal";
}

template<>
struct has_fixed_size<rover_control::action::SetVelocity_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<rover_control::action::SetVelocity_SendGoal_Request>::value &&
    has_fixed_size<rover_control::action::SetVelocity_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<rover_control::action::SetVelocity_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<rover_control::action::SetVelocity_SendGoal_Request>::value &&
    has_bounded_size<rover_control::action::SetVelocity_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<rover_control::action::SetVelocity_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<rover_control::action::SetVelocity_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rover_control::action::SetVelocity_SendGoal_Response>
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
  const SetVelocity_GetResult_Request & msg,
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
  const SetVelocity_GetResult_Request & msg,
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

inline std::string to_yaml(const SetVelocity_GetResult_Request & msg, bool use_flow_style = false)
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
  const rover_control::action::SetVelocity_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::SetVelocity_GetResult_Request & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::SetVelocity_GetResult_Request>()
{
  return "rover_control::action::SetVelocity_GetResult_Request";
}

template<>
inline const char * name<rover_control::action::SetVelocity_GetResult_Request>()
{
  return "rover_control/action/SetVelocity_GetResult_Request";
}

template<>
struct has_fixed_size<rover_control::action::SetVelocity_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rover_control::action::SetVelocity_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rover_control::action::SetVelocity_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "rover_control/action/detail/set_velocity__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const SetVelocity_GetResult_Response & msg,
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
  const SetVelocity_GetResult_Response & msg,
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

inline std::string to_yaml(const SetVelocity_GetResult_Response & msg, bool use_flow_style = false)
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
  const rover_control::action::SetVelocity_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::SetVelocity_GetResult_Response & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::SetVelocity_GetResult_Response>()
{
  return "rover_control::action::SetVelocity_GetResult_Response";
}

template<>
inline const char * name<rover_control::action::SetVelocity_GetResult_Response>()
{
  return "rover_control/action/SetVelocity_GetResult_Response";
}

template<>
struct has_fixed_size<rover_control::action::SetVelocity_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<rover_control::action::SetVelocity_Result>::value> {};

template<>
struct has_bounded_size<rover_control::action::SetVelocity_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<rover_control::action::SetVelocity_Result>::value> {};

template<>
struct is_message<rover_control::action::SetVelocity_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rover_control::action::SetVelocity_GetResult>()
{
  return "rover_control::action::SetVelocity_GetResult";
}

template<>
inline const char * name<rover_control::action::SetVelocity_GetResult>()
{
  return "rover_control/action/SetVelocity_GetResult";
}

template<>
struct has_fixed_size<rover_control::action::SetVelocity_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<rover_control::action::SetVelocity_GetResult_Request>::value &&
    has_fixed_size<rover_control::action::SetVelocity_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<rover_control::action::SetVelocity_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<rover_control::action::SetVelocity_GetResult_Request>::value &&
    has_bounded_size<rover_control::action::SetVelocity_GetResult_Response>::value
  >
{
};

template<>
struct is_service<rover_control::action::SetVelocity_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<rover_control::action::SetVelocity_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rover_control::action::SetVelocity_GetResult_Response>
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
// #include "rover_control/action/detail/set_velocity__traits.hpp"

namespace rover_control
{

namespace action
{

inline void to_flow_style_yaml(
  const SetVelocity_FeedbackMessage & msg,
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
  const SetVelocity_FeedbackMessage & msg,
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

inline std::string to_yaml(const SetVelocity_FeedbackMessage & msg, bool use_flow_style = false)
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
  const rover_control::action::SetVelocity_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::action::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::action::SetVelocity_FeedbackMessage & msg)
{
  return rover_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::action::SetVelocity_FeedbackMessage>()
{
  return "rover_control::action::SetVelocity_FeedbackMessage";
}

template<>
inline const char * name<rover_control::action::SetVelocity_FeedbackMessage>()
{
  return "rover_control/action/SetVelocity_FeedbackMessage";
}

template<>
struct has_fixed_size<rover_control::action::SetVelocity_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<rover_control::action::SetVelocity_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rover_control::action::SetVelocity_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<rover_control::action::SetVelocity_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rover_control::action::SetVelocity_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<rover_control::action::SetVelocity>
  : std::true_type
{
};

template<>
struct is_action_goal<rover_control::action::SetVelocity_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<rover_control::action::SetVelocity_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<rover_control::action::SetVelocity_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ROVER_CONTROL__ACTION__DETAIL__SET_VELOCITY__TRAITS_HPP_
