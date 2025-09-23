// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arm_control:action/GoToNamedPose.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__ACTION__DETAIL__GO_TO_NAMED_POSE__TRAITS_HPP_
#define ARM_CONTROL__ACTION__DETAIL__GO_TO_NAMED_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arm_control/action/detail/go_to_named_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const GoToNamedPose_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose_name
  {
    out << "pose_name: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_name, out);
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
    out << ", ";
  }

  // member: plan_only
  {
    out << "plan_only: ";
    rosidl_generator_traits::value_to_yaml(msg.plan_only, out);
    out << ", ";
  }

  // member: planning_timeout
  {
    out << "planning_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.planning_timeout, out);
    out << ", ";
  }

  // member: execution_timeout
  {
    out << "execution_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.execution_timeout, out);
    out << ", ";
  }

  // member: collision_checking
  {
    out << "collision_checking: ";
    rosidl_generator_traits::value_to_yaml(msg.collision_checking, out);
    out << ", ";
  }

  // member: planning_group
  {
    out << "planning_group: ";
    rosidl_generator_traits::value_to_yaml(msg.planning_group, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoToNamedPose_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_name: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_name, out);
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

  // member: plan_only
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "plan_only: ";
    rosidl_generator_traits::value_to_yaml(msg.plan_only, out);
    out << "\n";
  }

  // member: planning_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "planning_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.planning_timeout, out);
    out << "\n";
  }

  // member: execution_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "execution_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.execution_timeout, out);
    out << "\n";
  }

  // member: collision_checking
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "collision_checking: ";
    rosidl_generator_traits::value_to_yaml(msg.collision_checking, out);
    out << "\n";
  }

  // member: planning_group
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "planning_group: ";
    rosidl_generator_traits::value_to_yaml(msg.planning_group, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoToNamedPose_Goal & msg, bool use_flow_style = false)
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
  const arm_control::action::GoToNamedPose_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::GoToNamedPose_Goal & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::GoToNamedPose_Goal>()
{
  return "arm_control::action::GoToNamedPose_Goal";
}

template<>
inline const char * name<arm_control::action::GoToNamedPose_Goal>()
{
  return "arm_control/action/GoToNamedPose_Goal";
}

template<>
struct has_fixed_size<arm_control::action::GoToNamedPose_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arm_control::action::GoToNamedPose_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arm_control::action::GoToNamedPose_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'final_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const GoToNamedPose_Result & msg,
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

  // member: final_pose
  {
    out << "final_pose: ";
    to_flow_style_yaml(msg.final_pose, out);
    out << ", ";
  }

  // member: planning_time
  {
    out << "planning_time: ";
    rosidl_generator_traits::value_to_yaml(msg.planning_time, out);
    out << ", ";
  }

  // member: execution_time
  {
    out << "execution_time: ";
    rosidl_generator_traits::value_to_yaml(msg.execution_time, out);
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoToNamedPose_Result & msg,
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

  // member: final_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_pose:\n";
    to_block_style_yaml(msg.final_pose, out, indentation + 2);
  }

  // member: planning_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "planning_time: ";
    rosidl_generator_traits::value_to_yaml(msg.planning_time, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoToNamedPose_Result & msg, bool use_flow_style = false)
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
  const arm_control::action::GoToNamedPose_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::GoToNamedPose_Result & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::GoToNamedPose_Result>()
{
  return "arm_control::action::GoToNamedPose_Result";
}

template<>
inline const char * name<arm_control::action::GoToNamedPose_Result>()
{
  return "arm_control/action/GoToNamedPose_Result";
}

template<>
struct has_fixed_size<arm_control::action::GoToNamedPose_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arm_control::action::GoToNamedPose_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arm_control::action::GoToNamedPose_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'current_joint_state'
#include "sensor_msgs/msg/detail/joint_state__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const GoToNamedPose_Feedback & msg,
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

  // member: current_joint_state
  {
    out << "current_joint_state: ";
    to_flow_style_yaml(msg.current_joint_state, out);
    out << ", ";
  }

  // member: collision_detected
  {
    out << "collision_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.collision_detected, out);
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
  const GoToNamedPose_Feedback & msg,
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

  // member: current_joint_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_joint_state:\n";
    to_block_style_yaml(msg.current_joint_state, out, indentation + 2);
  }

  // member: collision_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "collision_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.collision_detected, out);
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

inline std::string to_yaml(const GoToNamedPose_Feedback & msg, bool use_flow_style = false)
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
  const arm_control::action::GoToNamedPose_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::GoToNamedPose_Feedback & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::GoToNamedPose_Feedback>()
{
  return "arm_control::action::GoToNamedPose_Feedback";
}

template<>
inline const char * name<arm_control::action::GoToNamedPose_Feedback>()
{
  return "arm_control/action/GoToNamedPose_Feedback";
}

template<>
struct has_fixed_size<arm_control::action::GoToNamedPose_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arm_control::action::GoToNamedPose_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arm_control::action::GoToNamedPose_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "arm_control/action/detail/go_to_named_pose__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const GoToNamedPose_SendGoal_Request & msg,
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
  const GoToNamedPose_SendGoal_Request & msg,
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

inline std::string to_yaml(const GoToNamedPose_SendGoal_Request & msg, bool use_flow_style = false)
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
  const arm_control::action::GoToNamedPose_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::GoToNamedPose_SendGoal_Request & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::GoToNamedPose_SendGoal_Request>()
{
  return "arm_control::action::GoToNamedPose_SendGoal_Request";
}

template<>
inline const char * name<arm_control::action::GoToNamedPose_SendGoal_Request>()
{
  return "arm_control/action/GoToNamedPose_SendGoal_Request";
}

template<>
struct has_fixed_size<arm_control::action::GoToNamedPose_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<arm_control::action::GoToNamedPose_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<arm_control::action::GoToNamedPose_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<arm_control::action::GoToNamedPose_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<arm_control::action::GoToNamedPose_SendGoal_Request>
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
  const GoToNamedPose_SendGoal_Response & msg,
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
  const GoToNamedPose_SendGoal_Response & msg,
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

inline std::string to_yaml(const GoToNamedPose_SendGoal_Response & msg, bool use_flow_style = false)
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
  const arm_control::action::GoToNamedPose_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::GoToNamedPose_SendGoal_Response & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::GoToNamedPose_SendGoal_Response>()
{
  return "arm_control::action::GoToNamedPose_SendGoal_Response";
}

template<>
inline const char * name<arm_control::action::GoToNamedPose_SendGoal_Response>()
{
  return "arm_control/action/GoToNamedPose_SendGoal_Response";
}

template<>
struct has_fixed_size<arm_control::action::GoToNamedPose_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<arm_control::action::GoToNamedPose_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<arm_control::action::GoToNamedPose_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<arm_control::action::GoToNamedPose_SendGoal>()
{
  return "arm_control::action::GoToNamedPose_SendGoal";
}

template<>
inline const char * name<arm_control::action::GoToNamedPose_SendGoal>()
{
  return "arm_control/action/GoToNamedPose_SendGoal";
}

template<>
struct has_fixed_size<arm_control::action::GoToNamedPose_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<arm_control::action::GoToNamedPose_SendGoal_Request>::value &&
    has_fixed_size<arm_control::action::GoToNamedPose_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<arm_control::action::GoToNamedPose_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<arm_control::action::GoToNamedPose_SendGoal_Request>::value &&
    has_bounded_size<arm_control::action::GoToNamedPose_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<arm_control::action::GoToNamedPose_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<arm_control::action::GoToNamedPose_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<arm_control::action::GoToNamedPose_SendGoal_Response>
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
  const GoToNamedPose_GetResult_Request & msg,
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
  const GoToNamedPose_GetResult_Request & msg,
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

inline std::string to_yaml(const GoToNamedPose_GetResult_Request & msg, bool use_flow_style = false)
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
  const arm_control::action::GoToNamedPose_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::GoToNamedPose_GetResult_Request & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::GoToNamedPose_GetResult_Request>()
{
  return "arm_control::action::GoToNamedPose_GetResult_Request";
}

template<>
inline const char * name<arm_control::action::GoToNamedPose_GetResult_Request>()
{
  return "arm_control/action/GoToNamedPose_GetResult_Request";
}

template<>
struct has_fixed_size<arm_control::action::GoToNamedPose_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<arm_control::action::GoToNamedPose_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<arm_control::action::GoToNamedPose_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "arm_control/action/detail/go_to_named_pose__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const GoToNamedPose_GetResult_Response & msg,
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
  const GoToNamedPose_GetResult_Response & msg,
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

inline std::string to_yaml(const GoToNamedPose_GetResult_Response & msg, bool use_flow_style = false)
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
  const arm_control::action::GoToNamedPose_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::GoToNamedPose_GetResult_Response & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::GoToNamedPose_GetResult_Response>()
{
  return "arm_control::action::GoToNamedPose_GetResult_Response";
}

template<>
inline const char * name<arm_control::action::GoToNamedPose_GetResult_Response>()
{
  return "arm_control/action/GoToNamedPose_GetResult_Response";
}

template<>
struct has_fixed_size<arm_control::action::GoToNamedPose_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<arm_control::action::GoToNamedPose_Result>::value> {};

template<>
struct has_bounded_size<arm_control::action::GoToNamedPose_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<arm_control::action::GoToNamedPose_Result>::value> {};

template<>
struct is_message<arm_control::action::GoToNamedPose_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<arm_control::action::GoToNamedPose_GetResult>()
{
  return "arm_control::action::GoToNamedPose_GetResult";
}

template<>
inline const char * name<arm_control::action::GoToNamedPose_GetResult>()
{
  return "arm_control/action/GoToNamedPose_GetResult";
}

template<>
struct has_fixed_size<arm_control::action::GoToNamedPose_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<arm_control::action::GoToNamedPose_GetResult_Request>::value &&
    has_fixed_size<arm_control::action::GoToNamedPose_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<arm_control::action::GoToNamedPose_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<arm_control::action::GoToNamedPose_GetResult_Request>::value &&
    has_bounded_size<arm_control::action::GoToNamedPose_GetResult_Response>::value
  >
{
};

template<>
struct is_service<arm_control::action::GoToNamedPose_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<arm_control::action::GoToNamedPose_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<arm_control::action::GoToNamedPose_GetResult_Response>
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
// #include "arm_control/action/detail/go_to_named_pose__traits.hpp"

namespace arm_control
{

namespace action
{

inline void to_flow_style_yaml(
  const GoToNamedPose_FeedbackMessage & msg,
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
  const GoToNamedPose_FeedbackMessage & msg,
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

inline std::string to_yaml(const GoToNamedPose_FeedbackMessage & msg, bool use_flow_style = false)
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
  const arm_control::action::GoToNamedPose_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::action::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::action::GoToNamedPose_FeedbackMessage & msg)
{
  return arm_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::action::GoToNamedPose_FeedbackMessage>()
{
  return "arm_control::action::GoToNamedPose_FeedbackMessage";
}

template<>
inline const char * name<arm_control::action::GoToNamedPose_FeedbackMessage>()
{
  return "arm_control/action/GoToNamedPose_FeedbackMessage";
}

template<>
struct has_fixed_size<arm_control::action::GoToNamedPose_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<arm_control::action::GoToNamedPose_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<arm_control::action::GoToNamedPose_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<arm_control::action::GoToNamedPose_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<arm_control::action::GoToNamedPose_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<arm_control::action::GoToNamedPose>
  : std::true_type
{
};

template<>
struct is_action_goal<arm_control::action::GoToNamedPose_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<arm_control::action::GoToNamedPose_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<arm_control::action::GoToNamedPose_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ARM_CONTROL__ACTION__DETAIL__GO_TO_NAMED_POSE__TRAITS_HPP_
