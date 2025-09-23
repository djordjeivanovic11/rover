// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_control:msg/DriveCommand.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__TRAITS_HPP_
#define ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_control/msg/detail/drive_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace rover_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const DriveCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: acceleration
  {
    out << "acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration, out);
    out << ", ";
  }

  // member: duration
  {
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
    out << ", ";
  }

  // member: control_mode
  {
    out << "control_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.control_mode, out);
    out << ", ";
  }

  // member: relative_command
  {
    out << "relative_command: ";
    rosidl_generator_traits::value_to_yaml(msg.relative_command, out);
    out << ", ";
  }

  // member: max_velocity
  {
    out << "max_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.max_velocity, out);
    out << ", ";
  }

  // member: max_acceleration
  {
    out << "max_acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.max_acceleration, out);
    out << ", ";
  }

  // member: enforce_safety_limits
  {
    out << "enforce_safety_limits: ";
    rosidl_generator_traits::value_to_yaml(msg.enforce_safety_limits, out);
    out << ", ";
  }

  // member: mission_mode
  {
    out << "mission_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_mode, out);
    out << ", ";
  }

  // member: curvature
  {
    out << "curvature: ";
    rosidl_generator_traits::value_to_yaml(msg.curvature, out);
    out << ", ";
  }

  // member: lookahead_distance
  {
    out << "lookahead_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.lookahead_distance, out);
    out << ", ";
  }

  // member: use_slip_compensation
  {
    out << "use_slip_compensation: ";
    rosidl_generator_traits::value_to_yaml(msg.use_slip_compensation, out);
    out << ", ";
  }

  // member: priority
  {
    out << "priority: ";
    rosidl_generator_traits::value_to_yaml(msg.priority, out);
    out << ", ";
  }

  // member: timeout
  {
    out << "timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout, out);
    out << ", ";
  }

  // member: source
  {
    out << "source: ";
    rosidl_generator_traits::value_to_yaml(msg.source, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DriveCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
  }

  // member: acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration, out);
    out << "\n";
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

  // member: control_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.control_mode, out);
    out << "\n";
  }

  // member: relative_command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "relative_command: ";
    rosidl_generator_traits::value_to_yaml(msg.relative_command, out);
    out << "\n";
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

  // member: max_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.max_acceleration, out);
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

  // member: mission_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_mode, out);
    out << "\n";
  }

  // member: curvature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "curvature: ";
    rosidl_generator_traits::value_to_yaml(msg.curvature, out);
    out << "\n";
  }

  // member: lookahead_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lookahead_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.lookahead_distance, out);
    out << "\n";
  }

  // member: use_slip_compensation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_slip_compensation: ";
    rosidl_generator_traits::value_to_yaml(msg.use_slip_compensation, out);
    out << "\n";
  }

  // member: priority
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "priority: ";
    rosidl_generator_traits::value_to_yaml(msg.priority, out);
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

  // member: source
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "source: ";
    rosidl_generator_traits::value_to_yaml(msg.source, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DriveCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rover_control

namespace rosidl_generator_traits
{

[[deprecated("use rover_control::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_control::msg::DriveCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::msg::DriveCommand & msg)
{
  return rover_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::msg::DriveCommand>()
{
  return "rover_control::msg::DriveCommand";
}

template<>
inline const char * name<rover_control::msg::DriveCommand>()
{
  return "rover_control/msg/DriveCommand";
}

template<>
struct has_fixed_size<rover_control::msg::DriveCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_control::msg::DriveCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_control::msg::DriveCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__TRAITS_HPP_
