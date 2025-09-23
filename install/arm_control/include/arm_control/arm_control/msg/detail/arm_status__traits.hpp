// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arm_control:msg/ArmStatus.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__MSG__DETAIL__ARM_STATUS__TRAITS_HPP_
#define ARM_CONTROL__MSG__DETAIL__ARM_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arm_control/msg/detail/arm_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'current_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'joint_state'
#include "sensor_msgs/msg/detail/joint_state__traits.hpp"
// Member 'tcp_velocity'
// Member 'tcp_angular_velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'current_wrench'
#include "geometry_msgs/msg/detail/wrench_stamped__traits.hpp"

namespace arm_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const ArmStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: current_pose
  {
    out << "current_pose: ";
    to_flow_style_yaml(msg.current_pose, out);
    out << ", ";
  }

  // member: joint_state
  {
    out << "joint_state: ";
    to_flow_style_yaml(msg.joint_state, out);
    out << ", ";
  }

  // member: current_tool_name
  {
    out << "current_tool_name: ";
    rosidl_generator_traits::value_to_yaml(msg.current_tool_name, out);
    out << ", ";
  }

  // member: tool_attached
  {
    out << "tool_attached: ";
    rosidl_generator_traits::value_to_yaml(msg.tool_attached, out);
    out << ", ";
  }

  // member: gripper_opening
  {
    out << "gripper_opening: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_opening, out);
    out << ", ";
  }

  // member: gripper_force
  {
    out << "gripper_force: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_force, out);
    out << ", ";
  }

  // member: safety_ok
  {
    out << "safety_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_ok, out);
    out << ", ";
  }

  // member: estop_active
  {
    out << "estop_active: ";
    rosidl_generator_traits::value_to_yaml(msg.estop_active, out);
    out << ", ";
  }

  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << ", ";
  }

  // member: error_message
  {
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << ", ";
  }

  // member: in_motion
  {
    out << "in_motion: ";
    rosidl_generator_traits::value_to_yaml(msg.in_motion, out);
    out << ", ";
  }

  // member: velocity_norm
  {
    out << "velocity_norm: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_norm, out);
    out << ", ";
  }

  // member: tcp_velocity
  {
    out << "tcp_velocity: ";
    to_flow_style_yaml(msg.tcp_velocity, out);
    out << ", ";
  }

  // member: tcp_angular_velocity
  {
    out << "tcp_angular_velocity: ";
    to_flow_style_yaml(msg.tcp_angular_velocity, out);
    out << ", ";
  }

  // member: current_wrench
  {
    out << "current_wrench: ";
    to_flow_style_yaml(msg.current_wrench, out);
    out << ", ";
  }

  // member: force_limit_active
  {
    out << "force_limit_active: ";
    rosidl_generator_traits::value_to_yaml(msg.force_limit_active, out);
    out << ", ";
  }

  // member: joint_temperatures
  {
    if (msg.joint_temperatures.size() == 0) {
      out << "joint_temperatures: []";
    } else {
      out << "joint_temperatures: [";
      size_t pending_items = msg.joint_temperatures.size();
      for (auto item : msg.joint_temperatures) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: max_temperature
  {
    out << "max_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.max_temperature, out);
    out << ", ";
  }

  // member: temperature_warning
  {
    out << "temperature_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature_warning, out);
    out << ", ";
  }

  // member: joint_currents
  {
    if (msg.joint_currents.size() == 0) {
      out << "joint_currents: []";
    } else {
      out << "joint_currents: [";
      size_t pending_items = msg.joint_currents.size();
      for (auto item : msg.joint_currents) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: max_current
  {
    out << "max_current: ";
    rosidl_generator_traits::value_to_yaml(msg.max_current, out);
    out << ", ";
  }

  // member: current_warning
  {
    out << "current_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.current_warning, out);
    out << ", ";
  }

  // member: workspace_valid
  {
    out << "workspace_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.workspace_valid, out);
    out << ", ";
  }

  // member: workspace_margin
  {
    out << "workspace_margin: ";
    rosidl_generator_traits::value_to_yaml(msg.workspace_margin, out);
    out << ", ";
  }

  // member: current_mission
  {
    out << "current_mission: ";
    rosidl_generator_traits::value_to_yaml(msg.current_mission, out);
    out << ", ";
  }

  // member: current_action
  {
    out << "current_action: ";
    rosidl_generator_traits::value_to_yaml(msg.current_action, out);
    out << ", ";
  }

  // member: action_progress
  {
    out << "action_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.action_progress, out);
    out << ", ";
  }

  // member: system_uptime
  {
    out << "system_uptime: ";
    rosidl_generator_traits::value_to_yaml(msg.system_uptime, out);
    out << ", ";
  }

  // member: command_count
  {
    out << "command_count: ";
    rosidl_generator_traits::value_to_yaml(msg.command_count, out);
    out << ", ";
  }

  // member: fault_count
  {
    out << "fault_count: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_count, out);
    out << ", ";
  }

  // member: success_rate
  {
    out << "success_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.success_rate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmStatus & msg,
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

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
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

  // member: joint_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_state:\n";
    to_block_style_yaml(msg.joint_state, out, indentation + 2);
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

  // member: tool_attached
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tool_attached: ";
    rosidl_generator_traits::value_to_yaml(msg.tool_attached, out);
    out << "\n";
  }

  // member: gripper_opening
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gripper_opening: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_opening, out);
    out << "\n";
  }

  // member: gripper_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gripper_force: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_force, out);
    out << "\n";
  }

  // member: safety_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_ok, out);
    out << "\n";
  }

  // member: estop_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estop_active: ";
    rosidl_generator_traits::value_to_yaml(msg.estop_active, out);
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

  // member: error_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << "\n";
  }

  // member: in_motion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "in_motion: ";
    rosidl_generator_traits::value_to_yaml(msg.in_motion, out);
    out << "\n";
  }

  // member: velocity_norm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_norm: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_norm, out);
    out << "\n";
  }

  // member: tcp_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tcp_velocity:\n";
    to_block_style_yaml(msg.tcp_velocity, out, indentation + 2);
  }

  // member: tcp_angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tcp_angular_velocity:\n";
    to_block_style_yaml(msg.tcp_angular_velocity, out, indentation + 2);
  }

  // member: current_wrench
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_wrench:\n";
    to_block_style_yaml(msg.current_wrench, out, indentation + 2);
  }

  // member: force_limit_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "force_limit_active: ";
    rosidl_generator_traits::value_to_yaml(msg.force_limit_active, out);
    out << "\n";
  }

  // member: joint_temperatures
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_temperatures.size() == 0) {
      out << "joint_temperatures: []\n";
    } else {
      out << "joint_temperatures:\n";
      for (auto item : msg.joint_temperatures) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: max_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.max_temperature, out);
    out << "\n";
  }

  // member: temperature_warning
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature_warning, out);
    out << "\n";
  }

  // member: joint_currents
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_currents.size() == 0) {
      out << "joint_currents: []\n";
    } else {
      out << "joint_currents:\n";
      for (auto item : msg.joint_currents) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: max_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_current: ";
    rosidl_generator_traits::value_to_yaml(msg.max_current, out);
    out << "\n";
  }

  // member: current_warning
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.current_warning, out);
    out << "\n";
  }

  // member: workspace_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "workspace_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.workspace_valid, out);
    out << "\n";
  }

  // member: workspace_margin
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "workspace_margin: ";
    rosidl_generator_traits::value_to_yaml(msg.workspace_margin, out);
    out << "\n";
  }

  // member: current_mission
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_mission: ";
    rosidl_generator_traits::value_to_yaml(msg.current_mission, out);
    out << "\n";
  }

  // member: current_action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_action: ";
    rosidl_generator_traits::value_to_yaml(msg.current_action, out);
    out << "\n";
  }

  // member: action_progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.action_progress, out);
    out << "\n";
  }

  // member: system_uptime
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_uptime: ";
    rosidl_generator_traits::value_to_yaml(msg.system_uptime, out);
    out << "\n";
  }

  // member: command_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command_count: ";
    rosidl_generator_traits::value_to_yaml(msg.command_count, out);
    out << "\n";
  }

  // member: fault_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault_count: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_count, out);
    out << "\n";
  }

  // member: success_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.success_rate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmStatus & msg, bool use_flow_style = false)
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

}  // namespace arm_control

namespace rosidl_generator_traits
{

[[deprecated("use arm_control::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arm_control::msg::ArmStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::msg::ArmStatus & msg)
{
  return arm_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::msg::ArmStatus>()
{
  return "arm_control::msg::ArmStatus";
}

template<>
inline const char * name<arm_control::msg::ArmStatus>()
{
  return "arm_control/msg/ArmStatus";
}

template<>
struct has_fixed_size<arm_control::msg::ArmStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arm_control::msg::ArmStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arm_control::msg::ArmStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARM_CONTROL__MSG__DETAIL__ARM_STATUS__TRAITS_HPP_
