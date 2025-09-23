// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arm_control:msg/Fault.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__MSG__DETAIL__FAULT__TRAITS_HPP_
#define ARM_CONTROL__MSG__DETAIL__FAULT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arm_control/msg/detail/fault__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'fault_location'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'time_since_last'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace arm_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const Fault & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: severity
  {
    out << "severity: ";
    rosidl_generator_traits::value_to_yaml(msg.severity, out);
    out << ", ";
  }

  // member: fault_code
  {
    out << "fault_code: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_code, out);
    out << ", ";
  }

  // member: fault_category
  {
    out << "fault_category: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_category, out);
    out << ", ";
  }

  // member: fault_description
  {
    out << "fault_description: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_description, out);
    out << ", ";
  }

  // member: component_name
  {
    out << "component_name: ";
    rosidl_generator_traits::value_to_yaml(msg.component_name, out);
    out << ", ";
  }

  // member: joint_name
  {
    out << "joint_name: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_name, out);
    out << ", ";
  }

  // member: fault_value
  {
    out << "fault_value: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_value, out);
    out << ", ";
  }

  // member: fault_threshold
  {
    out << "fault_threshold: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_threshold, out);
    out << ", ";
  }

  // member: fault_units
  {
    out << "fault_units: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_units, out);
    out << ", ";
  }

  // member: fault_location
  {
    out << "fault_location: ";
    to_flow_style_yaml(msg.fault_location, out);
    out << ", ";
  }

  // member: reference_frame
  {
    out << "reference_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.reference_frame, out);
    out << ", ";
  }

  // member: auto_recoverable
  {
    out << "auto_recoverable: ";
    rosidl_generator_traits::value_to_yaml(msg.auto_recoverable, out);
    out << ", ";
  }

  // member: recovery_attempted
  {
    out << "recovery_attempted: ";
    rosidl_generator_traits::value_to_yaml(msg.recovery_attempted, out);
    out << ", ";
  }

  // member: recovery_successful
  {
    out << "recovery_successful: ";
    rosidl_generator_traits::value_to_yaml(msg.recovery_successful, out);
    out << ", ";
  }

  // member: recovery_action
  {
    out << "recovery_action: ";
    rosidl_generator_traits::value_to_yaml(msg.recovery_action, out);
    out << ", ";
  }

  // member: system_state
  {
    out << "system_state: ";
    rosidl_generator_traits::value_to_yaml(msg.system_state, out);
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

  // member: related_faults
  {
    if (msg.related_faults.size() == 0) {
      out << "related_faults: []";
    } else {
      out << "related_faults: [";
      size_t pending_items = msg.related_faults.size();
      for (auto item : msg.related_faults) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: occurrence_count
  {
    out << "occurrence_count: ";
    rosidl_generator_traits::value_to_yaml(msg.occurrence_count, out);
    out << ", ";
  }

  // member: time_since_last
  {
    out << "time_since_last: ";
    to_flow_style_yaml(msg.time_since_last, out);
    out << ", ";
  }

  // member: recommended_action
  {
    out << "recommended_action: ";
    rosidl_generator_traits::value_to_yaml(msg.recommended_action, out);
    out << ", ";
  }

  // member: operator_intervention_required
  {
    out << "operator_intervention_required: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_intervention_required, out);
    out << ", ";
  }

  // member: troubleshooting_info
  {
    out << "troubleshooting_info: ";
    rosidl_generator_traits::value_to_yaml(msg.troubleshooting_info, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Fault & msg,
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

  // member: severity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "severity: ";
    rosidl_generator_traits::value_to_yaml(msg.severity, out);
    out << "\n";
  }

  // member: fault_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault_code: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_code, out);
    out << "\n";
  }

  // member: fault_category
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault_category: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_category, out);
    out << "\n";
  }

  // member: fault_description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault_description: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_description, out);
    out << "\n";
  }

  // member: component_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "component_name: ";
    rosidl_generator_traits::value_to_yaml(msg.component_name, out);
    out << "\n";
  }

  // member: joint_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_name: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_name, out);
    out << "\n";
  }

  // member: fault_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault_value: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_value, out);
    out << "\n";
  }

  // member: fault_threshold
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault_threshold: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_threshold, out);
    out << "\n";
  }

  // member: fault_units
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault_units: ";
    rosidl_generator_traits::value_to_yaml(msg.fault_units, out);
    out << "\n";
  }

  // member: fault_location
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fault_location:\n";
    to_block_style_yaml(msg.fault_location, out, indentation + 2);
  }

  // member: reference_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reference_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.reference_frame, out);
    out << "\n";
  }

  // member: auto_recoverable
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "auto_recoverable: ";
    rosidl_generator_traits::value_to_yaml(msg.auto_recoverable, out);
    out << "\n";
  }

  // member: recovery_attempted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "recovery_attempted: ";
    rosidl_generator_traits::value_to_yaml(msg.recovery_attempted, out);
    out << "\n";
  }

  // member: recovery_successful
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "recovery_successful: ";
    rosidl_generator_traits::value_to_yaml(msg.recovery_successful, out);
    out << "\n";
  }

  // member: recovery_action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "recovery_action: ";
    rosidl_generator_traits::value_to_yaml(msg.recovery_action, out);
    out << "\n";
  }

  // member: system_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_state: ";
    rosidl_generator_traits::value_to_yaml(msg.system_state, out);
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

  // member: related_faults
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.related_faults.size() == 0) {
      out << "related_faults: []\n";
    } else {
      out << "related_faults:\n";
      for (auto item : msg.related_faults) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: occurrence_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "occurrence_count: ";
    rosidl_generator_traits::value_to_yaml(msg.occurrence_count, out);
    out << "\n";
  }

  // member: time_since_last
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_since_last:\n";
    to_block_style_yaml(msg.time_since_last, out, indentation + 2);
  }

  // member: recommended_action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "recommended_action: ";
    rosidl_generator_traits::value_to_yaml(msg.recommended_action, out);
    out << "\n";
  }

  // member: operator_intervention_required
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operator_intervention_required: ";
    rosidl_generator_traits::value_to_yaml(msg.operator_intervention_required, out);
    out << "\n";
  }

  // member: troubleshooting_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "troubleshooting_info: ";
    rosidl_generator_traits::value_to_yaml(msg.troubleshooting_info, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Fault & msg, bool use_flow_style = false)
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
  const arm_control::msg::Fault & msg,
  std::ostream & out, size_t indentation = 0)
{
  arm_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arm_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const arm_control::msg::Fault & msg)
{
  return arm_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<arm_control::msg::Fault>()
{
  return "arm_control::msg::Fault";
}

template<>
inline const char * name<arm_control::msg::Fault>()
{
  return "arm_control/msg/Fault";
}

template<>
struct has_fixed_size<arm_control::msg::Fault>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arm_control::msg::Fault>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arm_control::msg::Fault>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARM_CONTROL__MSG__DETAIL__FAULT__TRAITS_HPP_
