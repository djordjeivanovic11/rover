// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from urc_msgs:msg/ScienceData.idl
// generated code does not contain a copyright notice

#ifndef URC_MSGS__MSG__DETAIL__SCIENCE_DATA__TRAITS_HPP_
#define URC_MSGS__MSG__DETAIL__SCIENCE_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "urc_msgs/msg/detail/science_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'sample_location'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace urc_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ScienceData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: sample_id
  {
    out << "sample_id: ";
    rosidl_generator_traits::value_to_yaml(msg.sample_id, out);
    out << ", ";
  }

  // member: sample_location
  {
    out << "sample_location: ";
    to_flow_style_yaml(msg.sample_location, out);
    out << ", ";
  }

  // member: ph_level
  {
    out << "ph_level: ";
    rosidl_generator_traits::value_to_yaml(msg.ph_level, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: moisture
  {
    out << "moisture: ";
    rosidl_generator_traits::value_to_yaml(msg.moisture, out);
    out << ", ";
  }

  // member: detected_compounds
  {
    if (msg.detected_compounds.size() == 0) {
      out << "detected_compounds: []";
    } else {
      out << "detected_compounds: [";
      size_t pending_items = msg.detected_compounds.size();
      for (auto item : msg.detected_compounds) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ScienceData & msg,
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

  // member: sample_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sample_id: ";
    rosidl_generator_traits::value_to_yaml(msg.sample_id, out);
    out << "\n";
  }

  // member: sample_location
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sample_location:\n";
    to_block_style_yaml(msg.sample_location, out, indentation + 2);
  }

  // member: ph_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ph_level: ";
    rosidl_generator_traits::value_to_yaml(msg.ph_level, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: moisture
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "moisture: ";
    rosidl_generator_traits::value_to_yaml(msg.moisture, out);
    out << "\n";
  }

  // member: detected_compounds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.detected_compounds.size() == 0) {
      out << "detected_compounds: []\n";
    } else {
      out << "detected_compounds:\n";
      for (auto item : msg.detected_compounds) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ScienceData & msg, bool use_flow_style = false)
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

}  // namespace urc_msgs

namespace rosidl_generator_traits
{

[[deprecated("use urc_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const urc_msgs::msg::ScienceData & msg,
  std::ostream & out, size_t indentation = 0)
{
  urc_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use urc_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const urc_msgs::msg::ScienceData & msg)
{
  return urc_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<urc_msgs::msg::ScienceData>()
{
  return "urc_msgs::msg::ScienceData";
}

template<>
inline const char * name<urc_msgs::msg::ScienceData>()
{
  return "urc_msgs/msg/ScienceData";
}

template<>
struct has_fixed_size<urc_msgs::msg::ScienceData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<urc_msgs::msg::ScienceData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<urc_msgs::msg::ScienceData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // URC_MSGS__MSG__DETAIL__SCIENCE_DATA__TRAITS_HPP_
