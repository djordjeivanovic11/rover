// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from urc_msgs:msg/GapCmd.idl
// generated code does not contain a copyright notice

#ifndef URC_MSGS__MSG__DETAIL__GAP_CMD__TRAITS_HPP_
#define URC_MSGS__MSG__DETAIL__GAP_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "urc_msgs/msg/detail/gap_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace urc_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GapCmd & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GapCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GapCmd & msg, bool use_flow_style = false)
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
  const urc_msgs::msg::GapCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  urc_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use urc_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const urc_msgs::msg::GapCmd & msg)
{
  return urc_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<urc_msgs::msg::GapCmd>()
{
  return "urc_msgs::msg::GapCmd";
}

template<>
inline const char * name<urc_msgs::msg::GapCmd>()
{
  return "urc_msgs/msg/GapCmd";
}

template<>
struct has_fixed_size<urc_msgs::msg::GapCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<urc_msgs::msg::GapCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<urc_msgs::msg::GapCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // URC_MSGS__MSG__DETAIL__GAP_CMD__TRAITS_HPP_
