// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from urc_msgs:msg/RoverStatus.idl
// generated code does not contain a copyright notice

#ifndef URC_MSGS__MSG__DETAIL__ROVER_STATUS__BUILDER_HPP_
#define URC_MSGS__MSG__DETAIL__ROVER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "urc_msgs/msg/detail/rover_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace urc_msgs
{

namespace msg
{

namespace builder
{

class Init_RoverStatus_current_velocity
{
public:
  explicit Init_RoverStatus_current_velocity(::urc_msgs::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  ::urc_msgs::msg::RoverStatus current_velocity(::urc_msgs::msg::RoverStatus::_current_velocity_type arg)
  {
    msg_.current_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::urc_msgs::msg::RoverStatus msg_;
};

class Init_RoverStatus_current_pose
{
public:
  explicit Init_RoverStatus_current_pose(::urc_msgs::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_current_velocity current_pose(::urc_msgs::msg::RoverStatus::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_RoverStatus_current_velocity(msg_);
  }

private:
  ::urc_msgs::msg::RoverStatus msg_;
};

class Init_RoverStatus_current_mode
{
public:
  explicit Init_RoverStatus_current_mode(::urc_msgs::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_current_pose current_mode(::urc_msgs::msg::RoverStatus::_current_mode_type arg)
  {
    msg_.current_mode = std::move(arg);
    return Init_RoverStatus_current_pose(msg_);
  }

private:
  ::urc_msgs::msg::RoverStatus msg_;
};

class Init_RoverStatus_emergency_stop
{
public:
  explicit Init_RoverStatus_emergency_stop(::urc_msgs::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_current_mode emergency_stop(::urc_msgs::msg::RoverStatus::_emergency_stop_type arg)
  {
    msg_.emergency_stop = std::move(arg);
    return Init_RoverStatus_current_mode(msg_);
  }

private:
  ::urc_msgs::msg::RoverStatus msg_;
};

class Init_RoverStatus_motors_enabled
{
public:
  explicit Init_RoverStatus_motors_enabled(::urc_msgs::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_emergency_stop motors_enabled(::urc_msgs::msg::RoverStatus::_motors_enabled_type arg)
  {
    msg_.motors_enabled = std::move(arg);
    return Init_RoverStatus_emergency_stop(msg_);
  }

private:
  ::urc_msgs::msg::RoverStatus msg_;
};

class Init_RoverStatus_battery_percentage
{
public:
  explicit Init_RoverStatus_battery_percentage(::urc_msgs::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_motors_enabled battery_percentage(::urc_msgs::msg::RoverStatus::_battery_percentage_type arg)
  {
    msg_.battery_percentage = std::move(arg);
    return Init_RoverStatus_motors_enabled(msg_);
  }

private:
  ::urc_msgs::msg::RoverStatus msg_;
};

class Init_RoverStatus_battery_voltage
{
public:
  explicit Init_RoverStatus_battery_voltage(::urc_msgs::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_battery_percentage battery_voltage(::urc_msgs::msg::RoverStatus::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_RoverStatus_battery_percentage(msg_);
  }

private:
  ::urc_msgs::msg::RoverStatus msg_;
};

class Init_RoverStatus_header
{
public:
  Init_RoverStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoverStatus_battery_voltage header(::urc_msgs::msg::RoverStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RoverStatus_battery_voltage(msg_);
  }

private:
  ::urc_msgs::msg::RoverStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::urc_msgs::msg::RoverStatus>()
{
  return urc_msgs::msg::builder::Init_RoverStatus_header();
}

}  // namespace urc_msgs

#endif  // URC_MSGS__MSG__DETAIL__ROVER_STATUS__BUILDER_HPP_
