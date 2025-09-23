// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_control:msg/DriveCommand.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__BUILDER_HPP_
#define ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_control/msg/detail/drive_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_control
{

namespace msg
{

namespace builder
{

class Init_DriveCommand_source
{
public:
  explicit Init_DriveCommand_source(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  ::rover_control::msg::DriveCommand source(::rover_control::msg::DriveCommand::_source_type arg)
  {
    msg_.source = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_timeout
{
public:
  explicit Init_DriveCommand_timeout(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  Init_DriveCommand_source timeout(::rover_control::msg::DriveCommand::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return Init_DriveCommand_source(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_priority
{
public:
  explicit Init_DriveCommand_priority(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  Init_DriveCommand_timeout priority(::rover_control::msg::DriveCommand::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return Init_DriveCommand_timeout(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_use_slip_compensation
{
public:
  explicit Init_DriveCommand_use_slip_compensation(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  Init_DriveCommand_priority use_slip_compensation(::rover_control::msg::DriveCommand::_use_slip_compensation_type arg)
  {
    msg_.use_slip_compensation = std::move(arg);
    return Init_DriveCommand_priority(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_lookahead_distance
{
public:
  explicit Init_DriveCommand_lookahead_distance(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  Init_DriveCommand_use_slip_compensation lookahead_distance(::rover_control::msg::DriveCommand::_lookahead_distance_type arg)
  {
    msg_.lookahead_distance = std::move(arg);
    return Init_DriveCommand_use_slip_compensation(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_curvature
{
public:
  explicit Init_DriveCommand_curvature(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  Init_DriveCommand_lookahead_distance curvature(::rover_control::msg::DriveCommand::_curvature_type arg)
  {
    msg_.curvature = std::move(arg);
    return Init_DriveCommand_lookahead_distance(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_mission_mode
{
public:
  explicit Init_DriveCommand_mission_mode(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  Init_DriveCommand_curvature mission_mode(::rover_control::msg::DriveCommand::_mission_mode_type arg)
  {
    msg_.mission_mode = std::move(arg);
    return Init_DriveCommand_curvature(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_enforce_safety_limits
{
public:
  explicit Init_DriveCommand_enforce_safety_limits(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  Init_DriveCommand_mission_mode enforce_safety_limits(::rover_control::msg::DriveCommand::_enforce_safety_limits_type arg)
  {
    msg_.enforce_safety_limits = std::move(arg);
    return Init_DriveCommand_mission_mode(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_max_acceleration
{
public:
  explicit Init_DriveCommand_max_acceleration(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  Init_DriveCommand_enforce_safety_limits max_acceleration(::rover_control::msg::DriveCommand::_max_acceleration_type arg)
  {
    msg_.max_acceleration = std::move(arg);
    return Init_DriveCommand_enforce_safety_limits(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_max_velocity
{
public:
  explicit Init_DriveCommand_max_velocity(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  Init_DriveCommand_max_acceleration max_velocity(::rover_control::msg::DriveCommand::_max_velocity_type arg)
  {
    msg_.max_velocity = std::move(arg);
    return Init_DriveCommand_max_acceleration(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_relative_command
{
public:
  explicit Init_DriveCommand_relative_command(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  Init_DriveCommand_max_velocity relative_command(::rover_control::msg::DriveCommand::_relative_command_type arg)
  {
    msg_.relative_command = std::move(arg);
    return Init_DriveCommand_max_velocity(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_control_mode
{
public:
  explicit Init_DriveCommand_control_mode(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  Init_DriveCommand_relative_command control_mode(::rover_control::msg::DriveCommand::_control_mode_type arg)
  {
    msg_.control_mode = std::move(arg);
    return Init_DriveCommand_relative_command(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_duration
{
public:
  explicit Init_DriveCommand_duration(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  Init_DriveCommand_control_mode duration(::rover_control::msg::DriveCommand::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return Init_DriveCommand_control_mode(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_acceleration
{
public:
  explicit Init_DriveCommand_acceleration(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  Init_DriveCommand_duration acceleration(::rover_control::msg::DriveCommand::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_DriveCommand_duration(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_velocity
{
public:
  explicit Init_DriveCommand_velocity(::rover_control::msg::DriveCommand & msg)
  : msg_(msg)
  {}
  Init_DriveCommand_acceleration velocity(::rover_control::msg::DriveCommand::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_DriveCommand_acceleration(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

class Init_DriveCommand_header
{
public:
  Init_DriveCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DriveCommand_velocity header(::rover_control::msg::DriveCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DriveCommand_velocity(msg_);
  }

private:
  ::rover_control::msg::DriveCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::msg::DriveCommand>()
{
  return rover_control::msg::builder::Init_DriveCommand_header();
}

}  // namespace rover_control

#endif  // ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__BUILDER_HPP_
