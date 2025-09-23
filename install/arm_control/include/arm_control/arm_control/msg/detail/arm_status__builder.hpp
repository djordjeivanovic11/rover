// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arm_control:msg/ArmStatus.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__MSG__DETAIL__ARM_STATUS__BUILDER_HPP_
#define ARM_CONTROL__MSG__DETAIL__ARM_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arm_control/msg/detail/arm_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arm_control
{

namespace msg
{

namespace builder
{

class Init_ArmStatus_success_rate
{
public:
  explicit Init_ArmStatus_success_rate(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  ::arm_control::msg::ArmStatus success_rate(::arm_control::msg::ArmStatus::_success_rate_type arg)
  {
    msg_.success_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_fault_count
{
public:
  explicit Init_ArmStatus_fault_count(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_success_rate fault_count(::arm_control::msg::ArmStatus::_fault_count_type arg)
  {
    msg_.fault_count = std::move(arg);
    return Init_ArmStatus_success_rate(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_command_count
{
public:
  explicit Init_ArmStatus_command_count(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_fault_count command_count(::arm_control::msg::ArmStatus::_command_count_type arg)
  {
    msg_.command_count = std::move(arg);
    return Init_ArmStatus_fault_count(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_system_uptime
{
public:
  explicit Init_ArmStatus_system_uptime(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_command_count system_uptime(::arm_control::msg::ArmStatus::_system_uptime_type arg)
  {
    msg_.system_uptime = std::move(arg);
    return Init_ArmStatus_command_count(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_action_progress
{
public:
  explicit Init_ArmStatus_action_progress(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_system_uptime action_progress(::arm_control::msg::ArmStatus::_action_progress_type arg)
  {
    msg_.action_progress = std::move(arg);
    return Init_ArmStatus_system_uptime(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_current_action
{
public:
  explicit Init_ArmStatus_current_action(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_action_progress current_action(::arm_control::msg::ArmStatus::_current_action_type arg)
  {
    msg_.current_action = std::move(arg);
    return Init_ArmStatus_action_progress(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_current_mission
{
public:
  explicit Init_ArmStatus_current_mission(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_current_action current_mission(::arm_control::msg::ArmStatus::_current_mission_type arg)
  {
    msg_.current_mission = std::move(arg);
    return Init_ArmStatus_current_action(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_workspace_margin
{
public:
  explicit Init_ArmStatus_workspace_margin(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_current_mission workspace_margin(::arm_control::msg::ArmStatus::_workspace_margin_type arg)
  {
    msg_.workspace_margin = std::move(arg);
    return Init_ArmStatus_current_mission(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_workspace_valid
{
public:
  explicit Init_ArmStatus_workspace_valid(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_workspace_margin workspace_valid(::arm_control::msg::ArmStatus::_workspace_valid_type arg)
  {
    msg_.workspace_valid = std::move(arg);
    return Init_ArmStatus_workspace_margin(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_current_warning
{
public:
  explicit Init_ArmStatus_current_warning(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_workspace_valid current_warning(::arm_control::msg::ArmStatus::_current_warning_type arg)
  {
    msg_.current_warning = std::move(arg);
    return Init_ArmStatus_workspace_valid(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_max_current
{
public:
  explicit Init_ArmStatus_max_current(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_current_warning max_current(::arm_control::msg::ArmStatus::_max_current_type arg)
  {
    msg_.max_current = std::move(arg);
    return Init_ArmStatus_current_warning(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_joint_currents
{
public:
  explicit Init_ArmStatus_joint_currents(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_max_current joint_currents(::arm_control::msg::ArmStatus::_joint_currents_type arg)
  {
    msg_.joint_currents = std::move(arg);
    return Init_ArmStatus_max_current(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_temperature_warning
{
public:
  explicit Init_ArmStatus_temperature_warning(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_joint_currents temperature_warning(::arm_control::msg::ArmStatus::_temperature_warning_type arg)
  {
    msg_.temperature_warning = std::move(arg);
    return Init_ArmStatus_joint_currents(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_max_temperature
{
public:
  explicit Init_ArmStatus_max_temperature(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_temperature_warning max_temperature(::arm_control::msg::ArmStatus::_max_temperature_type arg)
  {
    msg_.max_temperature = std::move(arg);
    return Init_ArmStatus_temperature_warning(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_joint_temperatures
{
public:
  explicit Init_ArmStatus_joint_temperatures(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_max_temperature joint_temperatures(::arm_control::msg::ArmStatus::_joint_temperatures_type arg)
  {
    msg_.joint_temperatures = std::move(arg);
    return Init_ArmStatus_max_temperature(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_force_limit_active
{
public:
  explicit Init_ArmStatus_force_limit_active(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_joint_temperatures force_limit_active(::arm_control::msg::ArmStatus::_force_limit_active_type arg)
  {
    msg_.force_limit_active = std::move(arg);
    return Init_ArmStatus_joint_temperatures(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_current_wrench
{
public:
  explicit Init_ArmStatus_current_wrench(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_force_limit_active current_wrench(::arm_control::msg::ArmStatus::_current_wrench_type arg)
  {
    msg_.current_wrench = std::move(arg);
    return Init_ArmStatus_force_limit_active(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_tcp_angular_velocity
{
public:
  explicit Init_ArmStatus_tcp_angular_velocity(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_current_wrench tcp_angular_velocity(::arm_control::msg::ArmStatus::_tcp_angular_velocity_type arg)
  {
    msg_.tcp_angular_velocity = std::move(arg);
    return Init_ArmStatus_current_wrench(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_tcp_velocity
{
public:
  explicit Init_ArmStatus_tcp_velocity(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_tcp_angular_velocity tcp_velocity(::arm_control::msg::ArmStatus::_tcp_velocity_type arg)
  {
    msg_.tcp_velocity = std::move(arg);
    return Init_ArmStatus_tcp_angular_velocity(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_velocity_norm
{
public:
  explicit Init_ArmStatus_velocity_norm(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_tcp_velocity velocity_norm(::arm_control::msg::ArmStatus::_velocity_norm_type arg)
  {
    msg_.velocity_norm = std::move(arg);
    return Init_ArmStatus_tcp_velocity(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_in_motion
{
public:
  explicit Init_ArmStatus_in_motion(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_velocity_norm in_motion(::arm_control::msg::ArmStatus::_in_motion_type arg)
  {
    msg_.in_motion = std::move(arg);
    return Init_ArmStatus_velocity_norm(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_error_message
{
public:
  explicit Init_ArmStatus_error_message(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_in_motion error_message(::arm_control::msg::ArmStatus::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_ArmStatus_in_motion(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_error_code
{
public:
  explicit Init_ArmStatus_error_code(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_error_message error_code(::arm_control::msg::ArmStatus::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_ArmStatus_error_message(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_estop_active
{
public:
  explicit Init_ArmStatus_estop_active(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_error_code estop_active(::arm_control::msg::ArmStatus::_estop_active_type arg)
  {
    msg_.estop_active = std::move(arg);
    return Init_ArmStatus_error_code(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_safety_ok
{
public:
  explicit Init_ArmStatus_safety_ok(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_estop_active safety_ok(::arm_control::msg::ArmStatus::_safety_ok_type arg)
  {
    msg_.safety_ok = std::move(arg);
    return Init_ArmStatus_estop_active(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_gripper_force
{
public:
  explicit Init_ArmStatus_gripper_force(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_safety_ok gripper_force(::arm_control::msg::ArmStatus::_gripper_force_type arg)
  {
    msg_.gripper_force = std::move(arg);
    return Init_ArmStatus_safety_ok(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_gripper_opening
{
public:
  explicit Init_ArmStatus_gripper_opening(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_gripper_force gripper_opening(::arm_control::msg::ArmStatus::_gripper_opening_type arg)
  {
    msg_.gripper_opening = std::move(arg);
    return Init_ArmStatus_gripper_force(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_tool_attached
{
public:
  explicit Init_ArmStatus_tool_attached(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_gripper_opening tool_attached(::arm_control::msg::ArmStatus::_tool_attached_type arg)
  {
    msg_.tool_attached = std::move(arg);
    return Init_ArmStatus_gripper_opening(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_current_tool_name
{
public:
  explicit Init_ArmStatus_current_tool_name(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_tool_attached current_tool_name(::arm_control::msg::ArmStatus::_current_tool_name_type arg)
  {
    msg_.current_tool_name = std::move(arg);
    return Init_ArmStatus_tool_attached(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_joint_state
{
public:
  explicit Init_ArmStatus_joint_state(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_current_tool_name joint_state(::arm_control::msg::ArmStatus::_joint_state_type arg)
  {
    msg_.joint_state = std::move(arg);
    return Init_ArmStatus_current_tool_name(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_current_pose
{
public:
  explicit Init_ArmStatus_current_pose(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_joint_state current_pose(::arm_control::msg::ArmStatus::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_ArmStatus_joint_state(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_state
{
public:
  explicit Init_ArmStatus_state(::arm_control::msg::ArmStatus & msg)
  : msg_(msg)
  {}
  Init_ArmStatus_current_pose state(::arm_control::msg::ArmStatus::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_ArmStatus_current_pose(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

class Init_ArmStatus_header
{
public:
  Init_ArmStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArmStatus_state header(::arm_control::msg::ArmStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ArmStatus_state(msg_);
  }

private:
  ::arm_control::msg::ArmStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::msg::ArmStatus>()
{
  return arm_control::msg::builder::Init_ArmStatus_header();
}

}  // namespace arm_control

#endif  // ARM_CONTROL__MSG__DETAIL__ARM_STATUS__BUILDER_HPP_
