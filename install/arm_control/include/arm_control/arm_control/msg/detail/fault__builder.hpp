// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arm_control:msg/Fault.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__MSG__DETAIL__FAULT__BUILDER_HPP_
#define ARM_CONTROL__MSG__DETAIL__FAULT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arm_control/msg/detail/fault__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arm_control
{

namespace msg
{

namespace builder
{

class Init_Fault_troubleshooting_info
{
public:
  explicit Init_Fault_troubleshooting_info(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  ::arm_control::msg::Fault troubleshooting_info(::arm_control::msg::Fault::_troubleshooting_info_type arg)
  {
    msg_.troubleshooting_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_operator_intervention_required
{
public:
  explicit Init_Fault_operator_intervention_required(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_troubleshooting_info operator_intervention_required(::arm_control::msg::Fault::_operator_intervention_required_type arg)
  {
    msg_.operator_intervention_required = std::move(arg);
    return Init_Fault_troubleshooting_info(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_recommended_action
{
public:
  explicit Init_Fault_recommended_action(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_operator_intervention_required recommended_action(::arm_control::msg::Fault::_recommended_action_type arg)
  {
    msg_.recommended_action = std::move(arg);
    return Init_Fault_operator_intervention_required(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_time_since_last
{
public:
  explicit Init_Fault_time_since_last(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_recommended_action time_since_last(::arm_control::msg::Fault::_time_since_last_type arg)
  {
    msg_.time_since_last = std::move(arg);
    return Init_Fault_recommended_action(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_occurrence_count
{
public:
  explicit Init_Fault_occurrence_count(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_time_since_last occurrence_count(::arm_control::msg::Fault::_occurrence_count_type arg)
  {
    msg_.occurrence_count = std::move(arg);
    return Init_Fault_time_since_last(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_related_faults
{
public:
  explicit Init_Fault_related_faults(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_occurrence_count related_faults(::arm_control::msg::Fault::_related_faults_type arg)
  {
    msg_.related_faults = std::move(arg);
    return Init_Fault_occurrence_count(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_action_progress
{
public:
  explicit Init_Fault_action_progress(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_related_faults action_progress(::arm_control::msg::Fault::_action_progress_type arg)
  {
    msg_.action_progress = std::move(arg);
    return Init_Fault_related_faults(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_current_action
{
public:
  explicit Init_Fault_current_action(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_action_progress current_action(::arm_control::msg::Fault::_current_action_type arg)
  {
    msg_.current_action = std::move(arg);
    return Init_Fault_action_progress(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_system_state
{
public:
  explicit Init_Fault_system_state(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_current_action system_state(::arm_control::msg::Fault::_system_state_type arg)
  {
    msg_.system_state = std::move(arg);
    return Init_Fault_current_action(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_recovery_action
{
public:
  explicit Init_Fault_recovery_action(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_system_state recovery_action(::arm_control::msg::Fault::_recovery_action_type arg)
  {
    msg_.recovery_action = std::move(arg);
    return Init_Fault_system_state(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_recovery_successful
{
public:
  explicit Init_Fault_recovery_successful(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_recovery_action recovery_successful(::arm_control::msg::Fault::_recovery_successful_type arg)
  {
    msg_.recovery_successful = std::move(arg);
    return Init_Fault_recovery_action(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_recovery_attempted
{
public:
  explicit Init_Fault_recovery_attempted(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_recovery_successful recovery_attempted(::arm_control::msg::Fault::_recovery_attempted_type arg)
  {
    msg_.recovery_attempted = std::move(arg);
    return Init_Fault_recovery_successful(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_auto_recoverable
{
public:
  explicit Init_Fault_auto_recoverable(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_recovery_attempted auto_recoverable(::arm_control::msg::Fault::_auto_recoverable_type arg)
  {
    msg_.auto_recoverable = std::move(arg);
    return Init_Fault_recovery_attempted(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_reference_frame
{
public:
  explicit Init_Fault_reference_frame(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_auto_recoverable reference_frame(::arm_control::msg::Fault::_reference_frame_type arg)
  {
    msg_.reference_frame = std::move(arg);
    return Init_Fault_auto_recoverable(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_fault_location
{
public:
  explicit Init_Fault_fault_location(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_reference_frame fault_location(::arm_control::msg::Fault::_fault_location_type arg)
  {
    msg_.fault_location = std::move(arg);
    return Init_Fault_reference_frame(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_fault_units
{
public:
  explicit Init_Fault_fault_units(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_fault_location fault_units(::arm_control::msg::Fault::_fault_units_type arg)
  {
    msg_.fault_units = std::move(arg);
    return Init_Fault_fault_location(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_fault_threshold
{
public:
  explicit Init_Fault_fault_threshold(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_fault_units fault_threshold(::arm_control::msg::Fault::_fault_threshold_type arg)
  {
    msg_.fault_threshold = std::move(arg);
    return Init_Fault_fault_units(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_fault_value
{
public:
  explicit Init_Fault_fault_value(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_fault_threshold fault_value(::arm_control::msg::Fault::_fault_value_type arg)
  {
    msg_.fault_value = std::move(arg);
    return Init_Fault_fault_threshold(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_joint_name
{
public:
  explicit Init_Fault_joint_name(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_fault_value joint_name(::arm_control::msg::Fault::_joint_name_type arg)
  {
    msg_.joint_name = std::move(arg);
    return Init_Fault_fault_value(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_component_name
{
public:
  explicit Init_Fault_component_name(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_joint_name component_name(::arm_control::msg::Fault::_component_name_type arg)
  {
    msg_.component_name = std::move(arg);
    return Init_Fault_joint_name(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_fault_description
{
public:
  explicit Init_Fault_fault_description(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_component_name fault_description(::arm_control::msg::Fault::_fault_description_type arg)
  {
    msg_.fault_description = std::move(arg);
    return Init_Fault_component_name(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_fault_category
{
public:
  explicit Init_Fault_fault_category(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_fault_description fault_category(::arm_control::msg::Fault::_fault_category_type arg)
  {
    msg_.fault_category = std::move(arg);
    return Init_Fault_fault_description(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_fault_code
{
public:
  explicit Init_Fault_fault_code(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_fault_category fault_code(::arm_control::msg::Fault::_fault_code_type arg)
  {
    msg_.fault_code = std::move(arg);
    return Init_Fault_fault_category(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_severity
{
public:
  explicit Init_Fault_severity(::arm_control::msg::Fault & msg)
  : msg_(msg)
  {}
  Init_Fault_fault_code severity(::arm_control::msg::Fault::_severity_type arg)
  {
    msg_.severity = std::move(arg);
    return Init_Fault_fault_code(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

class Init_Fault_header
{
public:
  Init_Fault_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Fault_severity header(::arm_control::msg::Fault::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Fault_severity(msg_);
  }

private:
  ::arm_control::msg::Fault msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::msg::Fault>()
{
  return arm_control::msg::builder::Init_Fault_header();
}

}  // namespace arm_control

#endif  // ARM_CONTROL__MSG__DETAIL__FAULT__BUILDER_HPP_
