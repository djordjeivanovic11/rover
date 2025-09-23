// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arm_control:action/ToolChange.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__ACTION__DETAIL__TOOL_CHANGE__BUILDER_HPP_
#define ARM_CONTROL__ACTION__DETAIL__TOOL_CHANGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arm_control/action/detail/tool_change__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_ToolChange_Goal_change_strategy
{
public:
  explicit Init_ToolChange_Goal_change_strategy(::arm_control::action::ToolChange_Goal & msg)
  : msg_(msg)
  {}
  ::arm_control::action::ToolChange_Goal change_strategy(::arm_control::action::ToolChange_Goal::_change_strategy_type arg)
  {
    msg_.change_strategy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::ToolChange_Goal msg_;
};

class Init_ToolChange_Goal_velocity_scaling
{
public:
  explicit Init_ToolChange_Goal_velocity_scaling(::arm_control::action::ToolChange_Goal & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Goal_change_strategy velocity_scaling(::arm_control::action::ToolChange_Goal::_velocity_scaling_type arg)
  {
    msg_.velocity_scaling = std::move(arg);
    return Init_ToolChange_Goal_change_strategy(msg_);
  }

private:
  ::arm_control::action::ToolChange_Goal msg_;
};

class Init_ToolChange_Goal_update_kinematics
{
public:
  explicit Init_ToolChange_Goal_update_kinematics(::arm_control::action::ToolChange_Goal & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Goal_velocity_scaling update_kinematics(::arm_control::action::ToolChange_Goal::_update_kinematics_type arg)
  {
    msg_.update_kinematics = std::move(arg);
    return Init_ToolChange_Goal_velocity_scaling(msg_);
  }

private:
  ::arm_control::action::ToolChange_Goal msg_;
};

class Init_ToolChange_Goal_verify_tool_change
{
public:
  explicit Init_ToolChange_Goal_verify_tool_change(::arm_control::action::ToolChange_Goal & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Goal_update_kinematics verify_tool_change(::arm_control::action::ToolChange_Goal::_verify_tool_change_type arg)
  {
    msg_.verify_tool_change = std::move(arg);
    return Init_ToolChange_Goal_update_kinematics(msg_);
  }

private:
  ::arm_control::action::ToolChange_Goal msg_;
};

class Init_ToolChange_Goal_coupling_timeout
{
public:
  explicit Init_ToolChange_Goal_coupling_timeout(::arm_control::action::ToolChange_Goal & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Goal_verify_tool_change coupling_timeout(::arm_control::action::ToolChange_Goal::_coupling_timeout_type arg)
  {
    msg_.coupling_timeout = std::move(arg);
    return Init_ToolChange_Goal_verify_tool_change(msg_);
  }

private:
  ::arm_control::action::ToolChange_Goal msg_;
};

class Init_ToolChange_Goal_approach_distance
{
public:
  explicit Init_ToolChange_Goal_approach_distance(::arm_control::action::ToolChange_Goal & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Goal_coupling_timeout approach_distance(::arm_control::action::ToolChange_Goal::_approach_distance_type arg)
  {
    msg_.approach_distance = std::move(arg);
    return Init_ToolChange_Goal_coupling_timeout(msg_);
  }

private:
  ::arm_control::action::ToolChange_Goal msg_;
};

class Init_ToolChange_Goal_tool_dock_pose
{
public:
  explicit Init_ToolChange_Goal_tool_dock_pose(::arm_control::action::ToolChange_Goal & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Goal_approach_distance tool_dock_pose(::arm_control::action::ToolChange_Goal::_tool_dock_pose_type arg)
  {
    msg_.tool_dock_pose = std::move(arg);
    return Init_ToolChange_Goal_approach_distance(msg_);
  }

private:
  ::arm_control::action::ToolChange_Goal msg_;
};

class Init_ToolChange_Goal_auto_detect_current
{
public:
  explicit Init_ToolChange_Goal_auto_detect_current(::arm_control::action::ToolChange_Goal & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Goal_tool_dock_pose auto_detect_current(::arm_control::action::ToolChange_Goal::_auto_detect_current_type arg)
  {
    msg_.auto_detect_current = std::move(arg);
    return Init_ToolChange_Goal_tool_dock_pose(msg_);
  }

private:
  ::arm_control::action::ToolChange_Goal msg_;
};

class Init_ToolChange_Goal_current_tool_name
{
public:
  explicit Init_ToolChange_Goal_current_tool_name(::arm_control::action::ToolChange_Goal & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Goal_auto_detect_current current_tool_name(::arm_control::action::ToolChange_Goal::_current_tool_name_type arg)
  {
    msg_.current_tool_name = std::move(arg);
    return Init_ToolChange_Goal_auto_detect_current(msg_);
  }

private:
  ::arm_control::action::ToolChange_Goal msg_;
};

class Init_ToolChange_Goal_new_tool_name
{
public:
  Init_ToolChange_Goal_new_tool_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToolChange_Goal_current_tool_name new_tool_name(::arm_control::action::ToolChange_Goal::_new_tool_name_type arg)
  {
    msg_.new_tool_name = std::move(arg);
    return Init_ToolChange_Goal_current_tool_name(msg_);
  }

private:
  ::arm_control::action::ToolChange_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::ToolChange_Goal>()
{
  return arm_control::action::builder::Init_ToolChange_Goal_new_tool_name();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_ToolChange_Result_tool_configuration
{
public:
  explicit Init_ToolChange_Result_tool_configuration(::arm_control::action::ToolChange_Result & msg)
  : msg_(msg)
  {}
  ::arm_control::action::ToolChange_Result tool_configuration(::arm_control::action::ToolChange_Result::_tool_configuration_type arg)
  {
    msg_.tool_configuration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::ToolChange_Result msg_;
};

class Init_ToolChange_Result_total_execution_time
{
public:
  explicit Init_ToolChange_Result_total_execution_time(::arm_control::action::ToolChange_Result & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Result_tool_configuration total_execution_time(::arm_control::action::ToolChange_Result::_total_execution_time_type arg)
  {
    msg_.total_execution_time = std::move(arg);
    return Init_ToolChange_Result_tool_configuration(msg_);
  }

private:
  ::arm_control::action::ToolChange_Result msg_;
};

class Init_ToolChange_Result_final_tool_pose
{
public:
  explicit Init_ToolChange_Result_final_tool_pose(::arm_control::action::ToolChange_Result & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Result_total_execution_time final_tool_pose(::arm_control::action::ToolChange_Result::_final_tool_pose_type arg)
  {
    msg_.final_tool_pose = std::move(arg);
    return Init_ToolChange_Result_total_execution_time(msg_);
  }

private:
  ::arm_control::action::ToolChange_Result msg_;
};

class Init_ToolChange_Result_kinematics_updated
{
public:
  explicit Init_ToolChange_Result_kinematics_updated(::arm_control::action::ToolChange_Result & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Result_final_tool_pose kinematics_updated(::arm_control::action::ToolChange_Result::_kinematics_updated_type arg)
  {
    msg_.kinematics_updated = std::move(arg);
    return Init_ToolChange_Result_final_tool_pose(msg_);
  }

private:
  ::arm_control::action::ToolChange_Result msg_;
};

class Init_ToolChange_Result_coupling_verified
{
public:
  explicit Init_ToolChange_Result_coupling_verified(::arm_control::action::ToolChange_Result & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Result_kinematics_updated coupling_verified(::arm_control::action::ToolChange_Result::_coupling_verified_type arg)
  {
    msg_.coupling_verified = std::move(arg);
    return Init_ToolChange_Result_kinematics_updated(msg_);
  }

private:
  ::arm_control::action::ToolChange_Result msg_;
};

class Init_ToolChange_Result_new_tool_name
{
public:
  explicit Init_ToolChange_Result_new_tool_name(::arm_control::action::ToolChange_Result & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Result_coupling_verified new_tool_name(::arm_control::action::ToolChange_Result::_new_tool_name_type arg)
  {
    msg_.new_tool_name = std::move(arg);
    return Init_ToolChange_Result_coupling_verified(msg_);
  }

private:
  ::arm_control::action::ToolChange_Result msg_;
};

class Init_ToolChange_Result_old_tool_name
{
public:
  explicit Init_ToolChange_Result_old_tool_name(::arm_control::action::ToolChange_Result & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Result_new_tool_name old_tool_name(::arm_control::action::ToolChange_Result::_old_tool_name_type arg)
  {
    msg_.old_tool_name = std::move(arg);
    return Init_ToolChange_Result_new_tool_name(msg_);
  }

private:
  ::arm_control::action::ToolChange_Result msg_;
};

class Init_ToolChange_Result_error_code
{
public:
  explicit Init_ToolChange_Result_error_code(::arm_control::action::ToolChange_Result & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Result_old_tool_name error_code(::arm_control::action::ToolChange_Result::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_ToolChange_Result_old_tool_name(msg_);
  }

private:
  ::arm_control::action::ToolChange_Result msg_;
};

class Init_ToolChange_Result_error_message
{
public:
  explicit Init_ToolChange_Result_error_message(::arm_control::action::ToolChange_Result & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Result_error_code error_message(::arm_control::action::ToolChange_Result::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_ToolChange_Result_error_code(msg_);
  }

private:
  ::arm_control::action::ToolChange_Result msg_;
};

class Init_ToolChange_Result_success
{
public:
  Init_ToolChange_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToolChange_Result_error_message success(::arm_control::action::ToolChange_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ToolChange_Result_error_message(msg_);
  }

private:
  ::arm_control::action::ToolChange_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::ToolChange_Result>()
{
  return arm_control::action::builder::Init_ToolChange_Result_success();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_ToolChange_Feedback_estimated_time_remaining
{
public:
  explicit Init_ToolChange_Feedback_estimated_time_remaining(::arm_control::action::ToolChange_Feedback & msg)
  : msg_(msg)
  {}
  ::arm_control::action::ToolChange_Feedback estimated_time_remaining(::arm_control::action::ToolChange_Feedback::_estimated_time_remaining_type arg)
  {
    msg_.estimated_time_remaining = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::ToolChange_Feedback msg_;
};

class Init_ToolChange_Feedback_status_message
{
public:
  explicit Init_ToolChange_Feedback_status_message(::arm_control::action::ToolChange_Feedback & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Feedback_estimated_time_remaining status_message(::arm_control::action::ToolChange_Feedback::_status_message_type arg)
  {
    msg_.status_message = std::move(arg);
    return Init_ToolChange_Feedback_estimated_time_remaining(msg_);
  }

private:
  ::arm_control::action::ToolChange_Feedback msg_;
};

class Init_ToolChange_Feedback_kinematics_status
{
public:
  explicit Init_ToolChange_Feedback_kinematics_status(::arm_control::action::ToolChange_Feedback & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Feedback_status_message kinematics_status(::arm_control::action::ToolChange_Feedback::_kinematics_status_type arg)
  {
    msg_.kinematics_status = std::move(arg);
    return Init_ToolChange_Feedback_status_message(msg_);
  }

private:
  ::arm_control::action::ToolChange_Feedback msg_;
};

class Init_ToolChange_Feedback_coupling_status
{
public:
  explicit Init_ToolChange_Feedback_coupling_status(::arm_control::action::ToolChange_Feedback & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Feedback_kinematics_status coupling_status(::arm_control::action::ToolChange_Feedback::_coupling_status_type arg)
  {
    msg_.coupling_status = std::move(arg);
    return Init_ToolChange_Feedback_kinematics_status(msg_);
  }

private:
  ::arm_control::action::ToolChange_Feedback msg_;
};

class Init_ToolChange_Feedback_coupling_engaged
{
public:
  explicit Init_ToolChange_Feedback_coupling_engaged(::arm_control::action::ToolChange_Feedback & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Feedback_coupling_status coupling_engaged(::arm_control::action::ToolChange_Feedback::_coupling_engaged_type arg)
  {
    msg_.coupling_engaged = std::move(arg);
    return Init_ToolChange_Feedback_coupling_status(msg_);
  }

private:
  ::arm_control::action::ToolChange_Feedback msg_;
};

class Init_ToolChange_Feedback_tool_detected
{
public:
  explicit Init_ToolChange_Feedback_tool_detected(::arm_control::action::ToolChange_Feedback & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Feedback_coupling_engaged tool_detected(::arm_control::action::ToolChange_Feedback::_tool_detected_type arg)
  {
    msg_.tool_detected = std::move(arg);
    return Init_ToolChange_Feedback_coupling_engaged(msg_);
  }

private:
  ::arm_control::action::ToolChange_Feedback msg_;
};

class Init_ToolChange_Feedback_overall_progress
{
public:
  explicit Init_ToolChange_Feedback_overall_progress(::arm_control::action::ToolChange_Feedback & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Feedback_tool_detected overall_progress(::arm_control::action::ToolChange_Feedback::_overall_progress_type arg)
  {
    msg_.overall_progress = std::move(arg);
    return Init_ToolChange_Feedback_tool_detected(msg_);
  }

private:
  ::arm_control::action::ToolChange_Feedback msg_;
};

class Init_ToolChange_Feedback_phase_progress
{
public:
  explicit Init_ToolChange_Feedback_phase_progress(::arm_control::action::ToolChange_Feedback & msg)
  : msg_(msg)
  {}
  Init_ToolChange_Feedback_overall_progress phase_progress(::arm_control::action::ToolChange_Feedback::_phase_progress_type arg)
  {
    msg_.phase_progress = std::move(arg);
    return Init_ToolChange_Feedback_overall_progress(msg_);
  }

private:
  ::arm_control::action::ToolChange_Feedback msg_;
};

class Init_ToolChange_Feedback_current_phase
{
public:
  Init_ToolChange_Feedback_current_phase()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToolChange_Feedback_phase_progress current_phase(::arm_control::action::ToolChange_Feedback::_current_phase_type arg)
  {
    msg_.current_phase = std::move(arg);
    return Init_ToolChange_Feedback_phase_progress(msg_);
  }

private:
  ::arm_control::action::ToolChange_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::ToolChange_Feedback>()
{
  return arm_control::action::builder::Init_ToolChange_Feedback_current_phase();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_ToolChange_SendGoal_Request_goal
{
public:
  explicit Init_ToolChange_SendGoal_Request_goal(::arm_control::action::ToolChange_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::arm_control::action::ToolChange_SendGoal_Request goal(::arm_control::action::ToolChange_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::ToolChange_SendGoal_Request msg_;
};

class Init_ToolChange_SendGoal_Request_goal_id
{
public:
  Init_ToolChange_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToolChange_SendGoal_Request_goal goal_id(::arm_control::action::ToolChange_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ToolChange_SendGoal_Request_goal(msg_);
  }

private:
  ::arm_control::action::ToolChange_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::ToolChange_SendGoal_Request>()
{
  return arm_control::action::builder::Init_ToolChange_SendGoal_Request_goal_id();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_ToolChange_SendGoal_Response_stamp
{
public:
  explicit Init_ToolChange_SendGoal_Response_stamp(::arm_control::action::ToolChange_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::arm_control::action::ToolChange_SendGoal_Response stamp(::arm_control::action::ToolChange_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::ToolChange_SendGoal_Response msg_;
};

class Init_ToolChange_SendGoal_Response_accepted
{
public:
  Init_ToolChange_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToolChange_SendGoal_Response_stamp accepted(::arm_control::action::ToolChange_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ToolChange_SendGoal_Response_stamp(msg_);
  }

private:
  ::arm_control::action::ToolChange_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::ToolChange_SendGoal_Response>()
{
  return arm_control::action::builder::Init_ToolChange_SendGoal_Response_accepted();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_ToolChange_GetResult_Request_goal_id
{
public:
  Init_ToolChange_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::arm_control::action::ToolChange_GetResult_Request goal_id(::arm_control::action::ToolChange_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::ToolChange_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::ToolChange_GetResult_Request>()
{
  return arm_control::action::builder::Init_ToolChange_GetResult_Request_goal_id();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_ToolChange_GetResult_Response_result
{
public:
  explicit Init_ToolChange_GetResult_Response_result(::arm_control::action::ToolChange_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::arm_control::action::ToolChange_GetResult_Response result(::arm_control::action::ToolChange_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::ToolChange_GetResult_Response msg_;
};

class Init_ToolChange_GetResult_Response_status
{
public:
  Init_ToolChange_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToolChange_GetResult_Response_result status(::arm_control::action::ToolChange_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ToolChange_GetResult_Response_result(msg_);
  }

private:
  ::arm_control::action::ToolChange_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::ToolChange_GetResult_Response>()
{
  return arm_control::action::builder::Init_ToolChange_GetResult_Response_status();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_ToolChange_FeedbackMessage_feedback
{
public:
  explicit Init_ToolChange_FeedbackMessage_feedback(::arm_control::action::ToolChange_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::arm_control::action::ToolChange_FeedbackMessage feedback(::arm_control::action::ToolChange_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::ToolChange_FeedbackMessage msg_;
};

class Init_ToolChange_FeedbackMessage_goal_id
{
public:
  Init_ToolChange_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToolChange_FeedbackMessage_feedback goal_id(::arm_control::action::ToolChange_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ToolChange_FeedbackMessage_feedback(msg_);
  }

private:
  ::arm_control::action::ToolChange_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::ToolChange_FeedbackMessage>()
{
  return arm_control::action::builder::Init_ToolChange_FeedbackMessage_goal_id();
}

}  // namespace arm_control

#endif  // ARM_CONTROL__ACTION__DETAIL__TOOL_CHANGE__BUILDER_HPP_
