// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arm_control:action/GoToNamedPose.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__ACTION__DETAIL__GO_TO_NAMED_POSE__BUILDER_HPP_
#define ARM_CONTROL__ACTION__DETAIL__GO_TO_NAMED_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arm_control/action/detail/go_to_named_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_GoToNamedPose_Goal_planning_group
{
public:
  explicit Init_GoToNamedPose_Goal_planning_group(::arm_control::action::GoToNamedPose_Goal & msg)
  : msg_(msg)
  {}
  ::arm_control::action::GoToNamedPose_Goal planning_group(::arm_control::action::GoToNamedPose_Goal::_planning_group_type arg)
  {
    msg_.planning_group = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Goal msg_;
};

class Init_GoToNamedPose_Goal_collision_checking
{
public:
  explicit Init_GoToNamedPose_Goal_collision_checking(::arm_control::action::GoToNamedPose_Goal & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Goal_planning_group collision_checking(::arm_control::action::GoToNamedPose_Goal::_collision_checking_type arg)
  {
    msg_.collision_checking = std::move(arg);
    return Init_GoToNamedPose_Goal_planning_group(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Goal msg_;
};

class Init_GoToNamedPose_Goal_execution_timeout
{
public:
  explicit Init_GoToNamedPose_Goal_execution_timeout(::arm_control::action::GoToNamedPose_Goal & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Goal_collision_checking execution_timeout(::arm_control::action::GoToNamedPose_Goal::_execution_timeout_type arg)
  {
    msg_.execution_timeout = std::move(arg);
    return Init_GoToNamedPose_Goal_collision_checking(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Goal msg_;
};

class Init_GoToNamedPose_Goal_planning_timeout
{
public:
  explicit Init_GoToNamedPose_Goal_planning_timeout(::arm_control::action::GoToNamedPose_Goal & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Goal_execution_timeout planning_timeout(::arm_control::action::GoToNamedPose_Goal::_planning_timeout_type arg)
  {
    msg_.planning_timeout = std::move(arg);
    return Init_GoToNamedPose_Goal_execution_timeout(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Goal msg_;
};

class Init_GoToNamedPose_Goal_plan_only
{
public:
  explicit Init_GoToNamedPose_Goal_plan_only(::arm_control::action::GoToNamedPose_Goal & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Goal_planning_timeout plan_only(::arm_control::action::GoToNamedPose_Goal::_plan_only_type arg)
  {
    msg_.plan_only = std::move(arg);
    return Init_GoToNamedPose_Goal_planning_timeout(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Goal msg_;
};

class Init_GoToNamedPose_Goal_acceleration_scaling
{
public:
  explicit Init_GoToNamedPose_Goal_acceleration_scaling(::arm_control::action::GoToNamedPose_Goal & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Goal_plan_only acceleration_scaling(::arm_control::action::GoToNamedPose_Goal::_acceleration_scaling_type arg)
  {
    msg_.acceleration_scaling = std::move(arg);
    return Init_GoToNamedPose_Goal_plan_only(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Goal msg_;
};

class Init_GoToNamedPose_Goal_velocity_scaling
{
public:
  explicit Init_GoToNamedPose_Goal_velocity_scaling(::arm_control::action::GoToNamedPose_Goal & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Goal_acceleration_scaling velocity_scaling(::arm_control::action::GoToNamedPose_Goal::_velocity_scaling_type arg)
  {
    msg_.velocity_scaling = std::move(arg);
    return Init_GoToNamedPose_Goal_acceleration_scaling(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Goal msg_;
};

class Init_GoToNamedPose_Goal_pose_name
{
public:
  Init_GoToNamedPose_Goal_pose_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToNamedPose_Goal_velocity_scaling pose_name(::arm_control::action::GoToNamedPose_Goal::_pose_name_type arg)
  {
    msg_.pose_name = std::move(arg);
    return Init_GoToNamedPose_Goal_velocity_scaling(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::GoToNamedPose_Goal>()
{
  return arm_control::action::builder::Init_GoToNamedPose_Goal_pose_name();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_GoToNamedPose_Result_final_orientation_error
{
public:
  explicit Init_GoToNamedPose_Result_final_orientation_error(::arm_control::action::GoToNamedPose_Result & msg)
  : msg_(msg)
  {}
  ::arm_control::action::GoToNamedPose_Result final_orientation_error(::arm_control::action::GoToNamedPose_Result::_final_orientation_error_type arg)
  {
    msg_.final_orientation_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Result msg_;
};

class Init_GoToNamedPose_Result_final_position_error
{
public:
  explicit Init_GoToNamedPose_Result_final_position_error(::arm_control::action::GoToNamedPose_Result & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Result_final_orientation_error final_position_error(::arm_control::action::GoToNamedPose_Result::_final_position_error_type arg)
  {
    msg_.final_position_error = std::move(arg);
    return Init_GoToNamedPose_Result_final_orientation_error(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Result msg_;
};

class Init_GoToNamedPose_Result_execution_time
{
public:
  explicit Init_GoToNamedPose_Result_execution_time(::arm_control::action::GoToNamedPose_Result & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Result_final_position_error execution_time(::arm_control::action::GoToNamedPose_Result::_execution_time_type arg)
  {
    msg_.execution_time = std::move(arg);
    return Init_GoToNamedPose_Result_final_position_error(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Result msg_;
};

class Init_GoToNamedPose_Result_planning_time
{
public:
  explicit Init_GoToNamedPose_Result_planning_time(::arm_control::action::GoToNamedPose_Result & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Result_execution_time planning_time(::arm_control::action::GoToNamedPose_Result::_planning_time_type arg)
  {
    msg_.planning_time = std::move(arg);
    return Init_GoToNamedPose_Result_execution_time(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Result msg_;
};

class Init_GoToNamedPose_Result_final_pose
{
public:
  explicit Init_GoToNamedPose_Result_final_pose(::arm_control::action::GoToNamedPose_Result & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Result_planning_time final_pose(::arm_control::action::GoToNamedPose_Result::_final_pose_type arg)
  {
    msg_.final_pose = std::move(arg);
    return Init_GoToNamedPose_Result_planning_time(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Result msg_;
};

class Init_GoToNamedPose_Result_error_code
{
public:
  explicit Init_GoToNamedPose_Result_error_code(::arm_control::action::GoToNamedPose_Result & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Result_final_pose error_code(::arm_control::action::GoToNamedPose_Result::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_GoToNamedPose_Result_final_pose(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Result msg_;
};

class Init_GoToNamedPose_Result_error_message
{
public:
  explicit Init_GoToNamedPose_Result_error_message(::arm_control::action::GoToNamedPose_Result & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Result_error_code error_message(::arm_control::action::GoToNamedPose_Result::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_GoToNamedPose_Result_error_code(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Result msg_;
};

class Init_GoToNamedPose_Result_success
{
public:
  Init_GoToNamedPose_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToNamedPose_Result_error_message success(::arm_control::action::GoToNamedPose_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GoToNamedPose_Result_error_message(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::GoToNamedPose_Result>()
{
  return arm_control::action::builder::Init_GoToNamedPose_Result_success();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_GoToNamedPose_Feedback_status_message
{
public:
  explicit Init_GoToNamedPose_Feedback_status_message(::arm_control::action::GoToNamedPose_Feedback & msg)
  : msg_(msg)
  {}
  ::arm_control::action::GoToNamedPose_Feedback status_message(::arm_control::action::GoToNamedPose_Feedback::_status_message_type arg)
  {
    msg_.status_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Feedback msg_;
};

class Init_GoToNamedPose_Feedback_collision_detected
{
public:
  explicit Init_GoToNamedPose_Feedback_collision_detected(::arm_control::action::GoToNamedPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Feedback_status_message collision_detected(::arm_control::action::GoToNamedPose_Feedback::_collision_detected_type arg)
  {
    msg_.collision_detected = std::move(arg);
    return Init_GoToNamedPose_Feedback_status_message(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Feedback msg_;
};

class Init_GoToNamedPose_Feedback_current_joint_state
{
public:
  explicit Init_GoToNamedPose_Feedback_current_joint_state(::arm_control::action::GoToNamedPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Feedback_collision_detected current_joint_state(::arm_control::action::GoToNamedPose_Feedback::_current_joint_state_type arg)
  {
    msg_.current_joint_state = std::move(arg);
    return Init_GoToNamedPose_Feedback_collision_detected(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Feedback msg_;
};

class Init_GoToNamedPose_Feedback_estimated_time_remaining
{
public:
  explicit Init_GoToNamedPose_Feedback_estimated_time_remaining(::arm_control::action::GoToNamedPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Feedback_current_joint_state estimated_time_remaining(::arm_control::action::GoToNamedPose_Feedback::_estimated_time_remaining_type arg)
  {
    msg_.estimated_time_remaining = std::move(arg);
    return Init_GoToNamedPose_Feedback_current_joint_state(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Feedback msg_;
};

class Init_GoToNamedPose_Feedback_current_pose
{
public:
  explicit Init_GoToNamedPose_Feedback_current_pose(::arm_control::action::GoToNamedPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Feedback_estimated_time_remaining current_pose(::arm_control::action::GoToNamedPose_Feedback::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_GoToNamedPose_Feedback_estimated_time_remaining(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Feedback msg_;
};

class Init_GoToNamedPose_Feedback_progress_percentage
{
public:
  explicit Init_GoToNamedPose_Feedback_progress_percentage(::arm_control::action::GoToNamedPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_GoToNamedPose_Feedback_current_pose progress_percentage(::arm_control::action::GoToNamedPose_Feedback::_progress_percentage_type arg)
  {
    msg_.progress_percentage = std::move(arg);
    return Init_GoToNamedPose_Feedback_current_pose(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Feedback msg_;
};

class Init_GoToNamedPose_Feedback_current_state
{
public:
  Init_GoToNamedPose_Feedback_current_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToNamedPose_Feedback_progress_percentage current_state(::arm_control::action::GoToNamedPose_Feedback::_current_state_type arg)
  {
    msg_.current_state = std::move(arg);
    return Init_GoToNamedPose_Feedback_progress_percentage(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::GoToNamedPose_Feedback>()
{
  return arm_control::action::builder::Init_GoToNamedPose_Feedback_current_state();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_GoToNamedPose_SendGoal_Request_goal
{
public:
  explicit Init_GoToNamedPose_SendGoal_Request_goal(::arm_control::action::GoToNamedPose_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::arm_control::action::GoToNamedPose_SendGoal_Request goal(::arm_control::action::GoToNamedPose_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_SendGoal_Request msg_;
};

class Init_GoToNamedPose_SendGoal_Request_goal_id
{
public:
  Init_GoToNamedPose_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToNamedPose_SendGoal_Request_goal goal_id(::arm_control::action::GoToNamedPose_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_GoToNamedPose_SendGoal_Request_goal(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::GoToNamedPose_SendGoal_Request>()
{
  return arm_control::action::builder::Init_GoToNamedPose_SendGoal_Request_goal_id();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_GoToNamedPose_SendGoal_Response_stamp
{
public:
  explicit Init_GoToNamedPose_SendGoal_Response_stamp(::arm_control::action::GoToNamedPose_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::arm_control::action::GoToNamedPose_SendGoal_Response stamp(::arm_control::action::GoToNamedPose_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_SendGoal_Response msg_;
};

class Init_GoToNamedPose_SendGoal_Response_accepted
{
public:
  Init_GoToNamedPose_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToNamedPose_SendGoal_Response_stamp accepted(::arm_control::action::GoToNamedPose_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_GoToNamedPose_SendGoal_Response_stamp(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::GoToNamedPose_SendGoal_Response>()
{
  return arm_control::action::builder::Init_GoToNamedPose_SendGoal_Response_accepted();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_GoToNamedPose_GetResult_Request_goal_id
{
public:
  Init_GoToNamedPose_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::arm_control::action::GoToNamedPose_GetResult_Request goal_id(::arm_control::action::GoToNamedPose_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::GoToNamedPose_GetResult_Request>()
{
  return arm_control::action::builder::Init_GoToNamedPose_GetResult_Request_goal_id();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_GoToNamedPose_GetResult_Response_result
{
public:
  explicit Init_GoToNamedPose_GetResult_Response_result(::arm_control::action::GoToNamedPose_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::arm_control::action::GoToNamedPose_GetResult_Response result(::arm_control::action::GoToNamedPose_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_GetResult_Response msg_;
};

class Init_GoToNamedPose_GetResult_Response_status
{
public:
  Init_GoToNamedPose_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToNamedPose_GetResult_Response_result status(::arm_control::action::GoToNamedPose_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_GoToNamedPose_GetResult_Response_result(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::GoToNamedPose_GetResult_Response>()
{
  return arm_control::action::builder::Init_GoToNamedPose_GetResult_Response_status();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_GoToNamedPose_FeedbackMessage_feedback
{
public:
  explicit Init_GoToNamedPose_FeedbackMessage_feedback(::arm_control::action::GoToNamedPose_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::arm_control::action::GoToNamedPose_FeedbackMessage feedback(::arm_control::action::GoToNamedPose_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_FeedbackMessage msg_;
};

class Init_GoToNamedPose_FeedbackMessage_goal_id
{
public:
  Init_GoToNamedPose_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToNamedPose_FeedbackMessage_feedback goal_id(::arm_control::action::GoToNamedPose_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_GoToNamedPose_FeedbackMessage_feedback(msg_);
  }

private:
  ::arm_control::action::GoToNamedPose_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::GoToNamedPose_FeedbackMessage>()
{
  return arm_control::action::builder::Init_GoToNamedPose_FeedbackMessage_goal_id();
}

}  // namespace arm_control

#endif  // ARM_CONTROL__ACTION__DETAIL__GO_TO_NAMED_POSE__BUILDER_HPP_
