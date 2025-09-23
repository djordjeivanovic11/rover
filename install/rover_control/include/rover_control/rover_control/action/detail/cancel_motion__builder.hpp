// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_control:action/CancelMotion.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__ACTION__DETAIL__CANCEL_MOTION__BUILDER_HPP_
#define ROVER_CONTROL__ACTION__DETAIL__CANCEL_MOTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_control/action/detail/cancel_motion__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_CancelMotion_Goal_reason
{
public:
  explicit Init_CancelMotion_Goal_reason(::rover_control::action::CancelMotion_Goal & msg)
  : msg_(msg)
  {}
  ::rover_control::action::CancelMotion_Goal reason(::rover_control::action::CancelMotion_Goal::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Goal msg_;
};

class Init_CancelMotion_Goal_cancel_all_actions
{
public:
  explicit Init_CancelMotion_Goal_cancel_all_actions(::rover_control::action::CancelMotion_Goal & msg)
  : msg_(msg)
  {}
  Init_CancelMotion_Goal_reason cancel_all_actions(::rover_control::action::CancelMotion_Goal::_cancel_all_actions_type arg)
  {
    msg_.cancel_all_actions = std::move(arg);
    return Init_CancelMotion_Goal_reason(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Goal msg_;
};

class Init_CancelMotion_Goal_deceleration_time
{
public:
  explicit Init_CancelMotion_Goal_deceleration_time(::rover_control::action::CancelMotion_Goal & msg)
  : msg_(msg)
  {}
  Init_CancelMotion_Goal_cancel_all_actions deceleration_time(::rover_control::action::CancelMotion_Goal::_deceleration_time_type arg)
  {
    msg_.deceleration_time = std::move(arg);
    return Init_CancelMotion_Goal_cancel_all_actions(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Goal msg_;
};

class Init_CancelMotion_Goal_emergency_stop
{
public:
  Init_CancelMotion_Goal_emergency_stop()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CancelMotion_Goal_deceleration_time emergency_stop(::rover_control::action::CancelMotion_Goal::_emergency_stop_type arg)
  {
    msg_.emergency_stop = std::move(arg);
    return Init_CancelMotion_Goal_deceleration_time(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::CancelMotion_Goal>()
{
  return rover_control::action::builder::Init_CancelMotion_Goal_emergency_stop();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_CancelMotion_Result_actions_cancelled
{
public:
  explicit Init_CancelMotion_Result_actions_cancelled(::rover_control::action::CancelMotion_Result & msg)
  : msg_(msg)
  {}
  ::rover_control::action::CancelMotion_Result actions_cancelled(::rover_control::action::CancelMotion_Result::_actions_cancelled_type arg)
  {
    msg_.actions_cancelled = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Result msg_;
};

class Init_CancelMotion_Result_final_pose
{
public:
  explicit Init_CancelMotion_Result_final_pose(::rover_control::action::CancelMotion_Result & msg)
  : msg_(msg)
  {}
  Init_CancelMotion_Result_actions_cancelled final_pose(::rover_control::action::CancelMotion_Result::_final_pose_type arg)
  {
    msg_.final_pose = std::move(arg);
    return Init_CancelMotion_Result_actions_cancelled(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Result msg_;
};

class Init_CancelMotion_Result_velocity_at_stop
{
public:
  explicit Init_CancelMotion_Result_velocity_at_stop(::rover_control::action::CancelMotion_Result & msg)
  : msg_(msg)
  {}
  Init_CancelMotion_Result_final_pose velocity_at_stop(::rover_control::action::CancelMotion_Result::_velocity_at_stop_type arg)
  {
    msg_.velocity_at_stop = std::move(arg);
    return Init_CancelMotion_Result_final_pose(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Result msg_;
};

class Init_CancelMotion_Result_stop_time
{
public:
  explicit Init_CancelMotion_Result_stop_time(::rover_control::action::CancelMotion_Result & msg)
  : msg_(msg)
  {}
  Init_CancelMotion_Result_velocity_at_stop stop_time(::rover_control::action::CancelMotion_Result::_stop_time_type arg)
  {
    msg_.stop_time = std::move(arg);
    return Init_CancelMotion_Result_velocity_at_stop(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Result msg_;
};

class Init_CancelMotion_Result_error_message
{
public:
  explicit Init_CancelMotion_Result_error_message(::rover_control::action::CancelMotion_Result & msg)
  : msg_(msg)
  {}
  Init_CancelMotion_Result_stop_time error_message(::rover_control::action::CancelMotion_Result::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_CancelMotion_Result_stop_time(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Result msg_;
};

class Init_CancelMotion_Result_result_code
{
public:
  explicit Init_CancelMotion_Result_result_code(::rover_control::action::CancelMotion_Result & msg)
  : msg_(msg)
  {}
  Init_CancelMotion_Result_error_message result_code(::rover_control::action::CancelMotion_Result::_result_code_type arg)
  {
    msg_.result_code = std::move(arg);
    return Init_CancelMotion_Result_error_message(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Result msg_;
};

class Init_CancelMotion_Result_success
{
public:
  Init_CancelMotion_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CancelMotion_Result_result_code success(::rover_control::action::CancelMotion_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CancelMotion_Result_result_code(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::CancelMotion_Result>()
{
  return rover_control::action::builder::Init_CancelMotion_Result_success();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_CancelMotion_Feedback_status_message
{
public:
  explicit Init_CancelMotion_Feedback_status_message(::rover_control::action::CancelMotion_Feedback & msg)
  : msg_(msg)
  {}
  ::rover_control::action::CancelMotion_Feedback status_message(::rover_control::action::CancelMotion_Feedback::_status_message_type arg)
  {
    msg_.status_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Feedback msg_;
};

class Init_CancelMotion_Feedback_progress_percentage
{
public:
  explicit Init_CancelMotion_Feedback_progress_percentage(::rover_control::action::CancelMotion_Feedback & msg)
  : msg_(msg)
  {}
  Init_CancelMotion_Feedback_status_message progress_percentage(::rover_control::action::CancelMotion_Feedback::_progress_percentage_type arg)
  {
    msg_.progress_percentage = std::move(arg);
    return Init_CancelMotion_Feedback_status_message(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Feedback msg_;
};

class Init_CancelMotion_Feedback_current_velocity
{
public:
  explicit Init_CancelMotion_Feedback_current_velocity(::rover_control::action::CancelMotion_Feedback & msg)
  : msg_(msg)
  {}
  Init_CancelMotion_Feedback_progress_percentage current_velocity(::rover_control::action::CancelMotion_Feedback::_current_velocity_type arg)
  {
    msg_.current_velocity = std::move(arg);
    return Init_CancelMotion_Feedback_progress_percentage(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Feedback msg_;
};

class Init_CancelMotion_Feedback_current_state
{
public:
  Init_CancelMotion_Feedback_current_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CancelMotion_Feedback_current_velocity current_state(::rover_control::action::CancelMotion_Feedback::_current_state_type arg)
  {
    msg_.current_state = std::move(arg);
    return Init_CancelMotion_Feedback_current_velocity(msg_);
  }

private:
  ::rover_control::action::CancelMotion_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::CancelMotion_Feedback>()
{
  return rover_control::action::builder::Init_CancelMotion_Feedback_current_state();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_CancelMotion_SendGoal_Request_goal
{
public:
  explicit Init_CancelMotion_SendGoal_Request_goal(::rover_control::action::CancelMotion_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::rover_control::action::CancelMotion_SendGoal_Request goal(::rover_control::action::CancelMotion_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::CancelMotion_SendGoal_Request msg_;
};

class Init_CancelMotion_SendGoal_Request_goal_id
{
public:
  Init_CancelMotion_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CancelMotion_SendGoal_Request_goal goal_id(::rover_control::action::CancelMotion_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_CancelMotion_SendGoal_Request_goal(msg_);
  }

private:
  ::rover_control::action::CancelMotion_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::CancelMotion_SendGoal_Request>()
{
  return rover_control::action::builder::Init_CancelMotion_SendGoal_Request_goal_id();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_CancelMotion_SendGoal_Response_stamp
{
public:
  explicit Init_CancelMotion_SendGoal_Response_stamp(::rover_control::action::CancelMotion_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::rover_control::action::CancelMotion_SendGoal_Response stamp(::rover_control::action::CancelMotion_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::CancelMotion_SendGoal_Response msg_;
};

class Init_CancelMotion_SendGoal_Response_accepted
{
public:
  Init_CancelMotion_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CancelMotion_SendGoal_Response_stamp accepted(::rover_control::action::CancelMotion_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_CancelMotion_SendGoal_Response_stamp(msg_);
  }

private:
  ::rover_control::action::CancelMotion_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::CancelMotion_SendGoal_Response>()
{
  return rover_control::action::builder::Init_CancelMotion_SendGoal_Response_accepted();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_CancelMotion_GetResult_Request_goal_id
{
public:
  Init_CancelMotion_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_control::action::CancelMotion_GetResult_Request goal_id(::rover_control::action::CancelMotion_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::CancelMotion_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::CancelMotion_GetResult_Request>()
{
  return rover_control::action::builder::Init_CancelMotion_GetResult_Request_goal_id();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_CancelMotion_GetResult_Response_result
{
public:
  explicit Init_CancelMotion_GetResult_Response_result(::rover_control::action::CancelMotion_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::rover_control::action::CancelMotion_GetResult_Response result(::rover_control::action::CancelMotion_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::CancelMotion_GetResult_Response msg_;
};

class Init_CancelMotion_GetResult_Response_status
{
public:
  Init_CancelMotion_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CancelMotion_GetResult_Response_result status(::rover_control::action::CancelMotion_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_CancelMotion_GetResult_Response_result(msg_);
  }

private:
  ::rover_control::action::CancelMotion_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::CancelMotion_GetResult_Response>()
{
  return rover_control::action::builder::Init_CancelMotion_GetResult_Response_status();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_CancelMotion_FeedbackMessage_feedback
{
public:
  explicit Init_CancelMotion_FeedbackMessage_feedback(::rover_control::action::CancelMotion_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::rover_control::action::CancelMotion_FeedbackMessage feedback(::rover_control::action::CancelMotion_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::CancelMotion_FeedbackMessage msg_;
};

class Init_CancelMotion_FeedbackMessage_goal_id
{
public:
  Init_CancelMotion_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CancelMotion_FeedbackMessage_feedback goal_id(::rover_control::action::CancelMotion_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_CancelMotion_FeedbackMessage_feedback(msg_);
  }

private:
  ::rover_control::action::CancelMotion_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::CancelMotion_FeedbackMessage>()
{
  return rover_control::action::builder::Init_CancelMotion_FeedbackMessage_goal_id();
}

}  // namespace rover_control

#endif  // ROVER_CONTROL__ACTION__DETAIL__CANCEL_MOTION__BUILDER_HPP_
