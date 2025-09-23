// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_control:action/SetVelocity.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__ACTION__DETAIL__SET_VELOCITY__BUILDER_HPP_
#define ROVER_CONTROL__ACTION__DETAIL__SET_VELOCITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_control/action/detail/set_velocity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_SetVelocity_Goal_ramp_to_velocity
{
public:
  explicit Init_SetVelocity_Goal_ramp_to_velocity(::rover_control::action::SetVelocity_Goal & msg)
  : msg_(msg)
  {}
  ::rover_control::action::SetVelocity_Goal ramp_to_velocity(::rover_control::action::SetVelocity_Goal::_ramp_to_velocity_type arg)
  {
    msg_.ramp_to_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Goal msg_;
};

class Init_SetVelocity_Goal_enforce_safety_limits
{
public:
  explicit Init_SetVelocity_Goal_enforce_safety_limits(::rover_control::action::SetVelocity_Goal & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Goal_ramp_to_velocity enforce_safety_limits(::rover_control::action::SetVelocity_Goal::_enforce_safety_limits_type arg)
  {
    msg_.enforce_safety_limits = std::move(arg);
    return Init_SetVelocity_Goal_ramp_to_velocity(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Goal msg_;
};

class Init_SetVelocity_Goal_mission_mode
{
public:
  explicit Init_SetVelocity_Goal_mission_mode(::rover_control::action::SetVelocity_Goal & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Goal_enforce_safety_limits mission_mode(::rover_control::action::SetVelocity_Goal::_mission_mode_type arg)
  {
    msg_.mission_mode = std::move(arg);
    return Init_SetVelocity_Goal_enforce_safety_limits(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Goal msg_;
};

class Init_SetVelocity_Goal_timeout
{
public:
  explicit Init_SetVelocity_Goal_timeout(::rover_control::action::SetVelocity_Goal & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Goal_mission_mode timeout(::rover_control::action::SetVelocity_Goal::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return Init_SetVelocity_Goal_mission_mode(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Goal msg_;
};

class Init_SetVelocity_Goal_acceleration_limit
{
public:
  explicit Init_SetVelocity_Goal_acceleration_limit(::rover_control::action::SetVelocity_Goal & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Goal_timeout acceleration_limit(::rover_control::action::SetVelocity_Goal::_acceleration_limit_type arg)
  {
    msg_.acceleration_limit = std::move(arg);
    return Init_SetVelocity_Goal_timeout(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Goal msg_;
};

class Init_SetVelocity_Goal_duration
{
public:
  explicit Init_SetVelocity_Goal_duration(::rover_control::action::SetVelocity_Goal & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Goal_acceleration_limit duration(::rover_control::action::SetVelocity_Goal::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return Init_SetVelocity_Goal_acceleration_limit(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Goal msg_;
};

class Init_SetVelocity_Goal_target_velocity
{
public:
  Init_SetVelocity_Goal_target_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetVelocity_Goal_duration target_velocity(::rover_control::action::SetVelocity_Goal::_target_velocity_type arg)
  {
    msg_.target_velocity = std::move(arg);
    return Init_SetVelocity_Goal_duration(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::SetVelocity_Goal>()
{
  return rover_control::action::builder::Init_SetVelocity_Goal_target_velocity();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_SetVelocity_Result_distance_traveled
{
public:
  explicit Init_SetVelocity_Result_distance_traveled(::rover_control::action::SetVelocity_Result & msg)
  : msg_(msg)
  {}
  ::rover_control::action::SetVelocity_Result distance_traveled(::rover_control::action::SetVelocity_Result::_distance_traveled_type arg)
  {
    msg_.distance_traveled = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Result msg_;
};

class Init_SetVelocity_Result_max_velocity_reached
{
public:
  explicit Init_SetVelocity_Result_max_velocity_reached(::rover_control::action::SetVelocity_Result & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Result_distance_traveled max_velocity_reached(::rover_control::action::SetVelocity_Result::_max_velocity_reached_type arg)
  {
    msg_.max_velocity_reached = std::move(arg);
    return Init_SetVelocity_Result_distance_traveled(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Result msg_;
};

class Init_SetVelocity_Result_final_velocity
{
public:
  explicit Init_SetVelocity_Result_final_velocity(::rover_control::action::SetVelocity_Result & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Result_max_velocity_reached final_velocity(::rover_control::action::SetVelocity_Result::_final_velocity_type arg)
  {
    msg_.final_velocity = std::move(arg);
    return Init_SetVelocity_Result_max_velocity_reached(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Result msg_;
};

class Init_SetVelocity_Result_execution_time
{
public:
  explicit Init_SetVelocity_Result_execution_time(::rover_control::action::SetVelocity_Result & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Result_final_velocity execution_time(::rover_control::action::SetVelocity_Result::_execution_time_type arg)
  {
    msg_.execution_time = std::move(arg);
    return Init_SetVelocity_Result_final_velocity(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Result msg_;
};

class Init_SetVelocity_Result_error_message
{
public:
  explicit Init_SetVelocity_Result_error_message(::rover_control::action::SetVelocity_Result & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Result_execution_time error_message(::rover_control::action::SetVelocity_Result::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_SetVelocity_Result_execution_time(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Result msg_;
};

class Init_SetVelocity_Result_result_code
{
public:
  explicit Init_SetVelocity_Result_result_code(::rover_control::action::SetVelocity_Result & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Result_error_message result_code(::rover_control::action::SetVelocity_Result::_result_code_type arg)
  {
    msg_.result_code = std::move(arg);
    return Init_SetVelocity_Result_error_message(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Result msg_;
};

class Init_SetVelocity_Result_success
{
public:
  Init_SetVelocity_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetVelocity_Result_result_code success(::rover_control::action::SetVelocity_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetVelocity_Result_result_code(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::SetVelocity_Result>()
{
  return rover_control::action::builder::Init_SetVelocity_Result_success();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_SetVelocity_Feedback_autonomy_time_remaining
{
public:
  explicit Init_SetVelocity_Feedback_autonomy_time_remaining(::rover_control::action::SetVelocity_Feedback & msg)
  : msg_(msg)
  {}
  ::rover_control::action::SetVelocity_Feedback autonomy_time_remaining(::rover_control::action::SetVelocity_Feedback::_autonomy_time_remaining_type arg)
  {
    msg_.autonomy_time_remaining = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Feedback msg_;
};

class Init_SetVelocity_Feedback_safety_limit_active
{
public:
  explicit Init_SetVelocity_Feedback_safety_limit_active(::rover_control::action::SetVelocity_Feedback & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Feedback_autonomy_time_remaining safety_limit_active(::rover_control::action::SetVelocity_Feedback::_safety_limit_active_type arg)
  {
    msg_.safety_limit_active = std::move(arg);
    return Init_SetVelocity_Feedback_autonomy_time_remaining(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Feedback msg_;
};

class Init_SetVelocity_Feedback_status_message
{
public:
  explicit Init_SetVelocity_Feedback_status_message(::rover_control::action::SetVelocity_Feedback & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Feedback_safety_limit_active status_message(::rover_control::action::SetVelocity_Feedback::_status_message_type arg)
  {
    msg_.status_message = std::move(arg);
    return Init_SetVelocity_Feedback_safety_limit_active(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Feedback msg_;
};

class Init_SetVelocity_Feedback_progress_percentage
{
public:
  explicit Init_SetVelocity_Feedback_progress_percentage(::rover_control::action::SetVelocity_Feedback & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Feedback_status_message progress_percentage(::rover_control::action::SetVelocity_Feedback::_progress_percentage_type arg)
  {
    msg_.progress_percentage = std::move(arg);
    return Init_SetVelocity_Feedback_status_message(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Feedback msg_;
};

class Init_SetVelocity_Feedback_time_remaining
{
public:
  explicit Init_SetVelocity_Feedback_time_remaining(::rover_control::action::SetVelocity_Feedback & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Feedback_progress_percentage time_remaining(::rover_control::action::SetVelocity_Feedback::_time_remaining_type arg)
  {
    msg_.time_remaining = std::move(arg);
    return Init_SetVelocity_Feedback_progress_percentage(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Feedback msg_;
};

class Init_SetVelocity_Feedback_current_velocity
{
public:
  explicit Init_SetVelocity_Feedback_current_velocity(::rover_control::action::SetVelocity_Feedback & msg)
  : msg_(msg)
  {}
  Init_SetVelocity_Feedback_time_remaining current_velocity(::rover_control::action::SetVelocity_Feedback::_current_velocity_type arg)
  {
    msg_.current_velocity = std::move(arg);
    return Init_SetVelocity_Feedback_time_remaining(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Feedback msg_;
};

class Init_SetVelocity_Feedback_current_state
{
public:
  Init_SetVelocity_Feedback_current_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetVelocity_Feedback_current_velocity current_state(::rover_control::action::SetVelocity_Feedback::_current_state_type arg)
  {
    msg_.current_state = std::move(arg);
    return Init_SetVelocity_Feedback_current_velocity(msg_);
  }

private:
  ::rover_control::action::SetVelocity_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::SetVelocity_Feedback>()
{
  return rover_control::action::builder::Init_SetVelocity_Feedback_current_state();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_SetVelocity_SendGoal_Request_goal
{
public:
  explicit Init_SetVelocity_SendGoal_Request_goal(::rover_control::action::SetVelocity_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::rover_control::action::SetVelocity_SendGoal_Request goal(::rover_control::action::SetVelocity_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::SetVelocity_SendGoal_Request msg_;
};

class Init_SetVelocity_SendGoal_Request_goal_id
{
public:
  Init_SetVelocity_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetVelocity_SendGoal_Request_goal goal_id(::rover_control::action::SetVelocity_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_SetVelocity_SendGoal_Request_goal(msg_);
  }

private:
  ::rover_control::action::SetVelocity_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::SetVelocity_SendGoal_Request>()
{
  return rover_control::action::builder::Init_SetVelocity_SendGoal_Request_goal_id();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_SetVelocity_SendGoal_Response_stamp
{
public:
  explicit Init_SetVelocity_SendGoal_Response_stamp(::rover_control::action::SetVelocity_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::rover_control::action::SetVelocity_SendGoal_Response stamp(::rover_control::action::SetVelocity_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::SetVelocity_SendGoal_Response msg_;
};

class Init_SetVelocity_SendGoal_Response_accepted
{
public:
  Init_SetVelocity_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetVelocity_SendGoal_Response_stamp accepted(::rover_control::action::SetVelocity_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_SetVelocity_SendGoal_Response_stamp(msg_);
  }

private:
  ::rover_control::action::SetVelocity_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::SetVelocity_SendGoal_Response>()
{
  return rover_control::action::builder::Init_SetVelocity_SendGoal_Response_accepted();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_SetVelocity_GetResult_Request_goal_id
{
public:
  Init_SetVelocity_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_control::action::SetVelocity_GetResult_Request goal_id(::rover_control::action::SetVelocity_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::SetVelocity_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::SetVelocity_GetResult_Request>()
{
  return rover_control::action::builder::Init_SetVelocity_GetResult_Request_goal_id();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_SetVelocity_GetResult_Response_result
{
public:
  explicit Init_SetVelocity_GetResult_Response_result(::rover_control::action::SetVelocity_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::rover_control::action::SetVelocity_GetResult_Response result(::rover_control::action::SetVelocity_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::SetVelocity_GetResult_Response msg_;
};

class Init_SetVelocity_GetResult_Response_status
{
public:
  Init_SetVelocity_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetVelocity_GetResult_Response_result status(::rover_control::action::SetVelocity_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_SetVelocity_GetResult_Response_result(msg_);
  }

private:
  ::rover_control::action::SetVelocity_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::SetVelocity_GetResult_Response>()
{
  return rover_control::action::builder::Init_SetVelocity_GetResult_Response_status();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_SetVelocity_FeedbackMessage_feedback
{
public:
  explicit Init_SetVelocity_FeedbackMessage_feedback(::rover_control::action::SetVelocity_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::rover_control::action::SetVelocity_FeedbackMessage feedback(::rover_control::action::SetVelocity_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::SetVelocity_FeedbackMessage msg_;
};

class Init_SetVelocity_FeedbackMessage_goal_id
{
public:
  Init_SetVelocity_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetVelocity_FeedbackMessage_feedback goal_id(::rover_control::action::SetVelocity_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_SetVelocity_FeedbackMessage_feedback(msg_);
  }

private:
  ::rover_control::action::SetVelocity_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::SetVelocity_FeedbackMessage>()
{
  return rover_control::action::builder::Init_SetVelocity_FeedbackMessage_goal_id();
}

}  // namespace rover_control

#endif  // ROVER_CONTROL__ACTION__DETAIL__SET_VELOCITY__BUILDER_HPP_
