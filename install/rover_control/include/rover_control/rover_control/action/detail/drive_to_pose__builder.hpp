// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_control:action/DriveToPose.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__ACTION__DETAIL__DRIVE_TO_POSE__BUILDER_HPP_
#define ROVER_CONTROL__ACTION__DETAIL__DRIVE_TO_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_control/action/detail/drive_to_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_DriveToPose_Goal_approach_velocity
{
public:
  explicit Init_DriveToPose_Goal_approach_velocity(::rover_control::action::DriveToPose_Goal & msg)
  : msg_(msg)
  {}
  ::rover_control::action::DriveToPose_Goal approach_velocity(::rover_control::action::DriveToPose_Goal::_approach_velocity_type arg)
  {
    msg_.approach_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Goal msg_;
};

class Init_DriveToPose_Goal_reverse_allowed
{
public:
  explicit Init_DriveToPose_Goal_reverse_allowed(::rover_control::action::DriveToPose_Goal & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Goal_approach_velocity reverse_allowed(::rover_control::action::DriveToPose_Goal::_reverse_allowed_type arg)
  {
    msg_.reverse_allowed = std::move(arg);
    return Init_DriveToPose_Goal_approach_velocity(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Goal msg_;
};

class Init_DriveToPose_Goal_mission_mode
{
public:
  explicit Init_DriveToPose_Goal_mission_mode(::rover_control::action::DriveToPose_Goal & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Goal_reverse_allowed mission_mode(::rover_control::action::DriveToPose_Goal::_mission_mode_type arg)
  {
    msg_.mission_mode = std::move(arg);
    return Init_DriveToPose_Goal_reverse_allowed(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Goal msg_;
};

class Init_DriveToPose_Goal_use_visual_servoing
{
public:
  explicit Init_DriveToPose_Goal_use_visual_servoing(::rover_control::action::DriveToPose_Goal & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Goal_mission_mode use_visual_servoing(::rover_control::action::DriveToPose_Goal::_use_visual_servoing_type arg)
  {
    msg_.use_visual_servoing = std::move(arg);
    return Init_DriveToPose_Goal_mission_mode(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Goal msg_;
};

class Init_DriveToPose_Goal_timeout
{
public:
  explicit Init_DriveToPose_Goal_timeout(::rover_control::action::DriveToPose_Goal & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Goal_use_visual_servoing timeout(::rover_control::action::DriveToPose_Goal::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return Init_DriveToPose_Goal_use_visual_servoing(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Goal msg_;
};

class Init_DriveToPose_Goal_orientation_tolerance
{
public:
  explicit Init_DriveToPose_Goal_orientation_tolerance(::rover_control::action::DriveToPose_Goal & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Goal_timeout orientation_tolerance(::rover_control::action::DriveToPose_Goal::_orientation_tolerance_type arg)
  {
    msg_.orientation_tolerance = std::move(arg);
    return Init_DriveToPose_Goal_timeout(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Goal msg_;
};

class Init_DriveToPose_Goal_position_tolerance
{
public:
  explicit Init_DriveToPose_Goal_position_tolerance(::rover_control::action::DriveToPose_Goal & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Goal_orientation_tolerance position_tolerance(::rover_control::action::DriveToPose_Goal::_position_tolerance_type arg)
  {
    msg_.position_tolerance = std::move(arg);
    return Init_DriveToPose_Goal_orientation_tolerance(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Goal msg_;
};

class Init_DriveToPose_Goal_max_velocity
{
public:
  explicit Init_DriveToPose_Goal_max_velocity(::rover_control::action::DriveToPose_Goal & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Goal_position_tolerance max_velocity(::rover_control::action::DriveToPose_Goal::_max_velocity_type arg)
  {
    msg_.max_velocity = std::move(arg);
    return Init_DriveToPose_Goal_position_tolerance(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Goal msg_;
};

class Init_DriveToPose_Goal_target_pose
{
public:
  Init_DriveToPose_Goal_target_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DriveToPose_Goal_max_velocity target_pose(::rover_control::action::DriveToPose_Goal::_target_pose_type arg)
  {
    msg_.target_pose = std::move(arg);
    return Init_DriveToPose_Goal_max_velocity(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::DriveToPose_Goal>()
{
  return rover_control::action::builder::Init_DriveToPose_Goal_target_pose();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_DriveToPose_Result_executed_path
{
public:
  explicit Init_DriveToPose_Result_executed_path(::rover_control::action::DriveToPose_Result & msg)
  : msg_(msg)
  {}
  ::rover_control::action::DriveToPose_Result executed_path(::rover_control::action::DriveToPose_Result::_executed_path_type arg)
  {
    msg_.executed_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Result msg_;
};

class Init_DriveToPose_Result_final_pose
{
public:
  explicit Init_DriveToPose_Result_final_pose(::rover_control::action::DriveToPose_Result & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Result_executed_path final_pose(::rover_control::action::DriveToPose_Result::_final_pose_type arg)
  {
    msg_.final_pose = std::move(arg);
    return Init_DriveToPose_Result_executed_path(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Result msg_;
};

class Init_DriveToPose_Result_distance_traveled
{
public:
  explicit Init_DriveToPose_Result_distance_traveled(::rover_control::action::DriveToPose_Result & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Result_final_pose distance_traveled(::rover_control::action::DriveToPose_Result::_distance_traveled_type arg)
  {
    msg_.distance_traveled = std::move(arg);
    return Init_DriveToPose_Result_final_pose(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Result msg_;
};

class Init_DriveToPose_Result_execution_time
{
public:
  explicit Init_DriveToPose_Result_execution_time(::rover_control::action::DriveToPose_Result & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Result_distance_traveled execution_time(::rover_control::action::DriveToPose_Result::_execution_time_type arg)
  {
    msg_.execution_time = std::move(arg);
    return Init_DriveToPose_Result_distance_traveled(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Result msg_;
};

class Init_DriveToPose_Result_final_orientation_error
{
public:
  explicit Init_DriveToPose_Result_final_orientation_error(::rover_control::action::DriveToPose_Result & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Result_execution_time final_orientation_error(::rover_control::action::DriveToPose_Result::_final_orientation_error_type arg)
  {
    msg_.final_orientation_error = std::move(arg);
    return Init_DriveToPose_Result_execution_time(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Result msg_;
};

class Init_DriveToPose_Result_final_position_error
{
public:
  explicit Init_DriveToPose_Result_final_position_error(::rover_control::action::DriveToPose_Result & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Result_final_orientation_error final_position_error(::rover_control::action::DriveToPose_Result::_final_position_error_type arg)
  {
    msg_.final_position_error = std::move(arg);
    return Init_DriveToPose_Result_final_orientation_error(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Result msg_;
};

class Init_DriveToPose_Result_error_message
{
public:
  explicit Init_DriveToPose_Result_error_message(::rover_control::action::DriveToPose_Result & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Result_final_position_error error_message(::rover_control::action::DriveToPose_Result::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_DriveToPose_Result_final_position_error(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Result msg_;
};

class Init_DriveToPose_Result_result_code
{
public:
  explicit Init_DriveToPose_Result_result_code(::rover_control::action::DriveToPose_Result & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Result_error_message result_code(::rover_control::action::DriveToPose_Result::_result_code_type arg)
  {
    msg_.result_code = std::move(arg);
    return Init_DriveToPose_Result_error_message(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Result msg_;
};

class Init_DriveToPose_Result_success
{
public:
  Init_DriveToPose_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DriveToPose_Result_result_code success(::rover_control::action::DriveToPose_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_DriveToPose_Result_result_code(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::DriveToPose_Result>()
{
  return rover_control::action::builder::Init_DriveToPose_Result_success();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_DriveToPose_Feedback_autonomy_time_remaining
{
public:
  explicit Init_DriveToPose_Feedback_autonomy_time_remaining(::rover_control::action::DriveToPose_Feedback & msg)
  : msg_(msg)
  {}
  ::rover_control::action::DriveToPose_Feedback autonomy_time_remaining(::rover_control::action::DriveToPose_Feedback::_autonomy_time_remaining_type arg)
  {
    msg_.autonomy_time_remaining = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Feedback msg_;
};

class Init_DriveToPose_Feedback_stuck_detected
{
public:
  explicit Init_DriveToPose_Feedback_stuck_detected(::rover_control::action::DriveToPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Feedback_autonomy_time_remaining stuck_detected(::rover_control::action::DriveToPose_Feedback::_stuck_detected_type arg)
  {
    msg_.stuck_detected = std::move(arg);
    return Init_DriveToPose_Feedback_autonomy_time_remaining(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Feedback msg_;
};

class Init_DriveToPose_Feedback_status_message
{
public:
  explicit Init_DriveToPose_Feedback_status_message(::rover_control::action::DriveToPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Feedback_stuck_detected status_message(::rover_control::action::DriveToPose_Feedback::_status_message_type arg)
  {
    msg_.status_message = std::move(arg);
    return Init_DriveToPose_Feedback_stuck_detected(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Feedback msg_;
};

class Init_DriveToPose_Feedback_current_velocity
{
public:
  explicit Init_DriveToPose_Feedback_current_velocity(::rover_control::action::DriveToPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Feedback_status_message current_velocity(::rover_control::action::DriveToPose_Feedback::_current_velocity_type arg)
  {
    msg_.current_velocity = std::move(arg);
    return Init_DriveToPose_Feedback_status_message(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Feedback msg_;
};

class Init_DriveToPose_Feedback_distance_remaining
{
public:
  explicit Init_DriveToPose_Feedback_distance_remaining(::rover_control::action::DriveToPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Feedback_current_velocity distance_remaining(::rover_control::action::DriveToPose_Feedback::_distance_remaining_type arg)
  {
    msg_.distance_remaining = std::move(arg);
    return Init_DriveToPose_Feedback_current_velocity(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Feedback msg_;
};

class Init_DriveToPose_Feedback_estimated_time_remaining
{
public:
  explicit Init_DriveToPose_Feedback_estimated_time_remaining(::rover_control::action::DriveToPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Feedback_distance_remaining estimated_time_remaining(::rover_control::action::DriveToPose_Feedback::_estimated_time_remaining_type arg)
  {
    msg_.estimated_time_remaining = std::move(arg);
    return Init_DriveToPose_Feedback_distance_remaining(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Feedback msg_;
};

class Init_DriveToPose_Feedback_current_pose
{
public:
  explicit Init_DriveToPose_Feedback_current_pose(::rover_control::action::DriveToPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Feedback_estimated_time_remaining current_pose(::rover_control::action::DriveToPose_Feedback::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_DriveToPose_Feedback_estimated_time_remaining(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Feedback msg_;
};

class Init_DriveToPose_Feedback_progress_percentage
{
public:
  explicit Init_DriveToPose_Feedback_progress_percentage(::rover_control::action::DriveToPose_Feedback & msg)
  : msg_(msg)
  {}
  Init_DriveToPose_Feedback_current_pose progress_percentage(::rover_control::action::DriveToPose_Feedback::_progress_percentage_type arg)
  {
    msg_.progress_percentage = std::move(arg);
    return Init_DriveToPose_Feedback_current_pose(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Feedback msg_;
};

class Init_DriveToPose_Feedback_current_state
{
public:
  Init_DriveToPose_Feedback_current_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DriveToPose_Feedback_progress_percentage current_state(::rover_control::action::DriveToPose_Feedback::_current_state_type arg)
  {
    msg_.current_state = std::move(arg);
    return Init_DriveToPose_Feedback_progress_percentage(msg_);
  }

private:
  ::rover_control::action::DriveToPose_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::DriveToPose_Feedback>()
{
  return rover_control::action::builder::Init_DriveToPose_Feedback_current_state();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_DriveToPose_SendGoal_Request_goal
{
public:
  explicit Init_DriveToPose_SendGoal_Request_goal(::rover_control::action::DriveToPose_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::rover_control::action::DriveToPose_SendGoal_Request goal(::rover_control::action::DriveToPose_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::DriveToPose_SendGoal_Request msg_;
};

class Init_DriveToPose_SendGoal_Request_goal_id
{
public:
  Init_DriveToPose_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DriveToPose_SendGoal_Request_goal goal_id(::rover_control::action::DriveToPose_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_DriveToPose_SendGoal_Request_goal(msg_);
  }

private:
  ::rover_control::action::DriveToPose_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::DriveToPose_SendGoal_Request>()
{
  return rover_control::action::builder::Init_DriveToPose_SendGoal_Request_goal_id();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_DriveToPose_SendGoal_Response_stamp
{
public:
  explicit Init_DriveToPose_SendGoal_Response_stamp(::rover_control::action::DriveToPose_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::rover_control::action::DriveToPose_SendGoal_Response stamp(::rover_control::action::DriveToPose_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::DriveToPose_SendGoal_Response msg_;
};

class Init_DriveToPose_SendGoal_Response_accepted
{
public:
  Init_DriveToPose_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DriveToPose_SendGoal_Response_stamp accepted(::rover_control::action::DriveToPose_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_DriveToPose_SendGoal_Response_stamp(msg_);
  }

private:
  ::rover_control::action::DriveToPose_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::DriveToPose_SendGoal_Response>()
{
  return rover_control::action::builder::Init_DriveToPose_SendGoal_Response_accepted();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_DriveToPose_GetResult_Request_goal_id
{
public:
  Init_DriveToPose_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_control::action::DriveToPose_GetResult_Request goal_id(::rover_control::action::DriveToPose_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::DriveToPose_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::DriveToPose_GetResult_Request>()
{
  return rover_control::action::builder::Init_DriveToPose_GetResult_Request_goal_id();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_DriveToPose_GetResult_Response_result
{
public:
  explicit Init_DriveToPose_GetResult_Response_result(::rover_control::action::DriveToPose_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::rover_control::action::DriveToPose_GetResult_Response result(::rover_control::action::DriveToPose_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::DriveToPose_GetResult_Response msg_;
};

class Init_DriveToPose_GetResult_Response_status
{
public:
  Init_DriveToPose_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DriveToPose_GetResult_Response_result status(::rover_control::action::DriveToPose_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_DriveToPose_GetResult_Response_result(msg_);
  }

private:
  ::rover_control::action::DriveToPose_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::DriveToPose_GetResult_Response>()
{
  return rover_control::action::builder::Init_DriveToPose_GetResult_Response_status();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_DriveToPose_FeedbackMessage_feedback
{
public:
  explicit Init_DriveToPose_FeedbackMessage_feedback(::rover_control::action::DriveToPose_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::rover_control::action::DriveToPose_FeedbackMessage feedback(::rover_control::action::DriveToPose_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::DriveToPose_FeedbackMessage msg_;
};

class Init_DriveToPose_FeedbackMessage_goal_id
{
public:
  Init_DriveToPose_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DriveToPose_FeedbackMessage_feedback goal_id(::rover_control::action::DriveToPose_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_DriveToPose_FeedbackMessage_feedback(msg_);
  }

private:
  ::rover_control::action::DriveToPose_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::DriveToPose_FeedbackMessage>()
{
  return rover_control::action::builder::Init_DriveToPose_FeedbackMessage_goal_id();
}

}  // namespace rover_control

#endif  // ROVER_CONTROL__ACTION__DETAIL__DRIVE_TO_POSE__BUILDER_HPP_
