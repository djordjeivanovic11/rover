// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_control:action/FollowPath.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__ACTION__DETAIL__FOLLOW_PATH__BUILDER_HPP_
#define ROVER_CONTROL__ACTION__DETAIL__FOLLOW_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_control/action/detail/follow_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_FollowPath_Goal_use_obstacle_avoidance
{
public:
  explicit Init_FollowPath_Goal_use_obstacle_avoidance(::rover_control::action::FollowPath_Goal & msg)
  : msg_(msg)
  {}
  ::rover_control::action::FollowPath_Goal use_obstacle_avoidance(::rover_control::action::FollowPath_Goal::_use_obstacle_avoidance_type arg)
  {
    msg_.use_obstacle_avoidance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::FollowPath_Goal msg_;
};

class Init_FollowPath_Goal_lookahead_distance
{
public:
  explicit Init_FollowPath_Goal_lookahead_distance(::rover_control::action::FollowPath_Goal & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Goal_use_obstacle_avoidance lookahead_distance(::rover_control::action::FollowPath_Goal::_lookahead_distance_type arg)
  {
    msg_.lookahead_distance = std::move(arg);
    return Init_FollowPath_Goal_use_obstacle_avoidance(msg_);
  }

private:
  ::rover_control::action::FollowPath_Goal msg_;
};

class Init_FollowPath_Goal_skip_unreachable
{
public:
  explicit Init_FollowPath_Goal_skip_unreachable(::rover_control::action::FollowPath_Goal & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Goal_lookahead_distance skip_unreachable(::rover_control::action::FollowPath_Goal::_skip_unreachable_type arg)
  {
    msg_.skip_unreachable = std::move(arg);
    return Init_FollowPath_Goal_lookahead_distance(msg_);
  }

private:
  ::rover_control::action::FollowPath_Goal msg_;
};

class Init_FollowPath_Goal_mission_mode
{
public:
  explicit Init_FollowPath_Goal_mission_mode(::rover_control::action::FollowPath_Goal & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Goal_skip_unreachable mission_mode(::rover_control::action::FollowPath_Goal::_mission_mode_type arg)
  {
    msg_.mission_mode = std::move(arg);
    return Init_FollowPath_Goal_skip_unreachable(msg_);
  }

private:
  ::rover_control::action::FollowPath_Goal msg_;
};

class Init_FollowPath_Goal_timeout
{
public:
  explicit Init_FollowPath_Goal_timeout(::rover_control::action::FollowPath_Goal & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Goal_mission_mode timeout(::rover_control::action::FollowPath_Goal::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return Init_FollowPath_Goal_mission_mode(msg_);
  }

private:
  ::rover_control::action::FollowPath_Goal msg_;
};

class Init_FollowPath_Goal_reverse_allowed
{
public:
  explicit Init_FollowPath_Goal_reverse_allowed(::rover_control::action::FollowPath_Goal & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Goal_timeout reverse_allowed(::rover_control::action::FollowPath_Goal::_reverse_allowed_type arg)
  {
    msg_.reverse_allowed = std::move(arg);
    return Init_FollowPath_Goal_timeout(msg_);
  }

private:
  ::rover_control::action::FollowPath_Goal msg_;
};

class Init_FollowPath_Goal_waypoint_tolerance
{
public:
  explicit Init_FollowPath_Goal_waypoint_tolerance(::rover_control::action::FollowPath_Goal & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Goal_reverse_allowed waypoint_tolerance(::rover_control::action::FollowPath_Goal::_waypoint_tolerance_type arg)
  {
    msg_.waypoint_tolerance = std::move(arg);
    return Init_FollowPath_Goal_reverse_allowed(msg_);
  }

private:
  ::rover_control::action::FollowPath_Goal msg_;
};

class Init_FollowPath_Goal_max_velocity
{
public:
  explicit Init_FollowPath_Goal_max_velocity(::rover_control::action::FollowPath_Goal & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Goal_waypoint_tolerance max_velocity(::rover_control::action::FollowPath_Goal::_max_velocity_type arg)
  {
    msg_.max_velocity = std::move(arg);
    return Init_FollowPath_Goal_waypoint_tolerance(msg_);
  }

private:
  ::rover_control::action::FollowPath_Goal msg_;
};

class Init_FollowPath_Goal_path
{
public:
  Init_FollowPath_Goal_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_Goal_max_velocity path(::rover_control::action::FollowPath_Goal::_path_type arg)
  {
    msg_.path = std::move(arg);
    return Init_FollowPath_Goal_max_velocity(msg_);
  }

private:
  ::rover_control::action::FollowPath_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::FollowPath_Goal>()
{
  return rover_control::action::builder::Init_FollowPath_Goal_path();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_FollowPath_Result_average_velocity
{
public:
  explicit Init_FollowPath_Result_average_velocity(::rover_control::action::FollowPath_Result & msg)
  : msg_(msg)
  {}
  ::rover_control::action::FollowPath_Result average_velocity(::rover_control::action::FollowPath_Result::_average_velocity_type arg)
  {
    msg_.average_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::FollowPath_Result msg_;
};

class Init_FollowPath_Result_final_pose
{
public:
  explicit Init_FollowPath_Result_final_pose(::rover_control::action::FollowPath_Result & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Result_average_velocity final_pose(::rover_control::action::FollowPath_Result::_final_pose_type arg)
  {
    msg_.final_pose = std::move(arg);
    return Init_FollowPath_Result_average_velocity(msg_);
  }

private:
  ::rover_control::action::FollowPath_Result msg_;
};

class Init_FollowPath_Result_executed_path
{
public:
  explicit Init_FollowPath_Result_executed_path(::rover_control::action::FollowPath_Result & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Result_final_pose executed_path(::rover_control::action::FollowPath_Result::_executed_path_type arg)
  {
    msg_.executed_path = std::move(arg);
    return Init_FollowPath_Result_final_pose(msg_);
  }

private:
  ::rover_control::action::FollowPath_Result msg_;
};

class Init_FollowPath_Result_execution_time
{
public:
  explicit Init_FollowPath_Result_execution_time(::rover_control::action::FollowPath_Result & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Result_executed_path execution_time(::rover_control::action::FollowPath_Result::_execution_time_type arg)
  {
    msg_.execution_time = std::move(arg);
    return Init_FollowPath_Result_executed_path(msg_);
  }

private:
  ::rover_control::action::FollowPath_Result msg_;
};

class Init_FollowPath_Result_total_distance
{
public:
  explicit Init_FollowPath_Result_total_distance(::rover_control::action::FollowPath_Result & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Result_execution_time total_distance(::rover_control::action::FollowPath_Result::_total_distance_type arg)
  {
    msg_.total_distance = std::move(arg);
    return Init_FollowPath_Result_execution_time(msg_);
  }

private:
  ::rover_control::action::FollowPath_Result msg_;
};

class Init_FollowPath_Result_total_waypoints
{
public:
  explicit Init_FollowPath_Result_total_waypoints(::rover_control::action::FollowPath_Result & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Result_total_distance total_waypoints(::rover_control::action::FollowPath_Result::_total_waypoints_type arg)
  {
    msg_.total_waypoints = std::move(arg);
    return Init_FollowPath_Result_total_distance(msg_);
  }

private:
  ::rover_control::action::FollowPath_Result msg_;
};

class Init_FollowPath_Result_waypoints_completed
{
public:
  explicit Init_FollowPath_Result_waypoints_completed(::rover_control::action::FollowPath_Result & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Result_total_waypoints waypoints_completed(::rover_control::action::FollowPath_Result::_waypoints_completed_type arg)
  {
    msg_.waypoints_completed = std::move(arg);
    return Init_FollowPath_Result_total_waypoints(msg_);
  }

private:
  ::rover_control::action::FollowPath_Result msg_;
};

class Init_FollowPath_Result_error_message
{
public:
  explicit Init_FollowPath_Result_error_message(::rover_control::action::FollowPath_Result & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Result_waypoints_completed error_message(::rover_control::action::FollowPath_Result::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_FollowPath_Result_waypoints_completed(msg_);
  }

private:
  ::rover_control::action::FollowPath_Result msg_;
};

class Init_FollowPath_Result_result_code
{
public:
  explicit Init_FollowPath_Result_result_code(::rover_control::action::FollowPath_Result & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Result_error_message result_code(::rover_control::action::FollowPath_Result::_result_code_type arg)
  {
    msg_.result_code = std::move(arg);
    return Init_FollowPath_Result_error_message(msg_);
  }

private:
  ::rover_control::action::FollowPath_Result msg_;
};

class Init_FollowPath_Result_success
{
public:
  Init_FollowPath_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_Result_result_code success(::rover_control::action::FollowPath_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_FollowPath_Result_result_code(msg_);
  }

private:
  ::rover_control::action::FollowPath_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::FollowPath_Result>()
{
  return rover_control::action::builder::Init_FollowPath_Result_success();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_FollowPath_Feedback_target_waypoint
{
public:
  explicit Init_FollowPath_Feedback_target_waypoint(::rover_control::action::FollowPath_Feedback & msg)
  : msg_(msg)
  {}
  ::rover_control::action::FollowPath_Feedback target_waypoint(::rover_control::action::FollowPath_Feedback::_target_waypoint_type arg)
  {
    msg_.target_waypoint = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::FollowPath_Feedback msg_;
};

class Init_FollowPath_Feedback_autonomy_time_remaining
{
public:
  explicit Init_FollowPath_Feedback_autonomy_time_remaining(::rover_control::action::FollowPath_Feedback & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Feedback_target_waypoint autonomy_time_remaining(::rover_control::action::FollowPath_Feedback::_autonomy_time_remaining_type arg)
  {
    msg_.autonomy_time_remaining = std::move(arg);
    return Init_FollowPath_Feedback_target_waypoint(msg_);
  }

private:
  ::rover_control::action::FollowPath_Feedback msg_;
};

class Init_FollowPath_Feedback_stuck_detected
{
public:
  explicit Init_FollowPath_Feedback_stuck_detected(::rover_control::action::FollowPath_Feedback & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Feedback_autonomy_time_remaining stuck_detected(::rover_control::action::FollowPath_Feedback::_stuck_detected_type arg)
  {
    msg_.stuck_detected = std::move(arg);
    return Init_FollowPath_Feedback_autonomy_time_remaining(msg_);
  }

private:
  ::rover_control::action::FollowPath_Feedback msg_;
};

class Init_FollowPath_Feedback_status_message
{
public:
  explicit Init_FollowPath_Feedback_status_message(::rover_control::action::FollowPath_Feedback & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Feedback_stuck_detected status_message(::rover_control::action::FollowPath_Feedback::_status_message_type arg)
  {
    msg_.status_message = std::move(arg);
    return Init_FollowPath_Feedback_stuck_detected(msg_);
  }

private:
  ::rover_control::action::FollowPath_Feedback msg_;
};

class Init_FollowPath_Feedback_current_velocity
{
public:
  explicit Init_FollowPath_Feedback_current_velocity(::rover_control::action::FollowPath_Feedback & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Feedback_status_message current_velocity(::rover_control::action::FollowPath_Feedback::_current_velocity_type arg)
  {
    msg_.current_velocity = std::move(arg);
    return Init_FollowPath_Feedback_status_message(msg_);
  }

private:
  ::rover_control::action::FollowPath_Feedback msg_;
};

class Init_FollowPath_Feedback_distance_remaining
{
public:
  explicit Init_FollowPath_Feedback_distance_remaining(::rover_control::action::FollowPath_Feedback & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Feedback_current_velocity distance_remaining(::rover_control::action::FollowPath_Feedback::_distance_remaining_type arg)
  {
    msg_.distance_remaining = std::move(arg);
    return Init_FollowPath_Feedback_current_velocity(msg_);
  }

private:
  ::rover_control::action::FollowPath_Feedback msg_;
};

class Init_FollowPath_Feedback_estimated_time_remaining
{
public:
  explicit Init_FollowPath_Feedback_estimated_time_remaining(::rover_control::action::FollowPath_Feedback & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Feedback_distance_remaining estimated_time_remaining(::rover_control::action::FollowPath_Feedback::_estimated_time_remaining_type arg)
  {
    msg_.estimated_time_remaining = std::move(arg);
    return Init_FollowPath_Feedback_distance_remaining(msg_);
  }

private:
  ::rover_control::action::FollowPath_Feedback msg_;
};

class Init_FollowPath_Feedback_current_pose
{
public:
  explicit Init_FollowPath_Feedback_current_pose(::rover_control::action::FollowPath_Feedback & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Feedback_estimated_time_remaining current_pose(::rover_control::action::FollowPath_Feedback::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_FollowPath_Feedback_estimated_time_remaining(msg_);
  }

private:
  ::rover_control::action::FollowPath_Feedback msg_;
};

class Init_FollowPath_Feedback_current_waypoint
{
public:
  explicit Init_FollowPath_Feedback_current_waypoint(::rover_control::action::FollowPath_Feedback & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Feedback_current_pose current_waypoint(::rover_control::action::FollowPath_Feedback::_current_waypoint_type arg)
  {
    msg_.current_waypoint = std::move(arg);
    return Init_FollowPath_Feedback_current_pose(msg_);
  }

private:
  ::rover_control::action::FollowPath_Feedback msg_;
};

class Init_FollowPath_Feedback_progress_percentage
{
public:
  explicit Init_FollowPath_Feedback_progress_percentage(::rover_control::action::FollowPath_Feedback & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Feedback_current_waypoint progress_percentage(::rover_control::action::FollowPath_Feedback::_progress_percentage_type arg)
  {
    msg_.progress_percentage = std::move(arg);
    return Init_FollowPath_Feedback_current_waypoint(msg_);
  }

private:
  ::rover_control::action::FollowPath_Feedback msg_;
};

class Init_FollowPath_Feedback_current_state
{
public:
  Init_FollowPath_Feedback_current_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_Feedback_progress_percentage current_state(::rover_control::action::FollowPath_Feedback::_current_state_type arg)
  {
    msg_.current_state = std::move(arg);
    return Init_FollowPath_Feedback_progress_percentage(msg_);
  }

private:
  ::rover_control::action::FollowPath_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::FollowPath_Feedback>()
{
  return rover_control::action::builder::Init_FollowPath_Feedback_current_state();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_FollowPath_SendGoal_Request_goal
{
public:
  explicit Init_FollowPath_SendGoal_Request_goal(::rover_control::action::FollowPath_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::rover_control::action::FollowPath_SendGoal_Request goal(::rover_control::action::FollowPath_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::FollowPath_SendGoal_Request msg_;
};

class Init_FollowPath_SendGoal_Request_goal_id
{
public:
  Init_FollowPath_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_SendGoal_Request_goal goal_id(::rover_control::action::FollowPath_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FollowPath_SendGoal_Request_goal(msg_);
  }

private:
  ::rover_control::action::FollowPath_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::FollowPath_SendGoal_Request>()
{
  return rover_control::action::builder::Init_FollowPath_SendGoal_Request_goal_id();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_FollowPath_SendGoal_Response_stamp
{
public:
  explicit Init_FollowPath_SendGoal_Response_stamp(::rover_control::action::FollowPath_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::rover_control::action::FollowPath_SendGoal_Response stamp(::rover_control::action::FollowPath_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::FollowPath_SendGoal_Response msg_;
};

class Init_FollowPath_SendGoal_Response_accepted
{
public:
  Init_FollowPath_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_SendGoal_Response_stamp accepted(::rover_control::action::FollowPath_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_FollowPath_SendGoal_Response_stamp(msg_);
  }

private:
  ::rover_control::action::FollowPath_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::FollowPath_SendGoal_Response>()
{
  return rover_control::action::builder::Init_FollowPath_SendGoal_Response_accepted();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_FollowPath_GetResult_Request_goal_id
{
public:
  Init_FollowPath_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rover_control::action::FollowPath_GetResult_Request goal_id(::rover_control::action::FollowPath_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::FollowPath_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::FollowPath_GetResult_Request>()
{
  return rover_control::action::builder::Init_FollowPath_GetResult_Request_goal_id();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_FollowPath_GetResult_Response_result
{
public:
  explicit Init_FollowPath_GetResult_Response_result(::rover_control::action::FollowPath_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::rover_control::action::FollowPath_GetResult_Response result(::rover_control::action::FollowPath_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::FollowPath_GetResult_Response msg_;
};

class Init_FollowPath_GetResult_Response_status
{
public:
  Init_FollowPath_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_GetResult_Response_result status(::rover_control::action::FollowPath_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_FollowPath_GetResult_Response_result(msg_);
  }

private:
  ::rover_control::action::FollowPath_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::FollowPath_GetResult_Response>()
{
  return rover_control::action::builder::Init_FollowPath_GetResult_Response_status();
}

}  // namespace rover_control


namespace rover_control
{

namespace action
{

namespace builder
{

class Init_FollowPath_FeedbackMessage_feedback
{
public:
  explicit Init_FollowPath_FeedbackMessage_feedback(::rover_control::action::FollowPath_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::rover_control::action::FollowPath_FeedbackMessage feedback(::rover_control::action::FollowPath_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::action::FollowPath_FeedbackMessage msg_;
};

class Init_FollowPath_FeedbackMessage_goal_id
{
public:
  Init_FollowPath_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_FeedbackMessage_feedback goal_id(::rover_control::action::FollowPath_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FollowPath_FeedbackMessage_feedback(msg_);
  }

private:
  ::rover_control::action::FollowPath_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::action::FollowPath_FeedbackMessage>()
{
  return rover_control::action::builder::Init_FollowPath_FeedbackMessage_goal_id();
}

}  // namespace rover_control

#endif  // ROVER_CONTROL__ACTION__DETAIL__FOLLOW_PATH__BUILDER_HPP_
