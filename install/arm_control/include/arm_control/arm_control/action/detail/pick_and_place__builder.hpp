// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arm_control:action/PickAndPlace.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__ACTION__DETAIL__PICK_AND_PLACE__BUILDER_HPP_
#define ARM_CONTROL__ACTION__DETAIL__PICK_AND_PLACE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arm_control/action/detail/pick_and_place__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_PickAndPlace_Goal_acceleration_scaling
{
public:
  explicit Init_PickAndPlace_Goal_acceleration_scaling(::arm_control::action::PickAndPlace_Goal & msg)
  : msg_(msg)
  {}
  ::arm_control::action::PickAndPlace_Goal acceleration_scaling(::arm_control::action::PickAndPlace_Goal::_acceleration_scaling_type arg)
  {
    msg_.acceleration_scaling = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Goal msg_;
};

class Init_PickAndPlace_Goal_velocity_scaling
{
public:
  explicit Init_PickAndPlace_Goal_velocity_scaling(::arm_control::action::PickAndPlace_Goal & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Goal_acceleration_scaling velocity_scaling(::arm_control::action::PickAndPlace_Goal::_velocity_scaling_type arg)
  {
    msg_.velocity_scaling = std::move(arg);
    return Init_PickAndPlace_Goal_acceleration_scaling(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Goal msg_;
};

class Init_PickAndPlace_Goal_gentle_place
{
public:
  explicit Init_PickAndPlace_Goal_gentle_place(::arm_control::action::PickAndPlace_Goal & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Goal_velocity_scaling gentle_place(::arm_control::action::PickAndPlace_Goal::_gentle_place_type arg)
  {
    msg_.gentle_place = std::move(arg);
    return Init_PickAndPlace_Goal_velocity_scaling(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Goal msg_;
};

class Init_PickAndPlace_Goal_place_force
{
public:
  explicit Init_PickAndPlace_Goal_place_force(::arm_control::action::PickAndPlace_Goal & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Goal_gentle_place place_force(::arm_control::action::PickAndPlace_Goal::_place_force_type arg)
  {
    msg_.place_force = std::move(arg);
    return Init_PickAndPlace_Goal_gentle_place(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Goal msg_;
};

class Init_PickAndPlace_Goal_lift_height
{
public:
  explicit Init_PickAndPlace_Goal_lift_height(::arm_control::action::PickAndPlace_Goal & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Goal_place_force lift_height(::arm_control::action::PickAndPlace_Goal::_lift_height_type arg)
  {
    msg_.lift_height = std::move(arg);
    return Init_PickAndPlace_Goal_place_force(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Goal msg_;
};

class Init_PickAndPlace_Goal_verify_grasp
{
public:
  explicit Init_PickAndPlace_Goal_verify_grasp(::arm_control::action::PickAndPlace_Goal & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Goal_lift_height verify_grasp(::arm_control::action::PickAndPlace_Goal::_verify_grasp_type arg)
  {
    msg_.verify_grasp = std::move(arg);
    return Init_PickAndPlace_Goal_lift_height(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Goal msg_;
};

class Init_PickAndPlace_Goal_grasp_strategy
{
public:
  explicit Init_PickAndPlace_Goal_grasp_strategy(::arm_control::action::PickAndPlace_Goal & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Goal_verify_grasp grasp_strategy(::arm_control::action::PickAndPlace_Goal::_grasp_strategy_type arg)
  {
    msg_.grasp_strategy = std::move(arg);
    return Init_PickAndPlace_Goal_verify_grasp(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Goal msg_;
};

class Init_PickAndPlace_Goal_grasp_timeout
{
public:
  explicit Init_PickAndPlace_Goal_grasp_timeout(::arm_control::action::PickAndPlace_Goal & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Goal_grasp_strategy grasp_timeout(::arm_control::action::PickAndPlace_Goal::_grasp_timeout_type arg)
  {
    msg_.grasp_timeout = std::move(arg);
    return Init_PickAndPlace_Goal_grasp_strategy(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Goal msg_;
};

class Init_PickAndPlace_Goal_grasp_force
{
public:
  explicit Init_PickAndPlace_Goal_grasp_force(::arm_control::action::PickAndPlace_Goal & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Goal_grasp_timeout grasp_force(::arm_control::action::PickAndPlace_Goal::_grasp_force_type arg)
  {
    msg_.grasp_force = std::move(arg);
    return Init_PickAndPlace_Goal_grasp_timeout(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Goal msg_;
};

class Init_PickAndPlace_Goal_retreat_offset
{
public:
  explicit Init_PickAndPlace_Goal_retreat_offset(::arm_control::action::PickAndPlace_Goal & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Goal_grasp_force retreat_offset(::arm_control::action::PickAndPlace_Goal::_retreat_offset_type arg)
  {
    msg_.retreat_offset = std::move(arg);
    return Init_PickAndPlace_Goal_grasp_force(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Goal msg_;
};

class Init_PickAndPlace_Goal_approach_offset
{
public:
  explicit Init_PickAndPlace_Goal_approach_offset(::arm_control::action::PickAndPlace_Goal & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Goal_retreat_offset approach_offset(::arm_control::action::PickAndPlace_Goal::_approach_offset_type arg)
  {
    msg_.approach_offset = std::move(arg);
    return Init_PickAndPlace_Goal_retreat_offset(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Goal msg_;
};

class Init_PickAndPlace_Goal_place_pose
{
public:
  explicit Init_PickAndPlace_Goal_place_pose(::arm_control::action::PickAndPlace_Goal & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Goal_approach_offset place_pose(::arm_control::action::PickAndPlace_Goal::_place_pose_type arg)
  {
    msg_.place_pose = std::move(arg);
    return Init_PickAndPlace_Goal_approach_offset(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Goal msg_;
};

class Init_PickAndPlace_Goal_pick_pose
{
public:
  Init_PickAndPlace_Goal_pick_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickAndPlace_Goal_place_pose pick_pose(::arm_control::action::PickAndPlace_Goal::_pick_pose_type arg)
  {
    msg_.pick_pose = std::move(arg);
    return Init_PickAndPlace_Goal_place_pose(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::PickAndPlace_Goal>()
{
  return arm_control::action::builder::Init_PickAndPlace_Goal_pick_pose();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_PickAndPlace_Result_final_joint_state
{
public:
  explicit Init_PickAndPlace_Result_final_joint_state(::arm_control::action::PickAndPlace_Result & msg)
  : msg_(msg)
  {}
  ::arm_control::action::PickAndPlace_Result final_joint_state(::arm_control::action::PickAndPlace_Result::_final_joint_state_type arg)
  {
    msg_.final_joint_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Result msg_;
};

class Init_PickAndPlace_Result_total_execution_time
{
public:
  explicit Init_PickAndPlace_Result_total_execution_time(::arm_control::action::PickAndPlace_Result & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Result_final_joint_state total_execution_time(::arm_control::action::PickAndPlace_Result::_total_execution_time_type arg)
  {
    msg_.total_execution_time = std::move(arg);
    return Init_PickAndPlace_Result_final_joint_state(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Result msg_;
};

class Init_PickAndPlace_Result_final_object_pose
{
public:
  explicit Init_PickAndPlace_Result_final_object_pose(::arm_control::action::PickAndPlace_Result & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Result_total_execution_time final_object_pose(::arm_control::action::PickAndPlace_Result::_final_object_pose_type arg)
  {
    msg_.final_object_pose = std::move(arg);
    return Init_PickAndPlace_Result_total_execution_time(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Result msg_;
};

class Init_PickAndPlace_Result_grasp_force_achieved
{
public:
  explicit Init_PickAndPlace_Result_grasp_force_achieved(::arm_control::action::PickAndPlace_Result & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Result_final_object_pose grasp_force_achieved(::arm_control::action::PickAndPlace_Result::_grasp_force_achieved_type arg)
  {
    msg_.grasp_force_achieved = std::move(arg);
    return Init_PickAndPlace_Result_final_object_pose(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Result msg_;
};

class Init_PickAndPlace_Result_place_successful
{
public:
  explicit Init_PickAndPlace_Result_place_successful(::arm_control::action::PickAndPlace_Result & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Result_grasp_force_achieved place_successful(::arm_control::action::PickAndPlace_Result::_place_successful_type arg)
  {
    msg_.place_successful = std::move(arg);
    return Init_PickAndPlace_Result_grasp_force_achieved(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Result msg_;
};

class Init_PickAndPlace_Result_grasp_successful
{
public:
  explicit Init_PickAndPlace_Result_grasp_successful(::arm_control::action::PickAndPlace_Result & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Result_place_successful grasp_successful(::arm_control::action::PickAndPlace_Result::_grasp_successful_type arg)
  {
    msg_.grasp_successful = std::move(arg);
    return Init_PickAndPlace_Result_place_successful(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Result msg_;
};

class Init_PickAndPlace_Result_failure_phase
{
public:
  explicit Init_PickAndPlace_Result_failure_phase(::arm_control::action::PickAndPlace_Result & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Result_grasp_successful failure_phase(::arm_control::action::PickAndPlace_Result::_failure_phase_type arg)
  {
    msg_.failure_phase = std::move(arg);
    return Init_PickAndPlace_Result_grasp_successful(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Result msg_;
};

class Init_PickAndPlace_Result_error_code
{
public:
  explicit Init_PickAndPlace_Result_error_code(::arm_control::action::PickAndPlace_Result & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Result_failure_phase error_code(::arm_control::action::PickAndPlace_Result::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_PickAndPlace_Result_failure_phase(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Result msg_;
};

class Init_PickAndPlace_Result_error_message
{
public:
  explicit Init_PickAndPlace_Result_error_message(::arm_control::action::PickAndPlace_Result & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Result_error_code error_message(::arm_control::action::PickAndPlace_Result::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return Init_PickAndPlace_Result_error_code(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Result msg_;
};

class Init_PickAndPlace_Result_success
{
public:
  Init_PickAndPlace_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickAndPlace_Result_error_message success(::arm_control::action::PickAndPlace_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PickAndPlace_Result_error_message(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::PickAndPlace_Result>()
{
  return arm_control::action::builder::Init_PickAndPlace_Result_success();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_PickAndPlace_Feedback_estimated_time_remaining
{
public:
  explicit Init_PickAndPlace_Feedback_estimated_time_remaining(::arm_control::action::PickAndPlace_Feedback & msg)
  : msg_(msg)
  {}
  ::arm_control::action::PickAndPlace_Feedback estimated_time_remaining(::arm_control::action::PickAndPlace_Feedback::_estimated_time_remaining_type arg)
  {
    msg_.estimated_time_remaining = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Feedback msg_;
};

class Init_PickAndPlace_Feedback_status_message
{
public:
  explicit Init_PickAndPlace_Feedback_status_message(::arm_control::action::PickAndPlace_Feedback & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Feedback_estimated_time_remaining status_message(::arm_control::action::PickAndPlace_Feedback::_status_message_type arg)
  {
    msg_.status_message = std::move(arg);
    return Init_PickAndPlace_Feedback_estimated_time_remaining(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Feedback msg_;
};

class Init_PickAndPlace_Feedback_grasp_status
{
public:
  explicit Init_PickAndPlace_Feedback_grasp_status(::arm_control::action::PickAndPlace_Feedback & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Feedback_status_message grasp_status(::arm_control::action::PickAndPlace_Feedback::_grasp_status_type arg)
  {
    msg_.grasp_status = std::move(arg);
    return Init_PickAndPlace_Feedback_status_message(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Feedback msg_;
};

class Init_PickAndPlace_Feedback_object_detected
{
public:
  explicit Init_PickAndPlace_Feedback_object_detected(::arm_control::action::PickAndPlace_Feedback & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Feedback_grasp_status object_detected(::arm_control::action::PickAndPlace_Feedback::_object_detected_type arg)
  {
    msg_.object_detected = std::move(arg);
    return Init_PickAndPlace_Feedback_grasp_status(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Feedback msg_;
};

class Init_PickAndPlace_Feedback_current_grasp_force
{
public:
  explicit Init_PickAndPlace_Feedback_current_grasp_force(::arm_control::action::PickAndPlace_Feedback & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Feedback_object_detected current_grasp_force(::arm_control::action::PickAndPlace_Feedback::_current_grasp_force_type arg)
  {
    msg_.current_grasp_force = std::move(arg);
    return Init_PickAndPlace_Feedback_object_detected(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Feedback msg_;
};

class Init_PickAndPlace_Feedback_current_pose
{
public:
  explicit Init_PickAndPlace_Feedback_current_pose(::arm_control::action::PickAndPlace_Feedback & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Feedback_current_grasp_force current_pose(::arm_control::action::PickAndPlace_Feedback::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_PickAndPlace_Feedback_current_grasp_force(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Feedback msg_;
};

class Init_PickAndPlace_Feedback_overall_progress
{
public:
  explicit Init_PickAndPlace_Feedback_overall_progress(::arm_control::action::PickAndPlace_Feedback & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Feedback_current_pose overall_progress(::arm_control::action::PickAndPlace_Feedback::_overall_progress_type arg)
  {
    msg_.overall_progress = std::move(arg);
    return Init_PickAndPlace_Feedback_current_pose(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Feedback msg_;
};

class Init_PickAndPlace_Feedback_phase_progress
{
public:
  explicit Init_PickAndPlace_Feedback_phase_progress(::arm_control::action::PickAndPlace_Feedback & msg)
  : msg_(msg)
  {}
  Init_PickAndPlace_Feedback_overall_progress phase_progress(::arm_control::action::PickAndPlace_Feedback::_phase_progress_type arg)
  {
    msg_.phase_progress = std::move(arg);
    return Init_PickAndPlace_Feedback_overall_progress(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Feedback msg_;
};

class Init_PickAndPlace_Feedback_current_phase
{
public:
  Init_PickAndPlace_Feedback_current_phase()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickAndPlace_Feedback_phase_progress current_phase(::arm_control::action::PickAndPlace_Feedback::_current_phase_type arg)
  {
    msg_.current_phase = std::move(arg);
    return Init_PickAndPlace_Feedback_phase_progress(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::PickAndPlace_Feedback>()
{
  return arm_control::action::builder::Init_PickAndPlace_Feedback_current_phase();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_PickAndPlace_SendGoal_Request_goal
{
public:
  explicit Init_PickAndPlace_SendGoal_Request_goal(::arm_control::action::PickAndPlace_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::arm_control::action::PickAndPlace_SendGoal_Request goal(::arm_control::action::PickAndPlace_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_SendGoal_Request msg_;
};

class Init_PickAndPlace_SendGoal_Request_goal_id
{
public:
  Init_PickAndPlace_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickAndPlace_SendGoal_Request_goal goal_id(::arm_control::action::PickAndPlace_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PickAndPlace_SendGoal_Request_goal(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::PickAndPlace_SendGoal_Request>()
{
  return arm_control::action::builder::Init_PickAndPlace_SendGoal_Request_goal_id();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_PickAndPlace_SendGoal_Response_stamp
{
public:
  explicit Init_PickAndPlace_SendGoal_Response_stamp(::arm_control::action::PickAndPlace_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::arm_control::action::PickAndPlace_SendGoal_Response stamp(::arm_control::action::PickAndPlace_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_SendGoal_Response msg_;
};

class Init_PickAndPlace_SendGoal_Response_accepted
{
public:
  Init_PickAndPlace_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickAndPlace_SendGoal_Response_stamp accepted(::arm_control::action::PickAndPlace_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_PickAndPlace_SendGoal_Response_stamp(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::PickAndPlace_SendGoal_Response>()
{
  return arm_control::action::builder::Init_PickAndPlace_SendGoal_Response_accepted();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_PickAndPlace_GetResult_Request_goal_id
{
public:
  Init_PickAndPlace_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::arm_control::action::PickAndPlace_GetResult_Request goal_id(::arm_control::action::PickAndPlace_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::PickAndPlace_GetResult_Request>()
{
  return arm_control::action::builder::Init_PickAndPlace_GetResult_Request_goal_id();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_PickAndPlace_GetResult_Response_result
{
public:
  explicit Init_PickAndPlace_GetResult_Response_result(::arm_control::action::PickAndPlace_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::arm_control::action::PickAndPlace_GetResult_Response result(::arm_control::action::PickAndPlace_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_GetResult_Response msg_;
};

class Init_PickAndPlace_GetResult_Response_status
{
public:
  Init_PickAndPlace_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickAndPlace_GetResult_Response_result status(::arm_control::action::PickAndPlace_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PickAndPlace_GetResult_Response_result(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::PickAndPlace_GetResult_Response>()
{
  return arm_control::action::builder::Init_PickAndPlace_GetResult_Response_status();
}

}  // namespace arm_control


namespace arm_control
{

namespace action
{

namespace builder
{

class Init_PickAndPlace_FeedbackMessage_feedback
{
public:
  explicit Init_PickAndPlace_FeedbackMessage_feedback(::arm_control::action::PickAndPlace_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::arm_control::action::PickAndPlace_FeedbackMessage feedback(::arm_control::action::PickAndPlace_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_FeedbackMessage msg_;
};

class Init_PickAndPlace_FeedbackMessage_goal_id
{
public:
  Init_PickAndPlace_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PickAndPlace_FeedbackMessage_feedback goal_id(::arm_control::action::PickAndPlace_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PickAndPlace_FeedbackMessage_feedback(msg_);
  }

private:
  ::arm_control::action::PickAndPlace_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::arm_control::action::PickAndPlace_FeedbackMessage>()
{
  return arm_control::action::builder::Init_PickAndPlace_FeedbackMessage_goal_id();
}

}  // namespace arm_control

#endif  // ARM_CONTROL__ACTION__DETAIL__PICK_AND_PLACE__BUILDER_HPP_
