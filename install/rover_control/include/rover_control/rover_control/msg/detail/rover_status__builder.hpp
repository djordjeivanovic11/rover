// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_control:msg/RoverStatus.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__MSG__DETAIL__ROVER_STATUS__BUILDER_HPP_
#define ROVER_CONTROL__MSG__DETAIL__ROVER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_control/msg/detail/rover_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_control
{

namespace msg
{

namespace builder
{

class Init_RoverStatus_sun_direction
{
public:
  explicit Init_RoverStatus_sun_direction(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  ::rover_control::msg::RoverStatus sun_direction(::rover_control::msg::RoverStatus::_sun_direction_type arg)
  {
    msg_.sun_direction = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_tennis_balls_detected
{
public:
  explicit Init_RoverStatus_tennis_balls_detected(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_sun_direction tennis_balls_detected(::rover_control::msg::RoverStatus::_tennis_balls_detected_type arg)
  {
    msg_.tennis_balls_detected = std::move(arg);
    return Init_RoverStatus_sun_direction(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_visual_servoing_active
{
public:
  explicit Init_RoverStatus_visual_servoing_active(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_tennis_balls_detected visual_servoing_active(::rover_control::msg::RoverStatus::_visual_servoing_active_type arg)
  {
    msg_.visual_servoing_active = std::move(arg);
    return Init_RoverStatus_tennis_balls_detected(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_judge_score_estimate
{
public:
  explicit Init_RoverStatus_judge_score_estimate(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_visual_servoing_active judge_score_estimate(::rover_control::msg::RoverStatus::_judge_score_estimate_type arg)
  {
    msg_.judge_score_estimate = std::move(arg);
    return Init_RoverStatus_visual_servoing_active(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_competition_mode
{
public:
  explicit Init_RoverStatus_competition_mode(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_judge_score_estimate competition_mode(::rover_control::msg::RoverStatus::_competition_mode_type arg)
  {
    msg_.competition_mode = std::move(arg);
    return Init_RoverStatus_judge_score_estimate(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_angular_velocity
{
public:
  explicit Init_RoverStatus_angular_velocity(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_competition_mode angular_velocity(::rover_control::msg::RoverStatus::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_RoverStatus_competition_mode(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_acceleration
{
public:
  explicit Init_RoverStatus_acceleration(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_angular_velocity acceleration(::rover_control::msg::RoverStatus::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_RoverStatus_angular_velocity(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_humidity
{
public:
  explicit Init_RoverStatus_humidity(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_acceleration humidity(::rover_control::msg::RoverStatus::_humidity_type arg)
  {
    msg_.humidity = std::move(arg);
    return Init_RoverStatus_acceleration(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_internal_temperature
{
public:
  explicit Init_RoverStatus_internal_temperature(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_humidity internal_temperature(::rover_control::msg::RoverStatus::_internal_temperature_type arg)
  {
    msg_.internal_temperature = std::move(arg);
    return Init_RoverStatus_humidity(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_ambient_temperature
{
public:
  explicit Init_RoverStatus_ambient_temperature(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_internal_temperature ambient_temperature(::rover_control::msg::RoverStatus::_ambient_temperature_type arg)
  {
    msg_.ambient_temperature = std::move(arg);
    return Init_RoverStatus_internal_temperature(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_collision_detected
{
public:
  explicit Init_RoverStatus_collision_detected(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_ambient_temperature collision_detected(::rover_control::msg::RoverStatus::_collision_detected_type arg)
  {
    msg_.collision_detected = std::move(arg);
    return Init_RoverStatus_ambient_temperature(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_slip_ratio
{
public:
  explicit Init_RoverStatus_slip_ratio(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_collision_detected slip_ratio(::rover_control::msg::RoverStatus::_slip_ratio_type arg)
  {
    msg_.slip_ratio = std::move(arg);
    return Init_RoverStatus_collision_detected(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_stuck_detected
{
public:
  explicit Init_RoverStatus_stuck_detected(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_slip_ratio stuck_detected(::rover_control::msg::RoverStatus::_stuck_detected_type arg)
  {
    msg_.stuck_detected = std::move(arg);
    return Init_RoverStatus_slip_ratio(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_fault_messages
{
public:
  explicit Init_RoverStatus_fault_messages(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_stuck_detected fault_messages(::rover_control::msg::RoverStatus::_fault_messages_type arg)
  {
    msg_.fault_messages = std::move(arg);
    return Init_RoverStatus_stuck_detected(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_active_faults
{
public:
  explicit Init_RoverStatus_active_faults(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_fault_messages active_faults(::rover_control::msg::RoverStatus::_active_faults_type arg)
  {
    msg_.active_faults = std::move(arg);
    return Init_RoverStatus_fault_messages(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_memory_usage
{
public:
  explicit Init_RoverStatus_memory_usage(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_active_faults memory_usage(::rover_control::msg::RoverStatus::_memory_usage_type arg)
  {
    msg_.memory_usage = std::move(arg);
    return Init_RoverStatus_active_faults(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_cpu_usage
{
public:
  explicit Init_RoverStatus_cpu_usage(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_memory_usage cpu_usage(::rover_control::msg::RoverStatus::_cpu_usage_type arg)
  {
    msg_.cpu_usage = std::move(arg);
    return Init_RoverStatus_memory_usage(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_bandwidth_usage
{
public:
  explicit Init_RoverStatus_bandwidth_usage(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_cpu_usage bandwidth_usage(::rover_control::msg::RoverStatus::_bandwidth_usage_type arg)
  {
    msg_.bandwidth_usage = std::move(arg);
    return Init_RoverStatus_cpu_usage(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_communication_quality
{
public:
  explicit Init_RoverStatus_communication_quality(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_bandwidth_usage communication_quality(::rover_control::msg::RoverStatus::_communication_quality_type arg)
  {
    msg_.communication_quality = std::move(arg);
    return Init_RoverStatus_bandwidth_usage(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_motor_temperatures
{
public:
  explicit Init_RoverStatus_motor_temperatures(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_communication_quality motor_temperatures(::rover_control::msg::RoverStatus::_motor_temperatures_type arg)
  {
    msg_.motor_temperatures = std::move(arg);
    return Init_RoverStatus_communication_quality(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_motor_currents
{
public:
  explicit Init_RoverStatus_motor_currents(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_motor_temperatures motor_currents(::rover_control::msg::RoverStatus::_motor_currents_type arg)
  {
    msg_.motor_currents = std::move(arg);
    return Init_RoverStatus_motor_temperatures(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_battery_temperature
{
public:
  explicit Init_RoverStatus_battery_temperature(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_motor_currents battery_temperature(::rover_control::msg::RoverStatus::_battery_temperature_type arg)
  {
    msg_.battery_temperature = std::move(arg);
    return Init_RoverStatus_motor_currents(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_battery_soc
{
public:
  explicit Init_RoverStatus_battery_soc(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_battery_temperature battery_soc(::rover_control::msg::RoverStatus::_battery_soc_type arg)
  {
    msg_.battery_soc = std::move(arg);
    return Init_RoverStatus_battery_temperature(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_battery_current
{
public:
  explicit Init_RoverStatus_battery_current(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_battery_soc battery_current(::rover_control::msg::RoverStatus::_battery_current_type arg)
  {
    msg_.battery_current = std::move(arg);
    return Init_RoverStatus_battery_soc(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_battery_voltage
{
public:
  explicit Init_RoverStatus_battery_voltage(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_battery_current battery_voltage(::rover_control::msg::RoverStatus::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_RoverStatus_battery_current(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_total_waypoints
{
public:
  explicit Init_RoverStatus_total_waypoints(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_battery_voltage total_waypoints(::rover_control::msg::RoverStatus::_total_waypoints_type arg)
  {
    msg_.total_waypoints = std::move(arg);
    return Init_RoverStatus_battery_voltage(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_waypoints_completed
{
public:
  explicit Init_RoverStatus_waypoints_completed(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_total_waypoints waypoints_completed(::rover_control::msg::RoverStatus::_waypoints_completed_type arg)
  {
    msg_.waypoints_completed = std::move(arg);
    return Init_RoverStatus_total_waypoints(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_autonomy_time_remaining
{
public:
  explicit Init_RoverStatus_autonomy_time_remaining(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_waypoints_completed autonomy_time_remaining(::rover_control::msg::RoverStatus::_autonomy_time_remaining_type arg)
  {
    msg_.autonomy_time_remaining = std::move(arg);
    return Init_RoverStatus_waypoints_completed(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_mission_progress_percentage
{
public:
  explicit Init_RoverStatus_mission_progress_percentage(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_autonomy_time_remaining mission_progress_percentage(::rover_control::msg::RoverStatus::_mission_progress_percentage_type arg)
  {
    msg_.mission_progress_percentage = std::move(arg);
    return Init_RoverStatus_autonomy_time_remaining(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_odometry
{
public:
  explicit Init_RoverStatus_odometry(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_mission_progress_percentage odometry(::rover_control::msg::RoverStatus::_odometry_type arg)
  {
    msg_.odometry = std::move(arg);
    return Init_RoverStatus_mission_progress_percentage(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_current_velocity
{
public:
  explicit Init_RoverStatus_current_velocity(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_odometry current_velocity(::rover_control::msg::RoverStatus::_current_velocity_type arg)
  {
    msg_.current_velocity = std::move(arg);
    return Init_RoverStatus_odometry(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_current_pose
{
public:
  explicit Init_RoverStatus_current_pose(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_current_velocity current_pose(::rover_control::msg::RoverStatus::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_RoverStatus_current_velocity(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_emergency_stop_active
{
public:
  explicit Init_RoverStatus_emergency_stop_active(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_current_pose emergency_stop_active(::rover_control::msg::RoverStatus::_emergency_stop_active_type arg)
  {
    msg_.emergency_stop_active = std::move(arg);
    return Init_RoverStatus_current_pose(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_safety_ok
{
public:
  explicit Init_RoverStatus_safety_ok(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_emergency_stop_active safety_ok(::rover_control::msg::RoverStatus::_safety_ok_type arg)
  {
    msg_.safety_ok = std::move(arg);
    return Init_RoverStatus_emergency_stop_active(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_is_autonomous
{
public:
  explicit Init_RoverStatus_is_autonomous(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_safety_ok is_autonomous(::rover_control::msg::RoverStatus::_is_autonomous_type arg)
  {
    msg_.is_autonomous = std::move(arg);
    return Init_RoverStatus_safety_ok(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_mission_mode
{
public:
  explicit Init_RoverStatus_mission_mode(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_is_autonomous mission_mode(::rover_control::msg::RoverStatus::_mission_mode_type arg)
  {
    msg_.mission_mode = std::move(arg);
    return Init_RoverStatus_is_autonomous(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_current_action
{
public:
  explicit Init_RoverStatus_current_action(::rover_control::msg::RoverStatus & msg)
  : msg_(msg)
  {}
  Init_RoverStatus_mission_mode current_action(::rover_control::msg::RoverStatus::_current_action_type arg)
  {
    msg_.current_action = std::move(arg);
    return Init_RoverStatus_mission_mode(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

class Init_RoverStatus_header
{
public:
  Init_RoverStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoverStatus_current_action header(::rover_control::msg::RoverStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RoverStatus_current_action(msg_);
  }

private:
  ::rover_control::msg::RoverStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_control::msg::RoverStatus>()
{
  return rover_control::msg::builder::Init_RoverStatus_header();
}

}  // namespace rover_control

#endif  // ROVER_CONTROL__MSG__DETAIL__ROVER_STATUS__BUILDER_HPP_
