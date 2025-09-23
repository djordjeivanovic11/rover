// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_control:msg/RoverStatus.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__MSG__DETAIL__ROVER_STATUS__TRAITS_HPP_
#define ROVER_CONTROL__MSG__DETAIL__ROVER_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_control/msg/detail/rover_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'current_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'current_velocity'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'odometry'
#include "nav_msgs/msg/detail/odometry__traits.hpp"
// Member 'acceleration'
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'sun_direction'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace rover_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const RoverStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: current_action
  {
    out << "current_action: ";
    rosidl_generator_traits::value_to_yaml(msg.current_action, out);
    out << ", ";
  }

  // member: mission_mode
  {
    out << "mission_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_mode, out);
    out << ", ";
  }

  // member: is_autonomous
  {
    out << "is_autonomous: ";
    rosidl_generator_traits::value_to_yaml(msg.is_autonomous, out);
    out << ", ";
  }

  // member: safety_ok
  {
    out << "safety_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_ok, out);
    out << ", ";
  }

  // member: emergency_stop_active
  {
    out << "emergency_stop_active: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop_active, out);
    out << ", ";
  }

  // member: current_pose
  {
    out << "current_pose: ";
    to_flow_style_yaml(msg.current_pose, out);
    out << ", ";
  }

  // member: current_velocity
  {
    out << "current_velocity: ";
    to_flow_style_yaml(msg.current_velocity, out);
    out << ", ";
  }

  // member: odometry
  {
    out << "odometry: ";
    to_flow_style_yaml(msg.odometry, out);
    out << ", ";
  }

  // member: mission_progress_percentage
  {
    out << "mission_progress_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_progress_percentage, out);
    out << ", ";
  }

  // member: autonomy_time_remaining
  {
    out << "autonomy_time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.autonomy_time_remaining, out);
    out << ", ";
  }

  // member: waypoints_completed
  {
    out << "waypoints_completed: ";
    rosidl_generator_traits::value_to_yaml(msg.waypoints_completed, out);
    out << ", ";
  }

  // member: total_waypoints
  {
    out << "total_waypoints: ";
    rosidl_generator_traits::value_to_yaml(msg.total_waypoints, out);
    out << ", ";
  }

  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: battery_current
  {
    out << "battery_current: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_current, out);
    out << ", ";
  }

  // member: battery_soc
  {
    out << "battery_soc: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_soc, out);
    out << ", ";
  }

  // member: battery_temperature
  {
    out << "battery_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_temperature, out);
    out << ", ";
  }

  // member: motor_currents
  {
    if (msg.motor_currents.size() == 0) {
      out << "motor_currents: []";
    } else {
      out << "motor_currents: [";
      size_t pending_items = msg.motor_currents.size();
      for (auto item : msg.motor_currents) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: motor_temperatures
  {
    if (msg.motor_temperatures.size() == 0) {
      out << "motor_temperatures: []";
    } else {
      out << "motor_temperatures: [";
      size_t pending_items = msg.motor_temperatures.size();
      for (auto item : msg.motor_temperatures) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: communication_quality
  {
    out << "communication_quality: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_quality, out);
    out << ", ";
  }

  // member: bandwidth_usage
  {
    out << "bandwidth_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.bandwidth_usage, out);
    out << ", ";
  }

  // member: cpu_usage
  {
    out << "cpu_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.cpu_usage, out);
    out << ", ";
  }

  // member: memory_usage
  {
    out << "memory_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.memory_usage, out);
    out << ", ";
  }

  // member: active_faults
  {
    if (msg.active_faults.size() == 0) {
      out << "active_faults: []";
    } else {
      out << "active_faults: [";
      size_t pending_items = msg.active_faults.size();
      for (auto item : msg.active_faults) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: fault_messages
  {
    if (msg.fault_messages.size() == 0) {
      out << "fault_messages: []";
    } else {
      out << "fault_messages: [";
      size_t pending_items = msg.fault_messages.size();
      for (auto item : msg.fault_messages) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: stuck_detected
  {
    out << "stuck_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.stuck_detected, out);
    out << ", ";
  }

  // member: slip_ratio
  {
    out << "slip_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.slip_ratio, out);
    out << ", ";
  }

  // member: collision_detected
  {
    out << "collision_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.collision_detected, out);
    out << ", ";
  }

  // member: ambient_temperature
  {
    out << "ambient_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.ambient_temperature, out);
    out << ", ";
  }

  // member: internal_temperature
  {
    out << "internal_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.internal_temperature, out);
    out << ", ";
  }

  // member: humidity
  {
    out << "humidity: ";
    rosidl_generator_traits::value_to_yaml(msg.humidity, out);
    out << ", ";
  }

  // member: acceleration
  {
    out << "acceleration: ";
    to_flow_style_yaml(msg.acceleration, out);
    out << ", ";
  }

  // member: angular_velocity
  {
    out << "angular_velocity: ";
    to_flow_style_yaml(msg.angular_velocity, out);
    out << ", ";
  }

  // member: competition_mode
  {
    out << "competition_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.competition_mode, out);
    out << ", ";
  }

  // member: judge_score_estimate
  {
    out << "judge_score_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.judge_score_estimate, out);
    out << ", ";
  }

  // member: visual_servoing_active
  {
    out << "visual_servoing_active: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_servoing_active, out);
    out << ", ";
  }

  // member: tennis_balls_detected
  {
    out << "tennis_balls_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.tennis_balls_detected, out);
    out << ", ";
  }

  // member: sun_direction
  {
    out << "sun_direction: ";
    to_flow_style_yaml(msg.sun_direction, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RoverStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: current_action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_action: ";
    rosidl_generator_traits::value_to_yaml(msg.current_action, out);
    out << "\n";
  }

  // member: mission_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_mode, out);
    out << "\n";
  }

  // member: is_autonomous
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_autonomous: ";
    rosidl_generator_traits::value_to_yaml(msg.is_autonomous, out);
    out << "\n";
  }

  // member: safety_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_ok, out);
    out << "\n";
  }

  // member: emergency_stop_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "emergency_stop_active: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop_active, out);
    out << "\n";
  }

  // member: current_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_pose:\n";
    to_block_style_yaml(msg.current_pose, out, indentation + 2);
  }

  // member: current_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_velocity:\n";
    to_block_style_yaml(msg.current_velocity, out, indentation + 2);
  }

  // member: odometry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "odometry:\n";
    to_block_style_yaml(msg.odometry, out, indentation + 2);
  }

  // member: mission_progress_percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_progress_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_progress_percentage, out);
    out << "\n";
  }

  // member: autonomy_time_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "autonomy_time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.autonomy_time_remaining, out);
    out << "\n";
  }

  // member: waypoints_completed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "waypoints_completed: ";
    rosidl_generator_traits::value_to_yaml(msg.waypoints_completed, out);
    out << "\n";
  }

  // member: total_waypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_waypoints: ";
    rosidl_generator_traits::value_to_yaml(msg.total_waypoints, out);
    out << "\n";
  }

  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: battery_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_current: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_current, out);
    out << "\n";
  }

  // member: battery_soc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_soc: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_soc, out);
    out << "\n";
  }

  // member: battery_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_temperature, out);
    out << "\n";
  }

  // member: motor_currents
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.motor_currents.size() == 0) {
      out << "motor_currents: []\n";
    } else {
      out << "motor_currents:\n";
      for (auto item : msg.motor_currents) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: motor_temperatures
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.motor_temperatures.size() == 0) {
      out << "motor_temperatures: []\n";
    } else {
      out << "motor_temperatures:\n";
      for (auto item : msg.motor_temperatures) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: communication_quality
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "communication_quality: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_quality, out);
    out << "\n";
  }

  // member: bandwidth_usage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bandwidth_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.bandwidth_usage, out);
    out << "\n";
  }

  // member: cpu_usage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cpu_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.cpu_usage, out);
    out << "\n";
  }

  // member: memory_usage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "memory_usage: ";
    rosidl_generator_traits::value_to_yaml(msg.memory_usage, out);
    out << "\n";
  }

  // member: active_faults
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.active_faults.size() == 0) {
      out << "active_faults: []\n";
    } else {
      out << "active_faults:\n";
      for (auto item : msg.active_faults) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: fault_messages
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.fault_messages.size() == 0) {
      out << "fault_messages: []\n";
    } else {
      out << "fault_messages:\n";
      for (auto item : msg.fault_messages) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: stuck_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stuck_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.stuck_detected, out);
    out << "\n";
  }

  // member: slip_ratio
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "slip_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.slip_ratio, out);
    out << "\n";
  }

  // member: collision_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "collision_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.collision_detected, out);
    out << "\n";
  }

  // member: ambient_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ambient_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.ambient_temperature, out);
    out << "\n";
  }

  // member: internal_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "internal_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.internal_temperature, out);
    out << "\n";
  }

  // member: humidity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "humidity: ";
    rosidl_generator_traits::value_to_yaml(msg.humidity, out);
    out << "\n";
  }

  // member: acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration:\n";
    to_block_style_yaml(msg.acceleration, out, indentation + 2);
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity:\n";
    to_block_style_yaml(msg.angular_velocity, out, indentation + 2);
  }

  // member: competition_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "competition_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.competition_mode, out);
    out << "\n";
  }

  // member: judge_score_estimate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "judge_score_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.judge_score_estimate, out);
    out << "\n";
  }

  // member: visual_servoing_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visual_servoing_active: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_servoing_active, out);
    out << "\n";
  }

  // member: tennis_balls_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tennis_balls_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.tennis_balls_detected, out);
    out << "\n";
  }

  // member: sun_direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sun_direction:\n";
    to_block_style_yaml(msg.sun_direction, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RoverStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rover_control

namespace rosidl_generator_traits
{

[[deprecated("use rover_control::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_control::msg::RoverStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const rover_control::msg::RoverStatus & msg)
{
  return rover_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rover_control::msg::RoverStatus>()
{
  return "rover_control::msg::RoverStatus";
}

template<>
inline const char * name<rover_control::msg::RoverStatus>()
{
  return "rover_control/msg/RoverStatus";
}

template<>
struct has_fixed_size<rover_control::msg::RoverStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rover_control::msg::RoverStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rover_control::msg::RoverStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROVER_CONTROL__MSG__DETAIL__ROVER_STATUS__TRAITS_HPP_
