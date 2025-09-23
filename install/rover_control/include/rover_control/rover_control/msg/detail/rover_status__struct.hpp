// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_control:msg/RoverStatus.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__MSG__DETAIL__ROVER_STATUS__STRUCT_HPP_
#define ROVER_CONTROL__MSG__DETAIL__ROVER_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'current_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'current_velocity'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'odometry'
#include "nav_msgs/msg/detail/odometry__struct.hpp"
// Member 'acceleration'
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'sun_direction'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__msg__RoverStatus __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__msg__RoverStatus __declspec(deprecated)
#endif

namespace rover_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RoverStatus_
{
  using Type = RoverStatus_<ContainerAllocator>;

  explicit RoverStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    current_pose(_init),
    current_velocity(_init),
    odometry(_init),
    acceleration(_init),
    angular_velocity(_init),
    sun_direction(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_action = "";
      this->mission_mode = "";
      this->is_autonomous = false;
      this->safety_ok = false;
      this->emergency_stop_active = false;
      this->mission_progress_percentage = 0.0f;
      this->autonomy_time_remaining = 0.0f;
      this->waypoints_completed = 0l;
      this->total_waypoints = 0l;
      this->battery_voltage = 0.0f;
      this->battery_current = 0.0f;
      this->battery_soc = 0.0f;
      this->battery_temperature = 0.0f;
      this->communication_quality = 0.0f;
      this->bandwidth_usage = 0.0f;
      this->cpu_usage = 0.0f;
      this->memory_usage = 0.0f;
      this->stuck_detected = false;
      this->slip_ratio = 0.0f;
      this->collision_detected = false;
      this->ambient_temperature = 0.0f;
      this->internal_temperature = 0.0f;
      this->humidity = 0.0f;
      this->competition_mode = "";
      this->judge_score_estimate = 0.0f;
      this->visual_servoing_active = false;
      this->tennis_balls_detected = 0l;
    }
  }

  explicit RoverStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    current_action(_alloc),
    mission_mode(_alloc),
    current_pose(_alloc, _init),
    current_velocity(_alloc, _init),
    odometry(_alloc, _init),
    acceleration(_alloc, _init),
    angular_velocity(_alloc, _init),
    competition_mode(_alloc),
    sun_direction(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_action = "";
      this->mission_mode = "";
      this->is_autonomous = false;
      this->safety_ok = false;
      this->emergency_stop_active = false;
      this->mission_progress_percentage = 0.0f;
      this->autonomy_time_remaining = 0.0f;
      this->waypoints_completed = 0l;
      this->total_waypoints = 0l;
      this->battery_voltage = 0.0f;
      this->battery_current = 0.0f;
      this->battery_soc = 0.0f;
      this->battery_temperature = 0.0f;
      this->communication_quality = 0.0f;
      this->bandwidth_usage = 0.0f;
      this->cpu_usage = 0.0f;
      this->memory_usage = 0.0f;
      this->stuck_detected = false;
      this->slip_ratio = 0.0f;
      this->collision_detected = false;
      this->ambient_temperature = 0.0f;
      this->internal_temperature = 0.0f;
      this->humidity = 0.0f;
      this->competition_mode = "";
      this->judge_score_estimate = 0.0f;
      this->visual_servoing_active = false;
      this->tennis_balls_detected = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _current_action_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_action_type current_action;
  using _mission_mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_mode_type mission_mode;
  using _is_autonomous_type =
    bool;
  _is_autonomous_type is_autonomous;
  using _safety_ok_type =
    bool;
  _safety_ok_type safety_ok;
  using _emergency_stop_active_type =
    bool;
  _emergency_stop_active_type emergency_stop_active;
  using _current_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _current_pose_type current_pose;
  using _current_velocity_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _current_velocity_type current_velocity;
  using _odometry_type =
    nav_msgs::msg::Odometry_<ContainerAllocator>;
  _odometry_type odometry;
  using _mission_progress_percentage_type =
    float;
  _mission_progress_percentage_type mission_progress_percentage;
  using _autonomy_time_remaining_type =
    float;
  _autonomy_time_remaining_type autonomy_time_remaining;
  using _waypoints_completed_type =
    int32_t;
  _waypoints_completed_type waypoints_completed;
  using _total_waypoints_type =
    int32_t;
  _total_waypoints_type total_waypoints;
  using _battery_voltage_type =
    float;
  _battery_voltage_type battery_voltage;
  using _battery_current_type =
    float;
  _battery_current_type battery_current;
  using _battery_soc_type =
    float;
  _battery_soc_type battery_soc;
  using _battery_temperature_type =
    float;
  _battery_temperature_type battery_temperature;
  using _motor_currents_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _motor_currents_type motor_currents;
  using _motor_temperatures_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _motor_temperatures_type motor_temperatures;
  using _communication_quality_type =
    float;
  _communication_quality_type communication_quality;
  using _bandwidth_usage_type =
    float;
  _bandwidth_usage_type bandwidth_usage;
  using _cpu_usage_type =
    float;
  _cpu_usage_type cpu_usage;
  using _memory_usage_type =
    float;
  _memory_usage_type memory_usage;
  using _active_faults_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _active_faults_type active_faults;
  using _fault_messages_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _fault_messages_type fault_messages;
  using _stuck_detected_type =
    bool;
  _stuck_detected_type stuck_detected;
  using _slip_ratio_type =
    float;
  _slip_ratio_type slip_ratio;
  using _collision_detected_type =
    bool;
  _collision_detected_type collision_detected;
  using _ambient_temperature_type =
    float;
  _ambient_temperature_type ambient_temperature;
  using _internal_temperature_type =
    float;
  _internal_temperature_type internal_temperature;
  using _humidity_type =
    float;
  _humidity_type humidity;
  using _acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _acceleration_type acceleration;
  using _angular_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _angular_velocity_type angular_velocity;
  using _competition_mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _competition_mode_type competition_mode;
  using _judge_score_estimate_type =
    float;
  _judge_score_estimate_type judge_score_estimate;
  using _visual_servoing_active_type =
    bool;
  _visual_servoing_active_type visual_servoing_active;
  using _tennis_balls_detected_type =
    int32_t;
  _tennis_balls_detected_type tennis_balls_detected;
  using _sun_direction_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _sun_direction_type sun_direction;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__current_action(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_action = _arg;
    return *this;
  }
  Type & set__mission_mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_mode = _arg;
    return *this;
  }
  Type & set__is_autonomous(
    const bool & _arg)
  {
    this->is_autonomous = _arg;
    return *this;
  }
  Type & set__safety_ok(
    const bool & _arg)
  {
    this->safety_ok = _arg;
    return *this;
  }
  Type & set__emergency_stop_active(
    const bool & _arg)
  {
    this->emergency_stop_active = _arg;
    return *this;
  }
  Type & set__current_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->current_pose = _arg;
    return *this;
  }
  Type & set__current_velocity(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->current_velocity = _arg;
    return *this;
  }
  Type & set__odometry(
    const nav_msgs::msg::Odometry_<ContainerAllocator> & _arg)
  {
    this->odometry = _arg;
    return *this;
  }
  Type & set__mission_progress_percentage(
    const float & _arg)
  {
    this->mission_progress_percentage = _arg;
    return *this;
  }
  Type & set__autonomy_time_remaining(
    const float & _arg)
  {
    this->autonomy_time_remaining = _arg;
    return *this;
  }
  Type & set__waypoints_completed(
    const int32_t & _arg)
  {
    this->waypoints_completed = _arg;
    return *this;
  }
  Type & set__total_waypoints(
    const int32_t & _arg)
  {
    this->total_waypoints = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const float & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__battery_current(
    const float & _arg)
  {
    this->battery_current = _arg;
    return *this;
  }
  Type & set__battery_soc(
    const float & _arg)
  {
    this->battery_soc = _arg;
    return *this;
  }
  Type & set__battery_temperature(
    const float & _arg)
  {
    this->battery_temperature = _arg;
    return *this;
  }
  Type & set__motor_currents(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->motor_currents = _arg;
    return *this;
  }
  Type & set__motor_temperatures(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->motor_temperatures = _arg;
    return *this;
  }
  Type & set__communication_quality(
    const float & _arg)
  {
    this->communication_quality = _arg;
    return *this;
  }
  Type & set__bandwidth_usage(
    const float & _arg)
  {
    this->bandwidth_usage = _arg;
    return *this;
  }
  Type & set__cpu_usage(
    const float & _arg)
  {
    this->cpu_usage = _arg;
    return *this;
  }
  Type & set__memory_usage(
    const float & _arg)
  {
    this->memory_usage = _arg;
    return *this;
  }
  Type & set__active_faults(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->active_faults = _arg;
    return *this;
  }
  Type & set__fault_messages(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->fault_messages = _arg;
    return *this;
  }
  Type & set__stuck_detected(
    const bool & _arg)
  {
    this->stuck_detected = _arg;
    return *this;
  }
  Type & set__slip_ratio(
    const float & _arg)
  {
    this->slip_ratio = _arg;
    return *this;
  }
  Type & set__collision_detected(
    const bool & _arg)
  {
    this->collision_detected = _arg;
    return *this;
  }
  Type & set__ambient_temperature(
    const float & _arg)
  {
    this->ambient_temperature = _arg;
    return *this;
  }
  Type & set__internal_temperature(
    const float & _arg)
  {
    this->internal_temperature = _arg;
    return *this;
  }
  Type & set__humidity(
    const float & _arg)
  {
    this->humidity = _arg;
    return *this;
  }
  Type & set__acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }
  Type & set__competition_mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->competition_mode = _arg;
    return *this;
  }
  Type & set__judge_score_estimate(
    const float & _arg)
  {
    this->judge_score_estimate = _arg;
    return *this;
  }
  Type & set__visual_servoing_active(
    const bool & _arg)
  {
    this->visual_servoing_active = _arg;
    return *this;
  }
  Type & set__tennis_balls_detected(
    const int32_t & _arg)
  {
    this->tennis_balls_detected = _arg;
    return *this;
  }
  Type & set__sun_direction(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->sun_direction = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_control::msg::RoverStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::msg::RoverStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::msg::RoverStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::msg::RoverStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::msg::RoverStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::msg::RoverStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::msg::RoverStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::msg::RoverStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::msg::RoverStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::msg::RoverStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__msg__RoverStatus
    std::shared_ptr<rover_control::msg::RoverStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__msg__RoverStatus
    std::shared_ptr<rover_control::msg::RoverStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RoverStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->current_action != other.current_action) {
      return false;
    }
    if (this->mission_mode != other.mission_mode) {
      return false;
    }
    if (this->is_autonomous != other.is_autonomous) {
      return false;
    }
    if (this->safety_ok != other.safety_ok) {
      return false;
    }
    if (this->emergency_stop_active != other.emergency_stop_active) {
      return false;
    }
    if (this->current_pose != other.current_pose) {
      return false;
    }
    if (this->current_velocity != other.current_velocity) {
      return false;
    }
    if (this->odometry != other.odometry) {
      return false;
    }
    if (this->mission_progress_percentage != other.mission_progress_percentage) {
      return false;
    }
    if (this->autonomy_time_remaining != other.autonomy_time_remaining) {
      return false;
    }
    if (this->waypoints_completed != other.waypoints_completed) {
      return false;
    }
    if (this->total_waypoints != other.total_waypoints) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->battery_current != other.battery_current) {
      return false;
    }
    if (this->battery_soc != other.battery_soc) {
      return false;
    }
    if (this->battery_temperature != other.battery_temperature) {
      return false;
    }
    if (this->motor_currents != other.motor_currents) {
      return false;
    }
    if (this->motor_temperatures != other.motor_temperatures) {
      return false;
    }
    if (this->communication_quality != other.communication_quality) {
      return false;
    }
    if (this->bandwidth_usage != other.bandwidth_usage) {
      return false;
    }
    if (this->cpu_usage != other.cpu_usage) {
      return false;
    }
    if (this->memory_usage != other.memory_usage) {
      return false;
    }
    if (this->active_faults != other.active_faults) {
      return false;
    }
    if (this->fault_messages != other.fault_messages) {
      return false;
    }
    if (this->stuck_detected != other.stuck_detected) {
      return false;
    }
    if (this->slip_ratio != other.slip_ratio) {
      return false;
    }
    if (this->collision_detected != other.collision_detected) {
      return false;
    }
    if (this->ambient_temperature != other.ambient_temperature) {
      return false;
    }
    if (this->internal_temperature != other.internal_temperature) {
      return false;
    }
    if (this->humidity != other.humidity) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    if (this->competition_mode != other.competition_mode) {
      return false;
    }
    if (this->judge_score_estimate != other.judge_score_estimate) {
      return false;
    }
    if (this->visual_servoing_active != other.visual_servoing_active) {
      return false;
    }
    if (this->tennis_balls_detected != other.tennis_balls_detected) {
      return false;
    }
    if (this->sun_direction != other.sun_direction) {
      return false;
    }
    return true;
  }
  bool operator!=(const RoverStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RoverStatus_

// alias to use template instance with default allocator
using RoverStatus =
  rover_control::msg::RoverStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rover_control

#endif  // ROVER_CONTROL__MSG__DETAIL__ROVER_STATUS__STRUCT_HPP_
