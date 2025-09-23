// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_control:msg/DriveCommand.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__STRUCT_HPP_
#define ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__STRUCT_HPP_

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
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__msg__DriveCommand __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__msg__DriveCommand __declspec(deprecated)
#endif

namespace rover_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DriveCommand_
{
  using Type = DriveCommand_<ContainerAllocator>;

  explicit DriveCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    velocity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->acceleration = 0.0f;
      this->duration = 0.0f;
      this->control_mode = "";
      this->relative_command = false;
      this->max_velocity = 0.0f;
      this->max_acceleration = 0.0f;
      this->enforce_safety_limits = false;
      this->mission_mode = "";
      this->curvature = 0.0f;
      this->lookahead_distance = 0.0f;
      this->use_slip_compensation = false;
      this->priority = 0l;
      this->timeout = 0.0f;
      this->source = "";
    }
  }

  explicit DriveCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    velocity(_alloc, _init),
    control_mode(_alloc),
    mission_mode(_alloc),
    source(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->acceleration = 0.0f;
      this->duration = 0.0f;
      this->control_mode = "";
      this->relative_command = false;
      this->max_velocity = 0.0f;
      this->max_acceleration = 0.0f;
      this->enforce_safety_limits = false;
      this->mission_mode = "";
      this->curvature = 0.0f;
      this->lookahead_distance = 0.0f;
      this->use_slip_compensation = false;
      this->priority = 0l;
      this->timeout = 0.0f;
      this->source = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _velocity_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _velocity_type velocity;
  using _acceleration_type =
    float;
  _acceleration_type acceleration;
  using _duration_type =
    float;
  _duration_type duration;
  using _control_mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _control_mode_type control_mode;
  using _relative_command_type =
    bool;
  _relative_command_type relative_command;
  using _max_velocity_type =
    float;
  _max_velocity_type max_velocity;
  using _max_acceleration_type =
    float;
  _max_acceleration_type max_acceleration;
  using _enforce_safety_limits_type =
    bool;
  _enforce_safety_limits_type enforce_safety_limits;
  using _mission_mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_mode_type mission_mode;
  using _curvature_type =
    float;
  _curvature_type curvature;
  using _lookahead_distance_type =
    float;
  _lookahead_distance_type lookahead_distance;
  using _use_slip_compensation_type =
    bool;
  _use_slip_compensation_type use_slip_compensation;
  using _priority_type =
    int32_t;
  _priority_type priority;
  using _timeout_type =
    float;
  _timeout_type timeout;
  using _source_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _source_type source;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__acceleration(
    const float & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__duration(
    const float & _arg)
  {
    this->duration = _arg;
    return *this;
  }
  Type & set__control_mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->control_mode = _arg;
    return *this;
  }
  Type & set__relative_command(
    const bool & _arg)
  {
    this->relative_command = _arg;
    return *this;
  }
  Type & set__max_velocity(
    const float & _arg)
  {
    this->max_velocity = _arg;
    return *this;
  }
  Type & set__max_acceleration(
    const float & _arg)
  {
    this->max_acceleration = _arg;
    return *this;
  }
  Type & set__enforce_safety_limits(
    const bool & _arg)
  {
    this->enforce_safety_limits = _arg;
    return *this;
  }
  Type & set__mission_mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_mode = _arg;
    return *this;
  }
  Type & set__curvature(
    const float & _arg)
  {
    this->curvature = _arg;
    return *this;
  }
  Type & set__lookahead_distance(
    const float & _arg)
  {
    this->lookahead_distance = _arg;
    return *this;
  }
  Type & set__use_slip_compensation(
    const bool & _arg)
  {
    this->use_slip_compensation = _arg;
    return *this;
  }
  Type & set__priority(
    const int32_t & _arg)
  {
    this->priority = _arg;
    return *this;
  }
  Type & set__timeout(
    const float & _arg)
  {
    this->timeout = _arg;
    return *this;
  }
  Type & set__source(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->source = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_control::msg::DriveCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::msg::DriveCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::msg::DriveCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::msg::DriveCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::msg::DriveCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::msg::DriveCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::msg::DriveCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::msg::DriveCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::msg::DriveCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::msg::DriveCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__msg__DriveCommand
    std::shared_ptr<rover_control::msg::DriveCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__msg__DriveCommand
    std::shared_ptr<rover_control::msg::DriveCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriveCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    if (this->control_mode != other.control_mode) {
      return false;
    }
    if (this->relative_command != other.relative_command) {
      return false;
    }
    if (this->max_velocity != other.max_velocity) {
      return false;
    }
    if (this->max_acceleration != other.max_acceleration) {
      return false;
    }
    if (this->enforce_safety_limits != other.enforce_safety_limits) {
      return false;
    }
    if (this->mission_mode != other.mission_mode) {
      return false;
    }
    if (this->curvature != other.curvature) {
      return false;
    }
    if (this->lookahead_distance != other.lookahead_distance) {
      return false;
    }
    if (this->use_slip_compensation != other.use_slip_compensation) {
      return false;
    }
    if (this->priority != other.priority) {
      return false;
    }
    if (this->timeout != other.timeout) {
      return false;
    }
    if (this->source != other.source) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriveCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriveCommand_

// alias to use template instance with default allocator
using DriveCommand =
  rover_control::msg::DriveCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rover_control

#endif  // ROVER_CONTROL__MSG__DETAIL__DRIVE_COMMAND__STRUCT_HPP_
