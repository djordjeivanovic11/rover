// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from urc_msgs:msg/RoverStatus.idl
// generated code does not contain a copyright notice

#ifndef URC_MSGS__MSG__DETAIL__ROVER_STATUS__STRUCT_HPP_
#define URC_MSGS__MSG__DETAIL__ROVER_STATUS__STRUCT_HPP_

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
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'current_velocity'
#include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__urc_msgs__msg__RoverStatus __attribute__((deprecated))
#else
# define DEPRECATED__urc_msgs__msg__RoverStatus __declspec(deprecated)
#endif

namespace urc_msgs
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
    current_velocity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_voltage = 0.0f;
      this->battery_percentage = 0.0f;
      this->motors_enabled = false;
      this->emergency_stop = false;
      this->current_mode = "";
    }
  }

  explicit RoverStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    current_mode(_alloc),
    current_pose(_alloc, _init),
    current_velocity(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_voltage = 0.0f;
      this->battery_percentage = 0.0f;
      this->motors_enabled = false;
      this->emergency_stop = false;
      this->current_mode = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _battery_voltage_type =
    float;
  _battery_voltage_type battery_voltage;
  using _battery_percentage_type =
    float;
  _battery_percentage_type battery_percentage;
  using _motors_enabled_type =
    bool;
  _motors_enabled_type motors_enabled;
  using _emergency_stop_type =
    bool;
  _emergency_stop_type emergency_stop;
  using _current_mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_mode_type current_mode;
  using _current_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _current_pose_type current_pose;
  using _current_velocity_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _current_velocity_type current_velocity;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const float & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__battery_percentage(
    const float & _arg)
  {
    this->battery_percentage = _arg;
    return *this;
  }
  Type & set__motors_enabled(
    const bool & _arg)
  {
    this->motors_enabled = _arg;
    return *this;
  }
  Type & set__emergency_stop(
    const bool & _arg)
  {
    this->emergency_stop = _arg;
    return *this;
  }
  Type & set__current_mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_mode = _arg;
    return *this;
  }
  Type & set__current_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
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

  // constant declarations

  // pointer types
  using RawPtr =
    urc_msgs::msg::RoverStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const urc_msgs::msg::RoverStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<urc_msgs::msg::RoverStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<urc_msgs::msg::RoverStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      urc_msgs::msg::RoverStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<urc_msgs::msg::RoverStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      urc_msgs::msg::RoverStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<urc_msgs::msg::RoverStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<urc_msgs::msg::RoverStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<urc_msgs::msg::RoverStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__urc_msgs__msg__RoverStatus
    std::shared_ptr<urc_msgs::msg::RoverStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__urc_msgs__msg__RoverStatus
    std::shared_ptr<urc_msgs::msg::RoverStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RoverStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->battery_percentage != other.battery_percentage) {
      return false;
    }
    if (this->motors_enabled != other.motors_enabled) {
      return false;
    }
    if (this->emergency_stop != other.emergency_stop) {
      return false;
    }
    if (this->current_mode != other.current_mode) {
      return false;
    }
    if (this->current_pose != other.current_pose) {
      return false;
    }
    if (this->current_velocity != other.current_velocity) {
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
  urc_msgs::msg::RoverStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace urc_msgs

#endif  // URC_MSGS__MSG__DETAIL__ROVER_STATUS__STRUCT_HPP_
