// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arm_control:msg/ArmStatus.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__MSG__DETAIL__ARM_STATUS__STRUCT_HPP_
#define ARM_CONTROL__MSG__DETAIL__ARM_STATUS__STRUCT_HPP_

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
// Member 'joint_state'
#include "sensor_msgs/msg/detail/joint_state__struct.hpp"
// Member 'tcp_velocity'
// Member 'tcp_angular_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'current_wrench'
#include "geometry_msgs/msg/detail/wrench_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__msg__ArmStatus __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__msg__ArmStatus __declspec(deprecated)
#endif

namespace arm_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ArmStatus_
{
  using Type = ArmStatus_<ContainerAllocator>;

  explicit ArmStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    current_pose(_init),
    joint_state(_init),
    tcp_velocity(_init),
    tcp_angular_velocity(_init),
    current_wrench(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->current_tool_name = "";
      this->tool_attached = false;
      this->gripper_opening = 0.0f;
      this->gripper_force = 0.0f;
      this->safety_ok = false;
      this->estop_active = false;
      this->error_code = 0;
      this->error_message = "";
      this->in_motion = false;
      this->velocity_norm = 0.0f;
      this->force_limit_active = false;
      this->max_temperature = 0.0f;
      this->temperature_warning = false;
      this->max_current = 0.0f;
      this->current_warning = false;
      this->workspace_valid = false;
      this->workspace_margin = 0.0f;
      this->current_mission = "";
      this->current_action = "";
      this->action_progress = 0.0f;
      this->system_uptime = 0.0f;
      this->command_count = 0ul;
      this->fault_count = 0ul;
      this->success_rate = 0.0f;
    }
  }

  explicit ArmStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    current_pose(_alloc, _init),
    joint_state(_alloc, _init),
    current_tool_name(_alloc),
    error_message(_alloc),
    tcp_velocity(_alloc, _init),
    tcp_angular_velocity(_alloc, _init),
    current_wrench(_alloc, _init),
    current_mission(_alloc),
    current_action(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->current_tool_name = "";
      this->tool_attached = false;
      this->gripper_opening = 0.0f;
      this->gripper_force = 0.0f;
      this->safety_ok = false;
      this->estop_active = false;
      this->error_code = 0;
      this->error_message = "";
      this->in_motion = false;
      this->velocity_norm = 0.0f;
      this->force_limit_active = false;
      this->max_temperature = 0.0f;
      this->temperature_warning = false;
      this->max_current = 0.0f;
      this->current_warning = false;
      this->workspace_valid = false;
      this->workspace_margin = 0.0f;
      this->current_mission = "";
      this->current_action = "";
      this->action_progress = 0.0f;
      this->system_uptime = 0.0f;
      this->command_count = 0ul;
      this->fault_count = 0ul;
      this->success_rate = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    uint8_t;
  _state_type state;
  using _current_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _current_pose_type current_pose;
  using _joint_state_type =
    sensor_msgs::msg::JointState_<ContainerAllocator>;
  _joint_state_type joint_state;
  using _current_tool_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_tool_name_type current_tool_name;
  using _tool_attached_type =
    bool;
  _tool_attached_type tool_attached;
  using _gripper_opening_type =
    float;
  _gripper_opening_type gripper_opening;
  using _gripper_force_type =
    float;
  _gripper_force_type gripper_force;
  using _safety_ok_type =
    bool;
  _safety_ok_type safety_ok;
  using _estop_active_type =
    bool;
  _estop_active_type estop_active;
  using _error_code_type =
    uint8_t;
  _error_code_type error_code;
  using _error_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_message_type error_message;
  using _in_motion_type =
    bool;
  _in_motion_type in_motion;
  using _velocity_norm_type =
    float;
  _velocity_norm_type velocity_norm;
  using _tcp_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _tcp_velocity_type tcp_velocity;
  using _tcp_angular_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _tcp_angular_velocity_type tcp_angular_velocity;
  using _current_wrench_type =
    geometry_msgs::msg::WrenchStamped_<ContainerAllocator>;
  _current_wrench_type current_wrench;
  using _force_limit_active_type =
    bool;
  _force_limit_active_type force_limit_active;
  using _joint_temperatures_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _joint_temperatures_type joint_temperatures;
  using _max_temperature_type =
    float;
  _max_temperature_type max_temperature;
  using _temperature_warning_type =
    bool;
  _temperature_warning_type temperature_warning;
  using _joint_currents_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _joint_currents_type joint_currents;
  using _max_current_type =
    float;
  _max_current_type max_current;
  using _current_warning_type =
    bool;
  _current_warning_type current_warning;
  using _workspace_valid_type =
    bool;
  _workspace_valid_type workspace_valid;
  using _workspace_margin_type =
    float;
  _workspace_margin_type workspace_margin;
  using _current_mission_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_mission_type current_mission;
  using _current_action_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_action_type current_action;
  using _action_progress_type =
    float;
  _action_progress_type action_progress;
  using _system_uptime_type =
    float;
  _system_uptime_type system_uptime;
  using _command_count_type =
    uint32_t;
  _command_count_type command_count;
  using _fault_count_type =
    uint32_t;
  _fault_count_type fault_count;
  using _success_rate_type =
    float;
  _success_rate_type success_rate;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__current_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->current_pose = _arg;
    return *this;
  }
  Type & set__joint_state(
    const sensor_msgs::msg::JointState_<ContainerAllocator> & _arg)
  {
    this->joint_state = _arg;
    return *this;
  }
  Type & set__current_tool_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_tool_name = _arg;
    return *this;
  }
  Type & set__tool_attached(
    const bool & _arg)
  {
    this->tool_attached = _arg;
    return *this;
  }
  Type & set__gripper_opening(
    const float & _arg)
  {
    this->gripper_opening = _arg;
    return *this;
  }
  Type & set__gripper_force(
    const float & _arg)
  {
    this->gripper_force = _arg;
    return *this;
  }
  Type & set__safety_ok(
    const bool & _arg)
  {
    this->safety_ok = _arg;
    return *this;
  }
  Type & set__estop_active(
    const bool & _arg)
  {
    this->estop_active = _arg;
    return *this;
  }
  Type & set__error_code(
    const uint8_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__error_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error_message = _arg;
    return *this;
  }
  Type & set__in_motion(
    const bool & _arg)
  {
    this->in_motion = _arg;
    return *this;
  }
  Type & set__velocity_norm(
    const float & _arg)
  {
    this->velocity_norm = _arg;
    return *this;
  }
  Type & set__tcp_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->tcp_velocity = _arg;
    return *this;
  }
  Type & set__tcp_angular_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->tcp_angular_velocity = _arg;
    return *this;
  }
  Type & set__current_wrench(
    const geometry_msgs::msg::WrenchStamped_<ContainerAllocator> & _arg)
  {
    this->current_wrench = _arg;
    return *this;
  }
  Type & set__force_limit_active(
    const bool & _arg)
  {
    this->force_limit_active = _arg;
    return *this;
  }
  Type & set__joint_temperatures(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->joint_temperatures = _arg;
    return *this;
  }
  Type & set__max_temperature(
    const float & _arg)
  {
    this->max_temperature = _arg;
    return *this;
  }
  Type & set__temperature_warning(
    const bool & _arg)
  {
    this->temperature_warning = _arg;
    return *this;
  }
  Type & set__joint_currents(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->joint_currents = _arg;
    return *this;
  }
  Type & set__max_current(
    const float & _arg)
  {
    this->max_current = _arg;
    return *this;
  }
  Type & set__current_warning(
    const bool & _arg)
  {
    this->current_warning = _arg;
    return *this;
  }
  Type & set__workspace_valid(
    const bool & _arg)
  {
    this->workspace_valid = _arg;
    return *this;
  }
  Type & set__workspace_margin(
    const float & _arg)
  {
    this->workspace_margin = _arg;
    return *this;
  }
  Type & set__current_mission(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_mission = _arg;
    return *this;
  }
  Type & set__current_action(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_action = _arg;
    return *this;
  }
  Type & set__action_progress(
    const float & _arg)
  {
    this->action_progress = _arg;
    return *this;
  }
  Type & set__system_uptime(
    const float & _arg)
  {
    this->system_uptime = _arg;
    return *this;
  }
  Type & set__command_count(
    const uint32_t & _arg)
  {
    this->command_count = _arg;
    return *this;
  }
  Type & set__fault_count(
    const uint32_t & _arg)
  {
    this->fault_count = _arg;
    return *this;
  }
  Type & set__success_rate(
    const float & _arg)
  {
    this->success_rate = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t IDLE =
    0u;
  static constexpr uint8_t PLANNING =
    1u;
  static constexpr uint8_t EXECUTING =
    2u;
  static constexpr uint8_t TOOL_CHANGE =
    3u;
  static constexpr uint8_t FAULT =
    4u;
  static constexpr uint8_t EMERGENCY_STOP =
    5u;
  static constexpr uint8_t CALIBRATING =
    6u;

  // pointer types
  using RawPtr =
    arm_control::msg::ArmStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::msg::ArmStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::msg::ArmStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::msg::ArmStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::msg::ArmStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::msg::ArmStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::msg::ArmStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::msg::ArmStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::msg::ArmStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::msg::ArmStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__msg__ArmStatus
    std::shared_ptr<arm_control::msg::ArmStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__msg__ArmStatus
    std::shared_ptr<arm_control::msg::ArmStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArmStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->current_pose != other.current_pose) {
      return false;
    }
    if (this->joint_state != other.joint_state) {
      return false;
    }
    if (this->current_tool_name != other.current_tool_name) {
      return false;
    }
    if (this->tool_attached != other.tool_attached) {
      return false;
    }
    if (this->gripper_opening != other.gripper_opening) {
      return false;
    }
    if (this->gripper_force != other.gripper_force) {
      return false;
    }
    if (this->safety_ok != other.safety_ok) {
      return false;
    }
    if (this->estop_active != other.estop_active) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->error_message != other.error_message) {
      return false;
    }
    if (this->in_motion != other.in_motion) {
      return false;
    }
    if (this->velocity_norm != other.velocity_norm) {
      return false;
    }
    if (this->tcp_velocity != other.tcp_velocity) {
      return false;
    }
    if (this->tcp_angular_velocity != other.tcp_angular_velocity) {
      return false;
    }
    if (this->current_wrench != other.current_wrench) {
      return false;
    }
    if (this->force_limit_active != other.force_limit_active) {
      return false;
    }
    if (this->joint_temperatures != other.joint_temperatures) {
      return false;
    }
    if (this->max_temperature != other.max_temperature) {
      return false;
    }
    if (this->temperature_warning != other.temperature_warning) {
      return false;
    }
    if (this->joint_currents != other.joint_currents) {
      return false;
    }
    if (this->max_current != other.max_current) {
      return false;
    }
    if (this->current_warning != other.current_warning) {
      return false;
    }
    if (this->workspace_valid != other.workspace_valid) {
      return false;
    }
    if (this->workspace_margin != other.workspace_margin) {
      return false;
    }
    if (this->current_mission != other.current_mission) {
      return false;
    }
    if (this->current_action != other.current_action) {
      return false;
    }
    if (this->action_progress != other.action_progress) {
      return false;
    }
    if (this->system_uptime != other.system_uptime) {
      return false;
    }
    if (this->command_count != other.command_count) {
      return false;
    }
    if (this->fault_count != other.fault_count) {
      return false;
    }
    if (this->success_rate != other.success_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArmStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArmStatus_

// alias to use template instance with default allocator
using ArmStatus =
  arm_control::msg::ArmStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ArmStatus_<ContainerAllocator>::IDLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ArmStatus_<ContainerAllocator>::PLANNING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ArmStatus_<ContainerAllocator>::EXECUTING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ArmStatus_<ContainerAllocator>::TOOL_CHANGE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ArmStatus_<ContainerAllocator>::FAULT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ArmStatus_<ContainerAllocator>::EMERGENCY_STOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ArmStatus_<ContainerAllocator>::CALIBRATING;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace arm_control

#endif  // ARM_CONTROL__MSG__DETAIL__ARM_STATUS__STRUCT_HPP_
