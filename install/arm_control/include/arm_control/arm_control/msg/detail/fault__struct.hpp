// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arm_control:msg/Fault.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__MSG__DETAIL__FAULT__STRUCT_HPP_
#define ARM_CONTROL__MSG__DETAIL__FAULT__STRUCT_HPP_

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
// Member 'fault_location'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'time_since_last'
#include "builtin_interfaces/msg/detail/duration__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__msg__Fault __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__msg__Fault __declspec(deprecated)
#endif

namespace arm_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Fault_
{
  using Type = Fault_<ContainerAllocator>;

  explicit Fault_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    fault_location(_init),
    time_since_last(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->severity = 0;
      this->fault_code = 0ul;
      this->fault_category = "";
      this->fault_description = "";
      this->component_name = "";
      this->joint_name = "";
      this->fault_value = 0.0;
      this->fault_threshold = 0.0;
      this->fault_units = "";
      this->reference_frame = "";
      this->auto_recoverable = false;
      this->recovery_attempted = false;
      this->recovery_successful = false;
      this->recovery_action = "";
      this->system_state = 0;
      this->current_action = "";
      this->action_progress = 0.0f;
      this->occurrence_count = 0ul;
      this->recommended_action = "";
      this->operator_intervention_required = 0;
      this->troubleshooting_info = "";
    }
  }

  explicit Fault_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    fault_category(_alloc),
    fault_description(_alloc),
    component_name(_alloc),
    joint_name(_alloc),
    fault_units(_alloc),
    fault_location(_alloc, _init),
    reference_frame(_alloc),
    recovery_action(_alloc),
    current_action(_alloc),
    time_since_last(_alloc, _init),
    recommended_action(_alloc),
    troubleshooting_info(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->severity = 0;
      this->fault_code = 0ul;
      this->fault_category = "";
      this->fault_description = "";
      this->component_name = "";
      this->joint_name = "";
      this->fault_value = 0.0;
      this->fault_threshold = 0.0;
      this->fault_units = "";
      this->reference_frame = "";
      this->auto_recoverable = false;
      this->recovery_attempted = false;
      this->recovery_successful = false;
      this->recovery_action = "";
      this->system_state = 0;
      this->current_action = "";
      this->action_progress = 0.0f;
      this->occurrence_count = 0ul;
      this->recommended_action = "";
      this->operator_intervention_required = 0;
      this->troubleshooting_info = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _severity_type =
    uint8_t;
  _severity_type severity;
  using _fault_code_type =
    uint32_t;
  _fault_code_type fault_code;
  using _fault_category_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _fault_category_type fault_category;
  using _fault_description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _fault_description_type fault_description;
  using _component_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _component_name_type component_name;
  using _joint_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _joint_name_type joint_name;
  using _fault_value_type =
    double;
  _fault_value_type fault_value;
  using _fault_threshold_type =
    double;
  _fault_threshold_type fault_threshold;
  using _fault_units_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _fault_units_type fault_units;
  using _fault_location_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _fault_location_type fault_location;
  using _reference_frame_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reference_frame_type reference_frame;
  using _auto_recoverable_type =
    bool;
  _auto_recoverable_type auto_recoverable;
  using _recovery_attempted_type =
    bool;
  _recovery_attempted_type recovery_attempted;
  using _recovery_successful_type =
    bool;
  _recovery_successful_type recovery_successful;
  using _recovery_action_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _recovery_action_type recovery_action;
  using _system_state_type =
    uint8_t;
  _system_state_type system_state;
  using _current_action_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_action_type current_action;
  using _action_progress_type =
    float;
  _action_progress_type action_progress;
  using _related_faults_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _related_faults_type related_faults;
  using _occurrence_count_type =
    uint32_t;
  _occurrence_count_type occurrence_count;
  using _time_since_last_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _time_since_last_type time_since_last;
  using _recommended_action_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _recommended_action_type recommended_action;
  using _operator_intervention_required_type =
    uint8_t;
  _operator_intervention_required_type operator_intervention_required;
  using _troubleshooting_info_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _troubleshooting_info_type troubleshooting_info;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__severity(
    const uint8_t & _arg)
  {
    this->severity = _arg;
    return *this;
  }
  Type & set__fault_code(
    const uint32_t & _arg)
  {
    this->fault_code = _arg;
    return *this;
  }
  Type & set__fault_category(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->fault_category = _arg;
    return *this;
  }
  Type & set__fault_description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->fault_description = _arg;
    return *this;
  }
  Type & set__component_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->component_name = _arg;
    return *this;
  }
  Type & set__joint_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->joint_name = _arg;
    return *this;
  }
  Type & set__fault_value(
    const double & _arg)
  {
    this->fault_value = _arg;
    return *this;
  }
  Type & set__fault_threshold(
    const double & _arg)
  {
    this->fault_threshold = _arg;
    return *this;
  }
  Type & set__fault_units(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->fault_units = _arg;
    return *this;
  }
  Type & set__fault_location(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->fault_location = _arg;
    return *this;
  }
  Type & set__reference_frame(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reference_frame = _arg;
    return *this;
  }
  Type & set__auto_recoverable(
    const bool & _arg)
  {
    this->auto_recoverable = _arg;
    return *this;
  }
  Type & set__recovery_attempted(
    const bool & _arg)
  {
    this->recovery_attempted = _arg;
    return *this;
  }
  Type & set__recovery_successful(
    const bool & _arg)
  {
    this->recovery_successful = _arg;
    return *this;
  }
  Type & set__recovery_action(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->recovery_action = _arg;
    return *this;
  }
  Type & set__system_state(
    const uint8_t & _arg)
  {
    this->system_state = _arg;
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
  Type & set__related_faults(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->related_faults = _arg;
    return *this;
  }
  Type & set__occurrence_count(
    const uint32_t & _arg)
  {
    this->occurrence_count = _arg;
    return *this;
  }
  Type & set__time_since_last(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->time_since_last = _arg;
    return *this;
  }
  Type & set__recommended_action(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->recommended_action = _arg;
    return *this;
  }
  Type & set__operator_intervention_required(
    const uint8_t & _arg)
  {
    this->operator_intervention_required = _arg;
    return *this;
  }
  Type & set__troubleshooting_info(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->troubleshooting_info = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t INFO =
    0u;
  static constexpr uint8_t WARNING =
    1u;
  // guard against 'ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(ERROR)
#    pragma push_macro("ERROR")
#    undef ERROR
#  endif
#endif
  static constexpr uint8_t ERROR =
    2u;
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("ERROR")
#endif
  static constexpr uint8_t CRITICAL =
    3u;
  static constexpr uint8_t EMERGENCY =
    4u;

  // pointer types
  using RawPtr =
    arm_control::msg::Fault_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::msg::Fault_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::msg::Fault_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::msg::Fault_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::msg::Fault_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::msg::Fault_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::msg::Fault_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::msg::Fault_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::msg::Fault_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::msg::Fault_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__msg__Fault
    std::shared_ptr<arm_control::msg::Fault_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__msg__Fault
    std::shared_ptr<arm_control::msg::Fault_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Fault_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->severity != other.severity) {
      return false;
    }
    if (this->fault_code != other.fault_code) {
      return false;
    }
    if (this->fault_category != other.fault_category) {
      return false;
    }
    if (this->fault_description != other.fault_description) {
      return false;
    }
    if (this->component_name != other.component_name) {
      return false;
    }
    if (this->joint_name != other.joint_name) {
      return false;
    }
    if (this->fault_value != other.fault_value) {
      return false;
    }
    if (this->fault_threshold != other.fault_threshold) {
      return false;
    }
    if (this->fault_units != other.fault_units) {
      return false;
    }
    if (this->fault_location != other.fault_location) {
      return false;
    }
    if (this->reference_frame != other.reference_frame) {
      return false;
    }
    if (this->auto_recoverable != other.auto_recoverable) {
      return false;
    }
    if (this->recovery_attempted != other.recovery_attempted) {
      return false;
    }
    if (this->recovery_successful != other.recovery_successful) {
      return false;
    }
    if (this->recovery_action != other.recovery_action) {
      return false;
    }
    if (this->system_state != other.system_state) {
      return false;
    }
    if (this->current_action != other.current_action) {
      return false;
    }
    if (this->action_progress != other.action_progress) {
      return false;
    }
    if (this->related_faults != other.related_faults) {
      return false;
    }
    if (this->occurrence_count != other.occurrence_count) {
      return false;
    }
    if (this->time_since_last != other.time_since_last) {
      return false;
    }
    if (this->recommended_action != other.recommended_action) {
      return false;
    }
    if (this->operator_intervention_required != other.operator_intervention_required) {
      return false;
    }
    if (this->troubleshooting_info != other.troubleshooting_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const Fault_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Fault_

// alias to use template instance with default allocator
using Fault =
  arm_control::msg::Fault_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Fault_<ContainerAllocator>::INFO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Fault_<ContainerAllocator>::WARNING;
#endif  // __cplusplus < 201703L
// guard against 'ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(ERROR)
#    pragma push_macro("ERROR")
#    undef ERROR
#  endif
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Fault_<ContainerAllocator>::ERROR;
#endif  // __cplusplus < 201703L
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("ERROR")
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Fault_<ContainerAllocator>::CRITICAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Fault_<ContainerAllocator>::EMERGENCY;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace arm_control

#endif  // ARM_CONTROL__MSG__DETAIL__FAULT__STRUCT_HPP_
