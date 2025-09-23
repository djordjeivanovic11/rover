// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from urc_msgs:msg/ScienceData.idl
// generated code does not contain a copyright notice

#ifndef URC_MSGS__MSG__DETAIL__SCIENCE_DATA__STRUCT_HPP_
#define URC_MSGS__MSG__DETAIL__SCIENCE_DATA__STRUCT_HPP_

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
// Member 'sample_location'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__urc_msgs__msg__ScienceData __attribute__((deprecated))
#else
# define DEPRECATED__urc_msgs__msg__ScienceData __declspec(deprecated)
#endif

namespace urc_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ScienceData_
{
  using Type = ScienceData_<ContainerAllocator>;

  explicit ScienceData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    sample_location(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sample_id = "";
      this->ph_level = 0.0f;
      this->temperature = 0.0f;
      this->moisture = 0.0f;
    }
  }

  explicit ScienceData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    sample_id(_alloc),
    sample_location(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sample_id = "";
      this->ph_level = 0.0f;
      this->temperature = 0.0f;
      this->moisture = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _sample_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sample_id_type sample_id;
  using _sample_location_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _sample_location_type sample_location;
  using _ph_level_type =
    float;
  _ph_level_type ph_level;
  using _temperature_type =
    float;
  _temperature_type temperature;
  using _moisture_type =
    float;
  _moisture_type moisture;
  using _detected_compounds_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _detected_compounds_type detected_compounds;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__sample_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sample_id = _arg;
    return *this;
  }
  Type & set__sample_location(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->sample_location = _arg;
    return *this;
  }
  Type & set__ph_level(
    const float & _arg)
  {
    this->ph_level = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__moisture(
    const float & _arg)
  {
    this->moisture = _arg;
    return *this;
  }
  Type & set__detected_compounds(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->detected_compounds = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    urc_msgs::msg::ScienceData_<ContainerAllocator> *;
  using ConstRawPtr =
    const urc_msgs::msg::ScienceData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<urc_msgs::msg::ScienceData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<urc_msgs::msg::ScienceData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      urc_msgs::msg::ScienceData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<urc_msgs::msg::ScienceData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      urc_msgs::msg::ScienceData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<urc_msgs::msg::ScienceData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<urc_msgs::msg::ScienceData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<urc_msgs::msg::ScienceData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__urc_msgs__msg__ScienceData
    std::shared_ptr<urc_msgs::msg::ScienceData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__urc_msgs__msg__ScienceData
    std::shared_ptr<urc_msgs::msg::ScienceData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ScienceData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->sample_id != other.sample_id) {
      return false;
    }
    if (this->sample_location != other.sample_location) {
      return false;
    }
    if (this->ph_level != other.ph_level) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->moisture != other.moisture) {
      return false;
    }
    if (this->detected_compounds != other.detected_compounds) {
      return false;
    }
    return true;
  }
  bool operator!=(const ScienceData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ScienceData_

// alias to use template instance with default allocator
using ScienceData =
  urc_msgs::msg::ScienceData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace urc_msgs

#endif  // URC_MSGS__MSG__DETAIL__SCIENCE_DATA__STRUCT_HPP_
