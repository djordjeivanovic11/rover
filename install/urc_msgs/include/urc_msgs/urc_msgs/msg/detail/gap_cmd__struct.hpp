// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from urc_msgs:msg/GapCmd.idl
// generated code does not contain a copyright notice

#ifndef URC_MSGS__MSG__DETAIL__GAP_CMD__STRUCT_HPP_
#define URC_MSGS__MSG__DETAIL__GAP_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__urc_msgs__msg__GapCmd __attribute__((deprecated))
#else
# define DEPRECATED__urc_msgs__msg__GapCmd __declspec(deprecated)
#endif

namespace urc_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GapCmd_
{
  using Type = GapCmd_<ContainerAllocator>;

  explicit GapCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GapCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    urc_msgs::msg::GapCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const urc_msgs::msg::GapCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<urc_msgs::msg::GapCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<urc_msgs::msg::GapCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      urc_msgs::msg::GapCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<urc_msgs::msg::GapCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      urc_msgs::msg::GapCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<urc_msgs::msg::GapCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<urc_msgs::msg::GapCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<urc_msgs::msg::GapCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__urc_msgs__msg__GapCmd
    std::shared_ptr<urc_msgs::msg::GapCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__urc_msgs__msg__GapCmd
    std::shared_ptr<urc_msgs::msg::GapCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GapCmd_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GapCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GapCmd_

// alias to use template instance with default allocator
using GapCmd =
  urc_msgs::msg::GapCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace urc_msgs

#endif  // URC_MSGS__MSG__DETAIL__GAP_CMD__STRUCT_HPP_
