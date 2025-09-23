// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arm_control:action/GoToNamedPose.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__ACTION__DETAIL__GO_TO_NAMED_POSE__STRUCT_HPP_
#define ARM_CONTROL__ACTION__DETAIL__GO_TO_NAMED_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__arm_control__action__GoToNamedPose_Goal __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__GoToNamedPose_Goal __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToNamedPose_Goal_
{
  using Type = GoToNamedPose_Goal_<ContainerAllocator>;

  explicit GoToNamedPose_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->velocity_scaling = 0.1f;
      this->acceleration_scaling = 0.1f;
      this->plan_only = false;
      this->planning_timeout = 5.0f;
      this->execution_timeout = 30.0f;
      this->collision_checking = true;
      this->planning_group = "arm";
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->pose_name = "";
      this->velocity_scaling = 0.0f;
      this->acceleration_scaling = 0.0f;
      this->plan_only = false;
      this->planning_timeout = 0.0f;
      this->execution_timeout = 0.0f;
      this->collision_checking = false;
      this->planning_group = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pose_name = "";
    }
  }

  explicit GoToNamedPose_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose_name(_alloc),
    planning_group(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->velocity_scaling = 0.1f;
      this->acceleration_scaling = 0.1f;
      this->plan_only = false;
      this->planning_timeout = 5.0f;
      this->execution_timeout = 30.0f;
      this->collision_checking = true;
      this->planning_group = "arm";
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->pose_name = "";
      this->velocity_scaling = 0.0f;
      this->acceleration_scaling = 0.0f;
      this->plan_only = false;
      this->planning_timeout = 0.0f;
      this->execution_timeout = 0.0f;
      this->collision_checking = false;
      this->planning_group = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pose_name = "";
    }
  }

  // field types and members
  using _pose_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _pose_name_type pose_name;
  using _velocity_scaling_type =
    float;
  _velocity_scaling_type velocity_scaling;
  using _acceleration_scaling_type =
    float;
  _acceleration_scaling_type acceleration_scaling;
  using _plan_only_type =
    bool;
  _plan_only_type plan_only;
  using _planning_timeout_type =
    float;
  _planning_timeout_type planning_timeout;
  using _execution_timeout_type =
    float;
  _execution_timeout_type execution_timeout;
  using _collision_checking_type =
    bool;
  _collision_checking_type collision_checking;
  using _planning_group_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _planning_group_type planning_group;

  // setters for named parameter idiom
  Type & set__pose_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->pose_name = _arg;
    return *this;
  }
  Type & set__velocity_scaling(
    const float & _arg)
  {
    this->velocity_scaling = _arg;
    return *this;
  }
  Type & set__acceleration_scaling(
    const float & _arg)
  {
    this->acceleration_scaling = _arg;
    return *this;
  }
  Type & set__plan_only(
    const bool & _arg)
  {
    this->plan_only = _arg;
    return *this;
  }
  Type & set__planning_timeout(
    const float & _arg)
  {
    this->planning_timeout = _arg;
    return *this;
  }
  Type & set__execution_timeout(
    const float & _arg)
  {
    this->execution_timeout = _arg;
    return *this;
  }
  Type & set__collision_checking(
    const bool & _arg)
  {
    this->collision_checking = _arg;
    return *this;
  }
  Type & set__planning_group(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->planning_group = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::GoToNamedPose_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::GoToNamedPose_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__GoToNamedPose_Goal
    std::shared_ptr<arm_control::action::GoToNamedPose_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__GoToNamedPose_Goal
    std::shared_ptr<arm_control::action::GoToNamedPose_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToNamedPose_Goal_ & other) const
  {
    if (this->pose_name != other.pose_name) {
      return false;
    }
    if (this->velocity_scaling != other.velocity_scaling) {
      return false;
    }
    if (this->acceleration_scaling != other.acceleration_scaling) {
      return false;
    }
    if (this->plan_only != other.plan_only) {
      return false;
    }
    if (this->planning_timeout != other.planning_timeout) {
      return false;
    }
    if (this->execution_timeout != other.execution_timeout) {
      return false;
    }
    if (this->collision_checking != other.collision_checking) {
      return false;
    }
    if (this->planning_group != other.planning_group) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToNamedPose_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToNamedPose_Goal_

// alias to use template instance with default allocator
using GoToNamedPose_Goal =
  arm_control::action::GoToNamedPose_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'final_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__GoToNamedPose_Result __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__GoToNamedPose_Result __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToNamedPose_Result_
{
  using Type = GoToNamedPose_Result_<ContainerAllocator>;

  explicit GoToNamedPose_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : final_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_message = "";
      this->error_code = 0l;
      this->planning_time = 0.0f;
      this->execution_time = 0.0f;
      this->final_position_error = 0.0f;
      this->final_orientation_error = 0.0f;
    }
  }

  explicit GoToNamedPose_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_message(_alloc),
    final_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_message = "";
      this->error_code = 0l;
      this->planning_time = 0.0f;
      this->execution_time = 0.0f;
      this->final_position_error = 0.0f;
      this->final_orientation_error = 0.0f;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _error_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_message_type error_message;
  using _error_code_type =
    int32_t;
  _error_code_type error_code;
  using _final_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _final_pose_type final_pose;
  using _planning_time_type =
    float;
  _planning_time_type planning_time;
  using _execution_time_type =
    float;
  _execution_time_type execution_time;
  using _final_position_error_type =
    float;
  _final_position_error_type final_position_error;
  using _final_orientation_error_type =
    float;
  _final_orientation_error_type final_orientation_error;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__error_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error_message = _arg;
    return *this;
  }
  Type & set__error_code(
    const int32_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__final_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->final_pose = _arg;
    return *this;
  }
  Type & set__planning_time(
    const float & _arg)
  {
    this->planning_time = _arg;
    return *this;
  }
  Type & set__execution_time(
    const float & _arg)
  {
    this->execution_time = _arg;
    return *this;
  }
  Type & set__final_position_error(
    const float & _arg)
  {
    this->final_position_error = _arg;
    return *this;
  }
  Type & set__final_orientation_error(
    const float & _arg)
  {
    this->final_orientation_error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::GoToNamedPose_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::GoToNamedPose_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__GoToNamedPose_Result
    std::shared_ptr<arm_control::action::GoToNamedPose_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__GoToNamedPose_Result
    std::shared_ptr<arm_control::action::GoToNamedPose_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToNamedPose_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->error_message != other.error_message) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->final_pose != other.final_pose) {
      return false;
    }
    if (this->planning_time != other.planning_time) {
      return false;
    }
    if (this->execution_time != other.execution_time) {
      return false;
    }
    if (this->final_position_error != other.final_position_error) {
      return false;
    }
    if (this->final_orientation_error != other.final_orientation_error) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToNamedPose_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToNamedPose_Result_

// alias to use template instance with default allocator
using GoToNamedPose_Result =
  arm_control::action::GoToNamedPose_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'current_joint_state'
#include "sensor_msgs/msg/detail/joint_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__GoToNamedPose_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__GoToNamedPose_Feedback __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToNamedPose_Feedback_
{
  using Type = GoToNamedPose_Feedback_<ContainerAllocator>;

  explicit GoToNamedPose_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_pose(_init),
    current_joint_state(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_state = "";
      this->progress_percentage = 0.0f;
      this->estimated_time_remaining = 0.0f;
      this->collision_detected = false;
      this->status_message = "";
    }
  }

  explicit GoToNamedPose_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_state(_alloc),
    current_pose(_alloc, _init),
    current_joint_state(_alloc, _init),
    status_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_state = "";
      this->progress_percentage = 0.0f;
      this->estimated_time_remaining = 0.0f;
      this->collision_detected = false;
      this->status_message = "";
    }
  }

  // field types and members
  using _current_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_state_type current_state;
  using _progress_percentage_type =
    float;
  _progress_percentage_type progress_percentage;
  using _current_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _current_pose_type current_pose;
  using _estimated_time_remaining_type =
    float;
  _estimated_time_remaining_type estimated_time_remaining;
  using _current_joint_state_type =
    sensor_msgs::msg::JointState_<ContainerAllocator>;
  _current_joint_state_type current_joint_state;
  using _collision_detected_type =
    bool;
  _collision_detected_type collision_detected;
  using _status_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_message_type status_message;

  // setters for named parameter idiom
  Type & set__current_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_state = _arg;
    return *this;
  }
  Type & set__progress_percentage(
    const float & _arg)
  {
    this->progress_percentage = _arg;
    return *this;
  }
  Type & set__current_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->current_pose = _arg;
    return *this;
  }
  Type & set__estimated_time_remaining(
    const float & _arg)
  {
    this->estimated_time_remaining = _arg;
    return *this;
  }
  Type & set__current_joint_state(
    const sensor_msgs::msg::JointState_<ContainerAllocator> & _arg)
  {
    this->current_joint_state = _arg;
    return *this;
  }
  Type & set__collision_detected(
    const bool & _arg)
  {
    this->collision_detected = _arg;
    return *this;
  }
  Type & set__status_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status_message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::GoToNamedPose_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::GoToNamedPose_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__GoToNamedPose_Feedback
    std::shared_ptr<arm_control::action::GoToNamedPose_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__GoToNamedPose_Feedback
    std::shared_ptr<arm_control::action::GoToNamedPose_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToNamedPose_Feedback_ & other) const
  {
    if (this->current_state != other.current_state) {
      return false;
    }
    if (this->progress_percentage != other.progress_percentage) {
      return false;
    }
    if (this->current_pose != other.current_pose) {
      return false;
    }
    if (this->estimated_time_remaining != other.estimated_time_remaining) {
      return false;
    }
    if (this->current_joint_state != other.current_joint_state) {
      return false;
    }
    if (this->collision_detected != other.collision_detected) {
      return false;
    }
    if (this->status_message != other.status_message) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToNamedPose_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToNamedPose_Feedback_

// alias to use template instance with default allocator
using GoToNamedPose_Feedback =
  arm_control::action::GoToNamedPose_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "arm_control/action/detail/go_to_named_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__GoToNamedPose_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__GoToNamedPose_SendGoal_Request __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToNamedPose_SendGoal_Request_
{
  using Type = GoToNamedPose_SendGoal_Request_<ContainerAllocator>;

  explicit GoToNamedPose_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit GoToNamedPose_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    arm_control::action::GoToNamedPose_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const arm_control::action::GoToNamedPose_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::GoToNamedPose_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::GoToNamedPose_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__GoToNamedPose_SendGoal_Request
    std::shared_ptr<arm_control::action::GoToNamedPose_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__GoToNamedPose_SendGoal_Request
    std::shared_ptr<arm_control::action::GoToNamedPose_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToNamedPose_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToNamedPose_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToNamedPose_SendGoal_Request_

// alias to use template instance with default allocator
using GoToNamedPose_SendGoal_Request =
  arm_control::action::GoToNamedPose_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__GoToNamedPose_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__GoToNamedPose_SendGoal_Response __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToNamedPose_SendGoal_Response_
{
  using Type = GoToNamedPose_SendGoal_Response_<ContainerAllocator>;

  explicit GoToNamedPose_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit GoToNamedPose_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::GoToNamedPose_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::GoToNamedPose_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__GoToNamedPose_SendGoal_Response
    std::shared_ptr<arm_control::action::GoToNamedPose_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__GoToNamedPose_SendGoal_Response
    std::shared_ptr<arm_control::action::GoToNamedPose_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToNamedPose_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToNamedPose_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToNamedPose_SendGoal_Response_

// alias to use template instance with default allocator
using GoToNamedPose_SendGoal_Response =
  arm_control::action::GoToNamedPose_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control

namespace arm_control
{

namespace action
{

struct GoToNamedPose_SendGoal
{
  using Request = arm_control::action::GoToNamedPose_SendGoal_Request;
  using Response = arm_control::action::GoToNamedPose_SendGoal_Response;
};

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__GoToNamedPose_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__GoToNamedPose_GetResult_Request __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToNamedPose_GetResult_Request_
{
  using Type = GoToNamedPose_GetResult_Request_<ContainerAllocator>;

  explicit GoToNamedPose_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit GoToNamedPose_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::GoToNamedPose_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::GoToNamedPose_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__GoToNamedPose_GetResult_Request
    std::shared_ptr<arm_control::action::GoToNamedPose_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__GoToNamedPose_GetResult_Request
    std::shared_ptr<arm_control::action::GoToNamedPose_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToNamedPose_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToNamedPose_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToNamedPose_GetResult_Request_

// alias to use template instance with default allocator
using GoToNamedPose_GetResult_Request =
  arm_control::action::GoToNamedPose_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'result'
// already included above
// #include "arm_control/action/detail/go_to_named_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__GoToNamedPose_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__GoToNamedPose_GetResult_Response __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToNamedPose_GetResult_Response_
{
  using Type = GoToNamedPose_GetResult_Response_<ContainerAllocator>;

  explicit GoToNamedPose_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit GoToNamedPose_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    arm_control::action::GoToNamedPose_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const arm_control::action::GoToNamedPose_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::GoToNamedPose_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::GoToNamedPose_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__GoToNamedPose_GetResult_Response
    std::shared_ptr<arm_control::action::GoToNamedPose_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__GoToNamedPose_GetResult_Response
    std::shared_ptr<arm_control::action::GoToNamedPose_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToNamedPose_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToNamedPose_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToNamedPose_GetResult_Response_

// alias to use template instance with default allocator
using GoToNamedPose_GetResult_Response =
  arm_control::action::GoToNamedPose_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control

namespace arm_control
{

namespace action
{

struct GoToNamedPose_GetResult
{
  using Request = arm_control::action::GoToNamedPose_GetResult_Request;
  using Response = arm_control::action::GoToNamedPose_GetResult_Response;
};

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "arm_control/action/detail/go_to_named_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__GoToNamedPose_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__GoToNamedPose_FeedbackMessage __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToNamedPose_FeedbackMessage_
{
  using Type = GoToNamedPose_FeedbackMessage_<ContainerAllocator>;

  explicit GoToNamedPose_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit GoToNamedPose_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    arm_control::action::GoToNamedPose_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const arm_control::action::GoToNamedPose_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::GoToNamedPose_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::GoToNamedPose_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::GoToNamedPose_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::GoToNamedPose_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::GoToNamedPose_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::GoToNamedPose_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__GoToNamedPose_FeedbackMessage
    std::shared_ptr<arm_control::action::GoToNamedPose_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__GoToNamedPose_FeedbackMessage
    std::shared_ptr<arm_control::action::GoToNamedPose_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToNamedPose_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToNamedPose_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToNamedPose_FeedbackMessage_

// alias to use template instance with default allocator
using GoToNamedPose_FeedbackMessage =
  arm_control::action::GoToNamedPose_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace arm_control
{

namespace action
{

struct GoToNamedPose
{
  /// The goal message defined in the action definition.
  using Goal = arm_control::action::GoToNamedPose_Goal;
  /// The result message defined in the action definition.
  using Result = arm_control::action::GoToNamedPose_Result;
  /// The feedback message defined in the action definition.
  using Feedback = arm_control::action::GoToNamedPose_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = arm_control::action::GoToNamedPose_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = arm_control::action::GoToNamedPose_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = arm_control::action::GoToNamedPose_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct GoToNamedPose GoToNamedPose;

}  // namespace action

}  // namespace arm_control

#endif  // ARM_CONTROL__ACTION__DETAIL__GO_TO_NAMED_POSE__STRUCT_HPP_
