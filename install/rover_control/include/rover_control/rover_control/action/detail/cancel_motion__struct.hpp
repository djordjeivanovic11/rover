// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_control:action/CancelMotion.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__ACTION__DETAIL__CANCEL_MOTION__STRUCT_HPP_
#define ROVER_CONTROL__ACTION__DETAIL__CANCEL_MOTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_control__action__CancelMotion_Goal __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__CancelMotion_Goal __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CancelMotion_Goal_
{
  using Type = CancelMotion_Goal_<ContainerAllocator>;

  explicit CancelMotion_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->emergency_stop = false;
      this->deceleration_time = 1.0f;
      this->cancel_all_actions = true;
      this->reason = "user_request";
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->emergency_stop = false;
      this->deceleration_time = 0.0f;
      this->cancel_all_actions = false;
      this->reason = "";
    }
  }

  explicit CancelMotion_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->emergency_stop = false;
      this->deceleration_time = 1.0f;
      this->cancel_all_actions = true;
      this->reason = "user_request";
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->emergency_stop = false;
      this->deceleration_time = 0.0f;
      this->cancel_all_actions = false;
      this->reason = "";
    }
  }

  // field types and members
  using _emergency_stop_type =
    bool;
  _emergency_stop_type emergency_stop;
  using _deceleration_time_type =
    float;
  _deceleration_time_type deceleration_time;
  using _cancel_all_actions_type =
    bool;
  _cancel_all_actions_type cancel_all_actions;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;

  // setters for named parameter idiom
  Type & set__emergency_stop(
    const bool & _arg)
  {
    this->emergency_stop = _arg;
    return *this;
  }
  Type & set__deceleration_time(
    const float & _arg)
  {
    this->deceleration_time = _arg;
    return *this;
  }
  Type & set__cancel_all_actions(
    const bool & _arg)
  {
    this->cancel_all_actions = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_control::action::CancelMotion_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::CancelMotion_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__CancelMotion_Goal
    std::shared_ptr<rover_control::action::CancelMotion_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__CancelMotion_Goal
    std::shared_ptr<rover_control::action::CancelMotion_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CancelMotion_Goal_ & other) const
  {
    if (this->emergency_stop != other.emergency_stop) {
      return false;
    }
    if (this->deceleration_time != other.deceleration_time) {
      return false;
    }
    if (this->cancel_all_actions != other.cancel_all_actions) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    return true;
  }
  bool operator!=(const CancelMotion_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CancelMotion_Goal_

// alias to use template instance with default allocator
using CancelMotion_Goal =
  rover_control::action::CancelMotion_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control


// Include directives for member types
// Member 'velocity_at_stop'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'final_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__CancelMotion_Result __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__CancelMotion_Result __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CancelMotion_Result_
{
  using Type = CancelMotion_Result_<ContainerAllocator>;

  explicit CancelMotion_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : velocity_at_stop(_init),
    final_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result_code = 0l;
      this->error_message = "";
      this->stop_time = 0.0f;
      this->actions_cancelled = 0l;
    }
  }

  explicit CancelMotion_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_message(_alloc),
    velocity_at_stop(_alloc, _init),
    final_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result_code = 0l;
      this->error_message = "";
      this->stop_time = 0.0f;
      this->actions_cancelled = 0l;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _result_code_type =
    int32_t;
  _result_code_type result_code;
  using _error_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_message_type error_message;
  using _stop_time_type =
    float;
  _stop_time_type stop_time;
  using _velocity_at_stop_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _velocity_at_stop_type velocity_at_stop;
  using _final_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _final_pose_type final_pose;
  using _actions_cancelled_type =
    int32_t;
  _actions_cancelled_type actions_cancelled;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__result_code(
    const int32_t & _arg)
  {
    this->result_code = _arg;
    return *this;
  }
  Type & set__error_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error_message = _arg;
    return *this;
  }
  Type & set__stop_time(
    const float & _arg)
  {
    this->stop_time = _arg;
    return *this;
  }
  Type & set__velocity_at_stop(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->velocity_at_stop = _arg;
    return *this;
  }
  Type & set__final_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->final_pose = _arg;
    return *this;
  }
  Type & set__actions_cancelled(
    const int32_t & _arg)
  {
    this->actions_cancelled = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_control::action::CancelMotion_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::CancelMotion_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__CancelMotion_Result
    std::shared_ptr<rover_control::action::CancelMotion_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__CancelMotion_Result
    std::shared_ptr<rover_control::action::CancelMotion_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CancelMotion_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->result_code != other.result_code) {
      return false;
    }
    if (this->error_message != other.error_message) {
      return false;
    }
    if (this->stop_time != other.stop_time) {
      return false;
    }
    if (this->velocity_at_stop != other.velocity_at_stop) {
      return false;
    }
    if (this->final_pose != other.final_pose) {
      return false;
    }
    if (this->actions_cancelled != other.actions_cancelled) {
      return false;
    }
    return true;
  }
  bool operator!=(const CancelMotion_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CancelMotion_Result_

// alias to use template instance with default allocator
using CancelMotion_Result =
  rover_control::action::CancelMotion_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control


// Include directives for member types
// Member 'current_velocity'
// already included above
// #include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__CancelMotion_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__CancelMotion_Feedback __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CancelMotion_Feedback_
{
  using Type = CancelMotion_Feedback_<ContainerAllocator>;

  explicit CancelMotion_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_velocity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_state = "";
      this->progress_percentage = 0.0f;
      this->status_message = "";
    }
  }

  explicit CancelMotion_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_state(_alloc),
    current_velocity(_alloc, _init),
    status_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_state = "";
      this->progress_percentage = 0.0f;
      this->status_message = "";
    }
  }

  // field types and members
  using _current_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_state_type current_state;
  using _current_velocity_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _current_velocity_type current_velocity;
  using _progress_percentage_type =
    float;
  _progress_percentage_type progress_percentage;
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
  Type & set__current_velocity(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->current_velocity = _arg;
    return *this;
  }
  Type & set__progress_percentage(
    const float & _arg)
  {
    this->progress_percentage = _arg;
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
    rover_control::action::CancelMotion_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::CancelMotion_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__CancelMotion_Feedback
    std::shared_ptr<rover_control::action::CancelMotion_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__CancelMotion_Feedback
    std::shared_ptr<rover_control::action::CancelMotion_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CancelMotion_Feedback_ & other) const
  {
    if (this->current_state != other.current_state) {
      return false;
    }
    if (this->current_velocity != other.current_velocity) {
      return false;
    }
    if (this->progress_percentage != other.progress_percentage) {
      return false;
    }
    if (this->status_message != other.status_message) {
      return false;
    }
    return true;
  }
  bool operator!=(const CancelMotion_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CancelMotion_Feedback_

// alias to use template instance with default allocator
using CancelMotion_Feedback =
  rover_control::action::CancelMotion_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "rover_control/action/detail/cancel_motion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__CancelMotion_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__CancelMotion_SendGoal_Request __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CancelMotion_SendGoal_Request_
{
  using Type = CancelMotion_SendGoal_Request_<ContainerAllocator>;

  explicit CancelMotion_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit CancelMotion_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rover_control::action::CancelMotion_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const rover_control::action::CancelMotion_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_control::action::CancelMotion_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::CancelMotion_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__CancelMotion_SendGoal_Request
    std::shared_ptr<rover_control::action::CancelMotion_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__CancelMotion_SendGoal_Request
    std::shared_ptr<rover_control::action::CancelMotion_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CancelMotion_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const CancelMotion_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CancelMotion_SendGoal_Request_

// alias to use template instance with default allocator
using CancelMotion_SendGoal_Request =
  rover_control::action::CancelMotion_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__CancelMotion_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__CancelMotion_SendGoal_Response __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CancelMotion_SendGoal_Response_
{
  using Type = CancelMotion_SendGoal_Response_<ContainerAllocator>;

  explicit CancelMotion_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit CancelMotion_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rover_control::action::CancelMotion_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::CancelMotion_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__CancelMotion_SendGoal_Response
    std::shared_ptr<rover_control::action::CancelMotion_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__CancelMotion_SendGoal_Response
    std::shared_ptr<rover_control::action::CancelMotion_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CancelMotion_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const CancelMotion_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CancelMotion_SendGoal_Response_

// alias to use template instance with default allocator
using CancelMotion_SendGoal_Response =
  rover_control::action::CancelMotion_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control

namespace rover_control
{

namespace action
{

struct CancelMotion_SendGoal
{
  using Request = rover_control::action::CancelMotion_SendGoal_Request;
  using Response = rover_control::action::CancelMotion_SendGoal_Response;
};

}  // namespace action

}  // namespace rover_control


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__CancelMotion_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__CancelMotion_GetResult_Request __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CancelMotion_GetResult_Request_
{
  using Type = CancelMotion_GetResult_Request_<ContainerAllocator>;

  explicit CancelMotion_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit CancelMotion_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rover_control::action::CancelMotion_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::CancelMotion_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__CancelMotion_GetResult_Request
    std::shared_ptr<rover_control::action::CancelMotion_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__CancelMotion_GetResult_Request
    std::shared_ptr<rover_control::action::CancelMotion_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CancelMotion_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const CancelMotion_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CancelMotion_GetResult_Request_

// alias to use template instance with default allocator
using CancelMotion_GetResult_Request =
  rover_control::action::CancelMotion_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control


// Include directives for member types
// Member 'result'
// already included above
// #include "rover_control/action/detail/cancel_motion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__CancelMotion_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__CancelMotion_GetResult_Response __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CancelMotion_GetResult_Response_
{
  using Type = CancelMotion_GetResult_Response_<ContainerAllocator>;

  explicit CancelMotion_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit CancelMotion_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rover_control::action::CancelMotion_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const rover_control::action::CancelMotion_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_control::action::CancelMotion_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::CancelMotion_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__CancelMotion_GetResult_Response
    std::shared_ptr<rover_control::action::CancelMotion_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__CancelMotion_GetResult_Response
    std::shared_ptr<rover_control::action::CancelMotion_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CancelMotion_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const CancelMotion_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CancelMotion_GetResult_Response_

// alias to use template instance with default allocator
using CancelMotion_GetResult_Response =
  rover_control::action::CancelMotion_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control

namespace rover_control
{

namespace action
{

struct CancelMotion_GetResult
{
  using Request = rover_control::action::CancelMotion_GetResult_Request;
  using Response = rover_control::action::CancelMotion_GetResult_Response;
};

}  // namespace action

}  // namespace rover_control


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "rover_control/action/detail/cancel_motion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__CancelMotion_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__CancelMotion_FeedbackMessage __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct CancelMotion_FeedbackMessage_
{
  using Type = CancelMotion_FeedbackMessage_<ContainerAllocator>;

  explicit CancelMotion_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit CancelMotion_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rover_control::action::CancelMotion_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const rover_control::action::CancelMotion_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_control::action::CancelMotion_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::CancelMotion_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::CancelMotion_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::CancelMotion_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::CancelMotion_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::CancelMotion_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__CancelMotion_FeedbackMessage
    std::shared_ptr<rover_control::action::CancelMotion_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__CancelMotion_FeedbackMessage
    std::shared_ptr<rover_control::action::CancelMotion_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CancelMotion_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const CancelMotion_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CancelMotion_FeedbackMessage_

// alias to use template instance with default allocator
using CancelMotion_FeedbackMessage =
  rover_control::action::CancelMotion_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace rover_control
{

namespace action
{

struct CancelMotion
{
  /// The goal message defined in the action definition.
  using Goal = rover_control::action::CancelMotion_Goal;
  /// The result message defined in the action definition.
  using Result = rover_control::action::CancelMotion_Result;
  /// The feedback message defined in the action definition.
  using Feedback = rover_control::action::CancelMotion_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = rover_control::action::CancelMotion_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = rover_control::action::CancelMotion_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = rover_control::action::CancelMotion_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct CancelMotion CancelMotion;

}  // namespace action

}  // namespace rover_control

#endif  // ROVER_CONTROL__ACTION__DETAIL__CANCEL_MOTION__STRUCT_HPP_
