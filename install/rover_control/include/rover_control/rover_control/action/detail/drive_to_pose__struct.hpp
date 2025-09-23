// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_control:action/DriveToPose.idl
// generated code does not contain a copyright notice

#ifndef ROVER_CONTROL__ACTION__DETAIL__DRIVE_TO_POSE__STRUCT_HPP_
#define ROVER_CONTROL__ACTION__DETAIL__DRIVE_TO_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__DriveToPose_Goal __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__DriveToPose_Goal __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct DriveToPose_Goal_
{
  using Type = DriveToPose_Goal_<ContainerAllocator>;

  explicit DriveToPose_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->max_velocity = 1.0f;
      this->position_tolerance = 0.1f;
      this->orientation_tolerance = 0.1f;
      this->timeout = 60.0f;
      this->use_visual_servoing = false;
      this->mission_mode = "exploration";
      this->reverse_allowed = false;
      this->approach_velocity = 0.5f;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->max_velocity = 0.0f;
      this->position_tolerance = 0.0f;
      this->orientation_tolerance = 0.0f;
      this->timeout = 0.0f;
      this->use_visual_servoing = false;
      this->mission_mode = "";
      this->reverse_allowed = false;
      this->approach_velocity = 0.0f;
    }
  }

  explicit DriveToPose_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_pose(_alloc, _init),
    mission_mode(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->max_velocity = 1.0f;
      this->position_tolerance = 0.1f;
      this->orientation_tolerance = 0.1f;
      this->timeout = 60.0f;
      this->use_visual_servoing = false;
      this->mission_mode = "exploration";
      this->reverse_allowed = false;
      this->approach_velocity = 0.5f;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->max_velocity = 0.0f;
      this->position_tolerance = 0.0f;
      this->orientation_tolerance = 0.0f;
      this->timeout = 0.0f;
      this->use_visual_servoing = false;
      this->mission_mode = "";
      this->reverse_allowed = false;
      this->approach_velocity = 0.0f;
    }
  }

  // field types and members
  using _target_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _target_pose_type target_pose;
  using _max_velocity_type =
    float;
  _max_velocity_type max_velocity;
  using _position_tolerance_type =
    float;
  _position_tolerance_type position_tolerance;
  using _orientation_tolerance_type =
    float;
  _orientation_tolerance_type orientation_tolerance;
  using _timeout_type =
    float;
  _timeout_type timeout;
  using _use_visual_servoing_type =
    bool;
  _use_visual_servoing_type use_visual_servoing;
  using _mission_mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_mode_type mission_mode;
  using _reverse_allowed_type =
    bool;
  _reverse_allowed_type reverse_allowed;
  using _approach_velocity_type =
    float;
  _approach_velocity_type approach_velocity;

  // setters for named parameter idiom
  Type & set__target_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->target_pose = _arg;
    return *this;
  }
  Type & set__max_velocity(
    const float & _arg)
  {
    this->max_velocity = _arg;
    return *this;
  }
  Type & set__position_tolerance(
    const float & _arg)
  {
    this->position_tolerance = _arg;
    return *this;
  }
  Type & set__orientation_tolerance(
    const float & _arg)
  {
    this->orientation_tolerance = _arg;
    return *this;
  }
  Type & set__timeout(
    const float & _arg)
  {
    this->timeout = _arg;
    return *this;
  }
  Type & set__use_visual_servoing(
    const bool & _arg)
  {
    this->use_visual_servoing = _arg;
    return *this;
  }
  Type & set__mission_mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_mode = _arg;
    return *this;
  }
  Type & set__reverse_allowed(
    const bool & _arg)
  {
    this->reverse_allowed = _arg;
    return *this;
  }
  Type & set__approach_velocity(
    const float & _arg)
  {
    this->approach_velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_control::action::DriveToPose_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::DriveToPose_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__DriveToPose_Goal
    std::shared_ptr<rover_control::action::DriveToPose_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__DriveToPose_Goal
    std::shared_ptr<rover_control::action::DriveToPose_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriveToPose_Goal_ & other) const
  {
    if (this->target_pose != other.target_pose) {
      return false;
    }
    if (this->max_velocity != other.max_velocity) {
      return false;
    }
    if (this->position_tolerance != other.position_tolerance) {
      return false;
    }
    if (this->orientation_tolerance != other.orientation_tolerance) {
      return false;
    }
    if (this->timeout != other.timeout) {
      return false;
    }
    if (this->use_visual_servoing != other.use_visual_servoing) {
      return false;
    }
    if (this->mission_mode != other.mission_mode) {
      return false;
    }
    if (this->reverse_allowed != other.reverse_allowed) {
      return false;
    }
    if (this->approach_velocity != other.approach_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriveToPose_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriveToPose_Goal_

// alias to use template instance with default allocator
using DriveToPose_Goal =
  rover_control::action::DriveToPose_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control


// Include directives for member types
// Member 'final_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'executed_path'
#include "nav_msgs/msg/detail/path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__DriveToPose_Result __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__DriveToPose_Result __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct DriveToPose_Result_
{
  using Type = DriveToPose_Result_<ContainerAllocator>;

  explicit DriveToPose_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : final_pose(_init),
    executed_path(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result_code = 0l;
      this->error_message = "";
      this->final_position_error = 0.0f;
      this->final_orientation_error = 0.0f;
      this->execution_time = 0.0f;
      this->distance_traveled = 0.0f;
    }
  }

  explicit DriveToPose_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_message(_alloc),
    final_pose(_alloc, _init),
    executed_path(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->result_code = 0l;
      this->error_message = "";
      this->final_position_error = 0.0f;
      this->final_orientation_error = 0.0f;
      this->execution_time = 0.0f;
      this->distance_traveled = 0.0f;
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
  using _final_position_error_type =
    float;
  _final_position_error_type final_position_error;
  using _final_orientation_error_type =
    float;
  _final_orientation_error_type final_orientation_error;
  using _execution_time_type =
    float;
  _execution_time_type execution_time;
  using _distance_traveled_type =
    float;
  _distance_traveled_type distance_traveled;
  using _final_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _final_pose_type final_pose;
  using _executed_path_type =
    nav_msgs::msg::Path_<ContainerAllocator>;
  _executed_path_type executed_path;

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
  Type & set__execution_time(
    const float & _arg)
  {
    this->execution_time = _arg;
    return *this;
  }
  Type & set__distance_traveled(
    const float & _arg)
  {
    this->distance_traveled = _arg;
    return *this;
  }
  Type & set__final_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->final_pose = _arg;
    return *this;
  }
  Type & set__executed_path(
    const nav_msgs::msg::Path_<ContainerAllocator> & _arg)
  {
    this->executed_path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_control::action::DriveToPose_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::DriveToPose_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__DriveToPose_Result
    std::shared_ptr<rover_control::action::DriveToPose_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__DriveToPose_Result
    std::shared_ptr<rover_control::action::DriveToPose_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriveToPose_Result_ & other) const
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
    if (this->final_position_error != other.final_position_error) {
      return false;
    }
    if (this->final_orientation_error != other.final_orientation_error) {
      return false;
    }
    if (this->execution_time != other.execution_time) {
      return false;
    }
    if (this->distance_traveled != other.distance_traveled) {
      return false;
    }
    if (this->final_pose != other.final_pose) {
      return false;
    }
    if (this->executed_path != other.executed_path) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriveToPose_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriveToPose_Result_

// alias to use template instance with default allocator
using DriveToPose_Result =
  rover_control::action::DriveToPose_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control


// Include directives for member types
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__DriveToPose_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__DriveToPose_Feedback __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct DriveToPose_Feedback_
{
  using Type = DriveToPose_Feedback_<ContainerAllocator>;

  explicit DriveToPose_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_state = "";
      this->progress_percentage = 0.0f;
      this->estimated_time_remaining = 0.0f;
      this->distance_remaining = 0.0f;
      this->current_velocity = 0.0f;
      this->status_message = "";
      this->stuck_detected = false;
      this->autonomy_time_remaining = 0.0f;
    }
  }

  explicit DriveToPose_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_state(_alloc),
    current_pose(_alloc, _init),
    status_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_state = "";
      this->progress_percentage = 0.0f;
      this->estimated_time_remaining = 0.0f;
      this->distance_remaining = 0.0f;
      this->current_velocity = 0.0f;
      this->status_message = "";
      this->stuck_detected = false;
      this->autonomy_time_remaining = 0.0f;
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
  using _distance_remaining_type =
    float;
  _distance_remaining_type distance_remaining;
  using _current_velocity_type =
    float;
  _current_velocity_type current_velocity;
  using _status_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_message_type status_message;
  using _stuck_detected_type =
    bool;
  _stuck_detected_type stuck_detected;
  using _autonomy_time_remaining_type =
    float;
  _autonomy_time_remaining_type autonomy_time_remaining;

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
  Type & set__distance_remaining(
    const float & _arg)
  {
    this->distance_remaining = _arg;
    return *this;
  }
  Type & set__current_velocity(
    const float & _arg)
  {
    this->current_velocity = _arg;
    return *this;
  }
  Type & set__status_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status_message = _arg;
    return *this;
  }
  Type & set__stuck_detected(
    const bool & _arg)
  {
    this->stuck_detected = _arg;
    return *this;
  }
  Type & set__autonomy_time_remaining(
    const float & _arg)
  {
    this->autonomy_time_remaining = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_control::action::DriveToPose_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::DriveToPose_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__DriveToPose_Feedback
    std::shared_ptr<rover_control::action::DriveToPose_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__DriveToPose_Feedback
    std::shared_ptr<rover_control::action::DriveToPose_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriveToPose_Feedback_ & other) const
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
    if (this->distance_remaining != other.distance_remaining) {
      return false;
    }
    if (this->current_velocity != other.current_velocity) {
      return false;
    }
    if (this->status_message != other.status_message) {
      return false;
    }
    if (this->stuck_detected != other.stuck_detected) {
      return false;
    }
    if (this->autonomy_time_remaining != other.autonomy_time_remaining) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriveToPose_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriveToPose_Feedback_

// alias to use template instance with default allocator
using DriveToPose_Feedback =
  rover_control::action::DriveToPose_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "rover_control/action/detail/drive_to_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__DriveToPose_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__DriveToPose_SendGoal_Request __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct DriveToPose_SendGoal_Request_
{
  using Type = DriveToPose_SendGoal_Request_<ContainerAllocator>;

  explicit DriveToPose_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit DriveToPose_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rover_control::action::DriveToPose_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const rover_control::action::DriveToPose_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_control::action::DriveToPose_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::DriveToPose_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__DriveToPose_SendGoal_Request
    std::shared_ptr<rover_control::action::DriveToPose_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__DriveToPose_SendGoal_Request
    std::shared_ptr<rover_control::action::DriveToPose_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriveToPose_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriveToPose_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriveToPose_SendGoal_Request_

// alias to use template instance with default allocator
using DriveToPose_SendGoal_Request =
  rover_control::action::DriveToPose_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__DriveToPose_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__DriveToPose_SendGoal_Response __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct DriveToPose_SendGoal_Response_
{
  using Type = DriveToPose_SendGoal_Response_<ContainerAllocator>;

  explicit DriveToPose_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit DriveToPose_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rover_control::action::DriveToPose_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::DriveToPose_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__DriveToPose_SendGoal_Response
    std::shared_ptr<rover_control::action::DriveToPose_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__DriveToPose_SendGoal_Response
    std::shared_ptr<rover_control::action::DriveToPose_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriveToPose_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriveToPose_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriveToPose_SendGoal_Response_

// alias to use template instance with default allocator
using DriveToPose_SendGoal_Response =
  rover_control::action::DriveToPose_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control

namespace rover_control
{

namespace action
{

struct DriveToPose_SendGoal
{
  using Request = rover_control::action::DriveToPose_SendGoal_Request;
  using Response = rover_control::action::DriveToPose_SendGoal_Response;
};

}  // namespace action

}  // namespace rover_control


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__DriveToPose_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__DriveToPose_GetResult_Request __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct DriveToPose_GetResult_Request_
{
  using Type = DriveToPose_GetResult_Request_<ContainerAllocator>;

  explicit DriveToPose_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit DriveToPose_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rover_control::action::DriveToPose_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::DriveToPose_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__DriveToPose_GetResult_Request
    std::shared_ptr<rover_control::action::DriveToPose_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__DriveToPose_GetResult_Request
    std::shared_ptr<rover_control::action::DriveToPose_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriveToPose_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriveToPose_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriveToPose_GetResult_Request_

// alias to use template instance with default allocator
using DriveToPose_GetResult_Request =
  rover_control::action::DriveToPose_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control


// Include directives for member types
// Member 'result'
// already included above
// #include "rover_control/action/detail/drive_to_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__DriveToPose_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__DriveToPose_GetResult_Response __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct DriveToPose_GetResult_Response_
{
  using Type = DriveToPose_GetResult_Response_<ContainerAllocator>;

  explicit DriveToPose_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit DriveToPose_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rover_control::action::DriveToPose_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const rover_control::action::DriveToPose_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_control::action::DriveToPose_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::DriveToPose_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__DriveToPose_GetResult_Response
    std::shared_ptr<rover_control::action::DriveToPose_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__DriveToPose_GetResult_Response
    std::shared_ptr<rover_control::action::DriveToPose_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriveToPose_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriveToPose_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriveToPose_GetResult_Response_

// alias to use template instance with default allocator
using DriveToPose_GetResult_Response =
  rover_control::action::DriveToPose_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace rover_control

namespace rover_control
{

namespace action
{

struct DriveToPose_GetResult
{
  using Request = rover_control::action::DriveToPose_GetResult_Request;
  using Response = rover_control::action::DriveToPose_GetResult_Response;
};

}  // namespace action

}  // namespace rover_control


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "rover_control/action/detail/drive_to_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rover_control__action__DriveToPose_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__rover_control__action__DriveToPose_FeedbackMessage __declspec(deprecated)
#endif

namespace rover_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct DriveToPose_FeedbackMessage_
{
  using Type = DriveToPose_FeedbackMessage_<ContainerAllocator>;

  explicit DriveToPose_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit DriveToPose_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rover_control::action::DriveToPose_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const rover_control::action::DriveToPose_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_control::action::DriveToPose_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_control::action::DriveToPose_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_control::action::DriveToPose_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_control::action::DriveToPose_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_control::action::DriveToPose_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_control::action::DriveToPose_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_control__action__DriveToPose_FeedbackMessage
    std::shared_ptr<rover_control::action::DriveToPose_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_control__action__DriveToPose_FeedbackMessage
    std::shared_ptr<rover_control::action::DriveToPose_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriveToPose_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriveToPose_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriveToPose_FeedbackMessage_

// alias to use template instance with default allocator
using DriveToPose_FeedbackMessage =
  rover_control::action::DriveToPose_FeedbackMessage_<std::allocator<void>>;

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

struct DriveToPose
{
  /// The goal message defined in the action definition.
  using Goal = rover_control::action::DriveToPose_Goal;
  /// The result message defined in the action definition.
  using Result = rover_control::action::DriveToPose_Result;
  /// The feedback message defined in the action definition.
  using Feedback = rover_control::action::DriveToPose_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = rover_control::action::DriveToPose_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = rover_control::action::DriveToPose_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = rover_control::action::DriveToPose_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct DriveToPose DriveToPose;

}  // namespace action

}  // namespace rover_control

#endif  // ROVER_CONTROL__ACTION__DETAIL__DRIVE_TO_POSE__STRUCT_HPP_
