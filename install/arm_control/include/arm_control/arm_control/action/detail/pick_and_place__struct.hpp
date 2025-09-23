// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arm_control:action/PickAndPlace.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__ACTION__DETAIL__PICK_AND_PLACE__STRUCT_HPP_
#define ARM_CONTROL__ACTION__DETAIL__PICK_AND_PLACE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pick_pose'
// Member 'place_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'approach_offset'
// Member 'retreat_offset'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__PickAndPlace_Goal __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__PickAndPlace_Goal __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickAndPlace_Goal_
{
  using Type = PickAndPlace_Goal_<ContainerAllocator>;

  explicit PickAndPlace_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pick_pose(_init),
    place_pose(_init),
    approach_offset(_init),
    retreat_offset(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->grasp_force = 50.0f;
      this->grasp_timeout = 5.0f;
      this->grasp_strategy = "force";
      this->verify_grasp = true;
      this->lift_height = 0.05f;
      this->place_force = 10.0f;
      this->gentle_place = true;
      this->velocity_scaling = 0.1f;
      this->acceleration_scaling = 0.1f;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->grasp_force = 0.0f;
      this->grasp_timeout = 0.0f;
      this->grasp_strategy = "";
      this->verify_grasp = false;
      this->lift_height = 0.0f;
      this->place_force = 0.0f;
      this->gentle_place = false;
      this->velocity_scaling = 0.0f;
      this->acceleration_scaling = 0.0f;
    }
  }

  explicit PickAndPlace_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pick_pose(_alloc, _init),
    place_pose(_alloc, _init),
    approach_offset(_alloc, _init),
    retreat_offset(_alloc, _init),
    grasp_strategy(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->grasp_force = 50.0f;
      this->grasp_timeout = 5.0f;
      this->grasp_strategy = "force";
      this->verify_grasp = true;
      this->lift_height = 0.05f;
      this->place_force = 10.0f;
      this->gentle_place = true;
      this->velocity_scaling = 0.1f;
      this->acceleration_scaling = 0.1f;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->grasp_force = 0.0f;
      this->grasp_timeout = 0.0f;
      this->grasp_strategy = "";
      this->verify_grasp = false;
      this->lift_height = 0.0f;
      this->place_force = 0.0f;
      this->gentle_place = false;
      this->velocity_scaling = 0.0f;
      this->acceleration_scaling = 0.0f;
    }
  }

  // field types and members
  using _pick_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _pick_pose_type pick_pose;
  using _place_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _place_pose_type place_pose;
  using _approach_offset_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _approach_offset_type approach_offset;
  using _retreat_offset_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _retreat_offset_type retreat_offset;
  using _grasp_force_type =
    float;
  _grasp_force_type grasp_force;
  using _grasp_timeout_type =
    float;
  _grasp_timeout_type grasp_timeout;
  using _grasp_strategy_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _grasp_strategy_type grasp_strategy;
  using _verify_grasp_type =
    bool;
  _verify_grasp_type verify_grasp;
  using _lift_height_type =
    float;
  _lift_height_type lift_height;
  using _place_force_type =
    float;
  _place_force_type place_force;
  using _gentle_place_type =
    bool;
  _gentle_place_type gentle_place;
  using _velocity_scaling_type =
    float;
  _velocity_scaling_type velocity_scaling;
  using _acceleration_scaling_type =
    float;
  _acceleration_scaling_type acceleration_scaling;

  // setters for named parameter idiom
  Type & set__pick_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->pick_pose = _arg;
    return *this;
  }
  Type & set__place_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->place_pose = _arg;
    return *this;
  }
  Type & set__approach_offset(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->approach_offset = _arg;
    return *this;
  }
  Type & set__retreat_offset(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->retreat_offset = _arg;
    return *this;
  }
  Type & set__grasp_force(
    const float & _arg)
  {
    this->grasp_force = _arg;
    return *this;
  }
  Type & set__grasp_timeout(
    const float & _arg)
  {
    this->grasp_timeout = _arg;
    return *this;
  }
  Type & set__grasp_strategy(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->grasp_strategy = _arg;
    return *this;
  }
  Type & set__verify_grasp(
    const bool & _arg)
  {
    this->verify_grasp = _arg;
    return *this;
  }
  Type & set__lift_height(
    const float & _arg)
  {
    this->lift_height = _arg;
    return *this;
  }
  Type & set__place_force(
    const float & _arg)
  {
    this->place_force = _arg;
    return *this;
  }
  Type & set__gentle_place(
    const bool & _arg)
  {
    this->gentle_place = _arg;
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

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::PickAndPlace_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::PickAndPlace_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__PickAndPlace_Goal
    std::shared_ptr<arm_control::action::PickAndPlace_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__PickAndPlace_Goal
    std::shared_ptr<arm_control::action::PickAndPlace_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickAndPlace_Goal_ & other) const
  {
    if (this->pick_pose != other.pick_pose) {
      return false;
    }
    if (this->place_pose != other.place_pose) {
      return false;
    }
    if (this->approach_offset != other.approach_offset) {
      return false;
    }
    if (this->retreat_offset != other.retreat_offset) {
      return false;
    }
    if (this->grasp_force != other.grasp_force) {
      return false;
    }
    if (this->grasp_timeout != other.grasp_timeout) {
      return false;
    }
    if (this->grasp_strategy != other.grasp_strategy) {
      return false;
    }
    if (this->verify_grasp != other.verify_grasp) {
      return false;
    }
    if (this->lift_height != other.lift_height) {
      return false;
    }
    if (this->place_force != other.place_force) {
      return false;
    }
    if (this->gentle_place != other.gentle_place) {
      return false;
    }
    if (this->velocity_scaling != other.velocity_scaling) {
      return false;
    }
    if (this->acceleration_scaling != other.acceleration_scaling) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickAndPlace_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickAndPlace_Goal_

// alias to use template instance with default allocator
using PickAndPlace_Goal =
  arm_control::action::PickAndPlace_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'final_object_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'final_joint_state'
#include "sensor_msgs/msg/detail/joint_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__PickAndPlace_Result __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__PickAndPlace_Result __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickAndPlace_Result_
{
  using Type = PickAndPlace_Result_<ContainerAllocator>;

  explicit PickAndPlace_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : final_object_pose(_init),
    final_joint_state(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_message = "";
      this->error_code = 0l;
      this->failure_phase = "";
      this->grasp_successful = false;
      this->place_successful = false;
      this->grasp_force_achieved = 0.0f;
      this->total_execution_time = 0.0f;
    }
  }

  explicit PickAndPlace_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_message(_alloc),
    failure_phase(_alloc),
    final_object_pose(_alloc, _init),
    final_joint_state(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_message = "";
      this->error_code = 0l;
      this->failure_phase = "";
      this->grasp_successful = false;
      this->place_successful = false;
      this->grasp_force_achieved = 0.0f;
      this->total_execution_time = 0.0f;
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
  using _failure_phase_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _failure_phase_type failure_phase;
  using _grasp_successful_type =
    bool;
  _grasp_successful_type grasp_successful;
  using _place_successful_type =
    bool;
  _place_successful_type place_successful;
  using _grasp_force_achieved_type =
    float;
  _grasp_force_achieved_type grasp_force_achieved;
  using _final_object_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _final_object_pose_type final_object_pose;
  using _total_execution_time_type =
    float;
  _total_execution_time_type total_execution_time;
  using _final_joint_state_type =
    sensor_msgs::msg::JointState_<ContainerAllocator>;
  _final_joint_state_type final_joint_state;

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
  Type & set__failure_phase(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->failure_phase = _arg;
    return *this;
  }
  Type & set__grasp_successful(
    const bool & _arg)
  {
    this->grasp_successful = _arg;
    return *this;
  }
  Type & set__place_successful(
    const bool & _arg)
  {
    this->place_successful = _arg;
    return *this;
  }
  Type & set__grasp_force_achieved(
    const float & _arg)
  {
    this->grasp_force_achieved = _arg;
    return *this;
  }
  Type & set__final_object_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->final_object_pose = _arg;
    return *this;
  }
  Type & set__total_execution_time(
    const float & _arg)
  {
    this->total_execution_time = _arg;
    return *this;
  }
  Type & set__final_joint_state(
    const sensor_msgs::msg::JointState_<ContainerAllocator> & _arg)
  {
    this->final_joint_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::PickAndPlace_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::PickAndPlace_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__PickAndPlace_Result
    std::shared_ptr<arm_control::action::PickAndPlace_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__PickAndPlace_Result
    std::shared_ptr<arm_control::action::PickAndPlace_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickAndPlace_Result_ & other) const
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
    if (this->failure_phase != other.failure_phase) {
      return false;
    }
    if (this->grasp_successful != other.grasp_successful) {
      return false;
    }
    if (this->place_successful != other.place_successful) {
      return false;
    }
    if (this->grasp_force_achieved != other.grasp_force_achieved) {
      return false;
    }
    if (this->final_object_pose != other.final_object_pose) {
      return false;
    }
    if (this->total_execution_time != other.total_execution_time) {
      return false;
    }
    if (this->final_joint_state != other.final_joint_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickAndPlace_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickAndPlace_Result_

// alias to use template instance with default allocator
using PickAndPlace_Result =
  arm_control::action::PickAndPlace_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__PickAndPlace_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__PickAndPlace_Feedback __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickAndPlace_Feedback_
{
  using Type = PickAndPlace_Feedback_<ContainerAllocator>;

  explicit PickAndPlace_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_phase = "";
      this->phase_progress = 0.0f;
      this->overall_progress = 0.0f;
      this->current_grasp_force = 0.0f;
      this->object_detected = false;
      this->grasp_status = "";
      this->status_message = "";
      this->estimated_time_remaining = 0.0f;
    }
  }

  explicit PickAndPlace_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_phase(_alloc),
    current_pose(_alloc, _init),
    grasp_status(_alloc),
    status_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_phase = "";
      this->phase_progress = 0.0f;
      this->overall_progress = 0.0f;
      this->current_grasp_force = 0.0f;
      this->object_detected = false;
      this->grasp_status = "";
      this->status_message = "";
      this->estimated_time_remaining = 0.0f;
    }
  }

  // field types and members
  using _current_phase_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_phase_type current_phase;
  using _phase_progress_type =
    float;
  _phase_progress_type phase_progress;
  using _overall_progress_type =
    float;
  _overall_progress_type overall_progress;
  using _current_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _current_pose_type current_pose;
  using _current_grasp_force_type =
    float;
  _current_grasp_force_type current_grasp_force;
  using _object_detected_type =
    bool;
  _object_detected_type object_detected;
  using _grasp_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _grasp_status_type grasp_status;
  using _status_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_message_type status_message;
  using _estimated_time_remaining_type =
    float;
  _estimated_time_remaining_type estimated_time_remaining;

  // setters for named parameter idiom
  Type & set__current_phase(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_phase = _arg;
    return *this;
  }
  Type & set__phase_progress(
    const float & _arg)
  {
    this->phase_progress = _arg;
    return *this;
  }
  Type & set__overall_progress(
    const float & _arg)
  {
    this->overall_progress = _arg;
    return *this;
  }
  Type & set__current_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->current_pose = _arg;
    return *this;
  }
  Type & set__current_grasp_force(
    const float & _arg)
  {
    this->current_grasp_force = _arg;
    return *this;
  }
  Type & set__object_detected(
    const bool & _arg)
  {
    this->object_detected = _arg;
    return *this;
  }
  Type & set__grasp_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->grasp_status = _arg;
    return *this;
  }
  Type & set__status_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status_message = _arg;
    return *this;
  }
  Type & set__estimated_time_remaining(
    const float & _arg)
  {
    this->estimated_time_remaining = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::PickAndPlace_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::PickAndPlace_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__PickAndPlace_Feedback
    std::shared_ptr<arm_control::action::PickAndPlace_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__PickAndPlace_Feedback
    std::shared_ptr<arm_control::action::PickAndPlace_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickAndPlace_Feedback_ & other) const
  {
    if (this->current_phase != other.current_phase) {
      return false;
    }
    if (this->phase_progress != other.phase_progress) {
      return false;
    }
    if (this->overall_progress != other.overall_progress) {
      return false;
    }
    if (this->current_pose != other.current_pose) {
      return false;
    }
    if (this->current_grasp_force != other.current_grasp_force) {
      return false;
    }
    if (this->object_detected != other.object_detected) {
      return false;
    }
    if (this->grasp_status != other.grasp_status) {
      return false;
    }
    if (this->status_message != other.status_message) {
      return false;
    }
    if (this->estimated_time_remaining != other.estimated_time_remaining) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickAndPlace_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickAndPlace_Feedback_

// alias to use template instance with default allocator
using PickAndPlace_Feedback =
  arm_control::action::PickAndPlace_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "arm_control/action/detail/pick_and_place__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__PickAndPlace_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__PickAndPlace_SendGoal_Request __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickAndPlace_SendGoal_Request_
{
  using Type = PickAndPlace_SendGoal_Request_<ContainerAllocator>;

  explicit PickAndPlace_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit PickAndPlace_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    arm_control::action::PickAndPlace_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const arm_control::action::PickAndPlace_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::PickAndPlace_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::PickAndPlace_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__PickAndPlace_SendGoal_Request
    std::shared_ptr<arm_control::action::PickAndPlace_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__PickAndPlace_SendGoal_Request
    std::shared_ptr<arm_control::action::PickAndPlace_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickAndPlace_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickAndPlace_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickAndPlace_SendGoal_Request_

// alias to use template instance with default allocator
using PickAndPlace_SendGoal_Request =
  arm_control::action::PickAndPlace_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__PickAndPlace_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__PickAndPlace_SendGoal_Response __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickAndPlace_SendGoal_Response_
{
  using Type = PickAndPlace_SendGoal_Response_<ContainerAllocator>;

  explicit PickAndPlace_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit PickAndPlace_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    arm_control::action::PickAndPlace_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::PickAndPlace_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__PickAndPlace_SendGoal_Response
    std::shared_ptr<arm_control::action::PickAndPlace_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__PickAndPlace_SendGoal_Response
    std::shared_ptr<arm_control::action::PickAndPlace_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickAndPlace_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickAndPlace_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickAndPlace_SendGoal_Response_

// alias to use template instance with default allocator
using PickAndPlace_SendGoal_Response =
  arm_control::action::PickAndPlace_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control

namespace arm_control
{

namespace action
{

struct PickAndPlace_SendGoal
{
  using Request = arm_control::action::PickAndPlace_SendGoal_Request;
  using Response = arm_control::action::PickAndPlace_SendGoal_Response;
};

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__PickAndPlace_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__PickAndPlace_GetResult_Request __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickAndPlace_GetResult_Request_
{
  using Type = PickAndPlace_GetResult_Request_<ContainerAllocator>;

  explicit PickAndPlace_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit PickAndPlace_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    arm_control::action::PickAndPlace_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::PickAndPlace_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__PickAndPlace_GetResult_Request
    std::shared_ptr<arm_control::action::PickAndPlace_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__PickAndPlace_GetResult_Request
    std::shared_ptr<arm_control::action::PickAndPlace_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickAndPlace_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickAndPlace_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickAndPlace_GetResult_Request_

// alias to use template instance with default allocator
using PickAndPlace_GetResult_Request =
  arm_control::action::PickAndPlace_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'result'
// already included above
// #include "arm_control/action/detail/pick_and_place__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__PickAndPlace_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__PickAndPlace_GetResult_Response __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickAndPlace_GetResult_Response_
{
  using Type = PickAndPlace_GetResult_Response_<ContainerAllocator>;

  explicit PickAndPlace_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit PickAndPlace_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    arm_control::action::PickAndPlace_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const arm_control::action::PickAndPlace_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::PickAndPlace_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::PickAndPlace_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__PickAndPlace_GetResult_Response
    std::shared_ptr<arm_control::action::PickAndPlace_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__PickAndPlace_GetResult_Response
    std::shared_ptr<arm_control::action::PickAndPlace_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickAndPlace_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickAndPlace_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickAndPlace_GetResult_Response_

// alias to use template instance with default allocator
using PickAndPlace_GetResult_Response =
  arm_control::action::PickAndPlace_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control

namespace arm_control
{

namespace action
{

struct PickAndPlace_GetResult
{
  using Request = arm_control::action::PickAndPlace_GetResult_Request;
  using Response = arm_control::action::PickAndPlace_GetResult_Response;
};

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "arm_control/action/detail/pick_and_place__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__PickAndPlace_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__PickAndPlace_FeedbackMessage __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickAndPlace_FeedbackMessage_
{
  using Type = PickAndPlace_FeedbackMessage_<ContainerAllocator>;

  explicit PickAndPlace_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit PickAndPlace_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    arm_control::action::PickAndPlace_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const arm_control::action::PickAndPlace_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::PickAndPlace_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::PickAndPlace_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::PickAndPlace_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::PickAndPlace_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::PickAndPlace_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::PickAndPlace_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__PickAndPlace_FeedbackMessage
    std::shared_ptr<arm_control::action::PickAndPlace_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__PickAndPlace_FeedbackMessage
    std::shared_ptr<arm_control::action::PickAndPlace_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickAndPlace_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickAndPlace_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickAndPlace_FeedbackMessage_

// alias to use template instance with default allocator
using PickAndPlace_FeedbackMessage =
  arm_control::action::PickAndPlace_FeedbackMessage_<std::allocator<void>>;

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

struct PickAndPlace
{
  /// The goal message defined in the action definition.
  using Goal = arm_control::action::PickAndPlace_Goal;
  /// The result message defined in the action definition.
  using Result = arm_control::action::PickAndPlace_Result;
  /// The feedback message defined in the action definition.
  using Feedback = arm_control::action::PickAndPlace_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = arm_control::action::PickAndPlace_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = arm_control::action::PickAndPlace_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = arm_control::action::PickAndPlace_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct PickAndPlace PickAndPlace;

}  // namespace action

}  // namespace arm_control

#endif  // ARM_CONTROL__ACTION__DETAIL__PICK_AND_PLACE__STRUCT_HPP_
