// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arm_control:action/ToolChange.idl
// generated code does not contain a copyright notice

#ifndef ARM_CONTROL__ACTION__DETAIL__TOOL_CHANGE__STRUCT_HPP_
#define ARM_CONTROL__ACTION__DETAIL__TOOL_CHANGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'tool_dock_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__ToolChange_Goal __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__ToolChange_Goal __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ToolChange_Goal_
{
  using Type = ToolChange_Goal_<ContainerAllocator>;

  explicit ToolChange_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : tool_dock_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->auto_detect_current = false;
      this->approach_distance = 0.1f;
      this->coupling_timeout = 10.0f;
      this->verify_tool_change = true;
      this->update_kinematics = true;
      this->velocity_scaling = 0.05f;
      this->change_strategy = "auto";
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->new_tool_name = "";
      this->current_tool_name = "";
      this->auto_detect_current = false;
      this->approach_distance = 0.0f;
      this->coupling_timeout = 0.0f;
      this->verify_tool_change = false;
      this->update_kinematics = false;
      this->velocity_scaling = 0.0f;
      this->change_strategy = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->new_tool_name = "";
      this->current_tool_name = "";
    }
  }

  explicit ToolChange_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : new_tool_name(_alloc),
    current_tool_name(_alloc),
    tool_dock_pose(_alloc, _init),
    change_strategy(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->auto_detect_current = false;
      this->approach_distance = 0.1f;
      this->coupling_timeout = 10.0f;
      this->verify_tool_change = true;
      this->update_kinematics = true;
      this->velocity_scaling = 0.05f;
      this->change_strategy = "auto";
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->new_tool_name = "";
      this->current_tool_name = "";
      this->auto_detect_current = false;
      this->approach_distance = 0.0f;
      this->coupling_timeout = 0.0f;
      this->verify_tool_change = false;
      this->update_kinematics = false;
      this->velocity_scaling = 0.0f;
      this->change_strategy = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->new_tool_name = "";
      this->current_tool_name = "";
    }
  }

  // field types and members
  using _new_tool_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _new_tool_name_type new_tool_name;
  using _current_tool_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_tool_name_type current_tool_name;
  using _auto_detect_current_type =
    bool;
  _auto_detect_current_type auto_detect_current;
  using _tool_dock_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _tool_dock_pose_type tool_dock_pose;
  using _approach_distance_type =
    float;
  _approach_distance_type approach_distance;
  using _coupling_timeout_type =
    float;
  _coupling_timeout_type coupling_timeout;
  using _verify_tool_change_type =
    bool;
  _verify_tool_change_type verify_tool_change;
  using _update_kinematics_type =
    bool;
  _update_kinematics_type update_kinematics;
  using _velocity_scaling_type =
    float;
  _velocity_scaling_type velocity_scaling;
  using _change_strategy_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _change_strategy_type change_strategy;

  // setters for named parameter idiom
  Type & set__new_tool_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->new_tool_name = _arg;
    return *this;
  }
  Type & set__current_tool_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_tool_name = _arg;
    return *this;
  }
  Type & set__auto_detect_current(
    const bool & _arg)
  {
    this->auto_detect_current = _arg;
    return *this;
  }
  Type & set__tool_dock_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->tool_dock_pose = _arg;
    return *this;
  }
  Type & set__approach_distance(
    const float & _arg)
  {
    this->approach_distance = _arg;
    return *this;
  }
  Type & set__coupling_timeout(
    const float & _arg)
  {
    this->coupling_timeout = _arg;
    return *this;
  }
  Type & set__verify_tool_change(
    const bool & _arg)
  {
    this->verify_tool_change = _arg;
    return *this;
  }
  Type & set__update_kinematics(
    const bool & _arg)
  {
    this->update_kinematics = _arg;
    return *this;
  }
  Type & set__velocity_scaling(
    const float & _arg)
  {
    this->velocity_scaling = _arg;
    return *this;
  }
  Type & set__change_strategy(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->change_strategy = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::ToolChange_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::ToolChange_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__ToolChange_Goal
    std::shared_ptr<arm_control::action::ToolChange_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__ToolChange_Goal
    std::shared_ptr<arm_control::action::ToolChange_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToolChange_Goal_ & other) const
  {
    if (this->new_tool_name != other.new_tool_name) {
      return false;
    }
    if (this->current_tool_name != other.current_tool_name) {
      return false;
    }
    if (this->auto_detect_current != other.auto_detect_current) {
      return false;
    }
    if (this->tool_dock_pose != other.tool_dock_pose) {
      return false;
    }
    if (this->approach_distance != other.approach_distance) {
      return false;
    }
    if (this->coupling_timeout != other.coupling_timeout) {
      return false;
    }
    if (this->verify_tool_change != other.verify_tool_change) {
      return false;
    }
    if (this->update_kinematics != other.update_kinematics) {
      return false;
    }
    if (this->velocity_scaling != other.velocity_scaling) {
      return false;
    }
    if (this->change_strategy != other.change_strategy) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToolChange_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToolChange_Goal_

// alias to use template instance with default allocator
using ToolChange_Goal =
  arm_control::action::ToolChange_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'final_tool_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__ToolChange_Result __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__ToolChange_Result __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ToolChange_Result_
{
  using Type = ToolChange_Result_<ContainerAllocator>;

  explicit ToolChange_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : final_tool_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_message = "";
      this->error_code = 0l;
      this->old_tool_name = "";
      this->new_tool_name = "";
      this->coupling_verified = false;
      this->kinematics_updated = false;
      this->total_execution_time = 0.0f;
      this->tool_configuration = "";
    }
  }

  explicit ToolChange_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_message(_alloc),
    old_tool_name(_alloc),
    new_tool_name(_alloc),
    final_tool_pose(_alloc, _init),
    tool_configuration(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_message = "";
      this->error_code = 0l;
      this->old_tool_name = "";
      this->new_tool_name = "";
      this->coupling_verified = false;
      this->kinematics_updated = false;
      this->total_execution_time = 0.0f;
      this->tool_configuration = "";
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
  using _old_tool_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _old_tool_name_type old_tool_name;
  using _new_tool_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _new_tool_name_type new_tool_name;
  using _coupling_verified_type =
    bool;
  _coupling_verified_type coupling_verified;
  using _kinematics_updated_type =
    bool;
  _kinematics_updated_type kinematics_updated;
  using _final_tool_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _final_tool_pose_type final_tool_pose;
  using _total_execution_time_type =
    float;
  _total_execution_time_type total_execution_time;
  using _tool_configuration_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _tool_configuration_type tool_configuration;

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
  Type & set__old_tool_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->old_tool_name = _arg;
    return *this;
  }
  Type & set__new_tool_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->new_tool_name = _arg;
    return *this;
  }
  Type & set__coupling_verified(
    const bool & _arg)
  {
    this->coupling_verified = _arg;
    return *this;
  }
  Type & set__kinematics_updated(
    const bool & _arg)
  {
    this->kinematics_updated = _arg;
    return *this;
  }
  Type & set__final_tool_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->final_tool_pose = _arg;
    return *this;
  }
  Type & set__total_execution_time(
    const float & _arg)
  {
    this->total_execution_time = _arg;
    return *this;
  }
  Type & set__tool_configuration(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->tool_configuration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::ToolChange_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::ToolChange_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__ToolChange_Result
    std::shared_ptr<arm_control::action::ToolChange_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__ToolChange_Result
    std::shared_ptr<arm_control::action::ToolChange_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToolChange_Result_ & other) const
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
    if (this->old_tool_name != other.old_tool_name) {
      return false;
    }
    if (this->new_tool_name != other.new_tool_name) {
      return false;
    }
    if (this->coupling_verified != other.coupling_verified) {
      return false;
    }
    if (this->kinematics_updated != other.kinematics_updated) {
      return false;
    }
    if (this->final_tool_pose != other.final_tool_pose) {
      return false;
    }
    if (this->total_execution_time != other.total_execution_time) {
      return false;
    }
    if (this->tool_configuration != other.tool_configuration) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToolChange_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToolChange_Result_

// alias to use template instance with default allocator
using ToolChange_Result =
  arm_control::action::ToolChange_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


#ifndef _WIN32
# define DEPRECATED__arm_control__action__ToolChange_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__ToolChange_Feedback __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ToolChange_Feedback_
{
  using Type = ToolChange_Feedback_<ContainerAllocator>;

  explicit ToolChange_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_phase = "";
      this->phase_progress = 0.0f;
      this->overall_progress = 0.0f;
      this->tool_detected = false;
      this->coupling_engaged = false;
      this->coupling_status = "";
      this->kinematics_status = "";
      this->status_message = "";
      this->estimated_time_remaining = 0.0f;
    }
  }

  explicit ToolChange_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_phase(_alloc),
    coupling_status(_alloc),
    kinematics_status(_alloc),
    status_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_phase = "";
      this->phase_progress = 0.0f;
      this->overall_progress = 0.0f;
      this->tool_detected = false;
      this->coupling_engaged = false;
      this->coupling_status = "";
      this->kinematics_status = "";
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
  using _tool_detected_type =
    bool;
  _tool_detected_type tool_detected;
  using _coupling_engaged_type =
    bool;
  _coupling_engaged_type coupling_engaged;
  using _coupling_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _coupling_status_type coupling_status;
  using _kinematics_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _kinematics_status_type kinematics_status;
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
  Type & set__tool_detected(
    const bool & _arg)
  {
    this->tool_detected = _arg;
    return *this;
  }
  Type & set__coupling_engaged(
    const bool & _arg)
  {
    this->coupling_engaged = _arg;
    return *this;
  }
  Type & set__coupling_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->coupling_status = _arg;
    return *this;
  }
  Type & set__kinematics_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->kinematics_status = _arg;
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
    arm_control::action::ToolChange_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::ToolChange_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__ToolChange_Feedback
    std::shared_ptr<arm_control::action::ToolChange_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__ToolChange_Feedback
    std::shared_ptr<arm_control::action::ToolChange_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToolChange_Feedback_ & other) const
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
    if (this->tool_detected != other.tool_detected) {
      return false;
    }
    if (this->coupling_engaged != other.coupling_engaged) {
      return false;
    }
    if (this->coupling_status != other.coupling_status) {
      return false;
    }
    if (this->kinematics_status != other.kinematics_status) {
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
  bool operator!=(const ToolChange_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToolChange_Feedback_

// alias to use template instance with default allocator
using ToolChange_Feedback =
  arm_control::action::ToolChange_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "arm_control/action/detail/tool_change__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__ToolChange_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__ToolChange_SendGoal_Request __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ToolChange_SendGoal_Request_
{
  using Type = ToolChange_SendGoal_Request_<ContainerAllocator>;

  explicit ToolChange_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit ToolChange_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    arm_control::action::ToolChange_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const arm_control::action::ToolChange_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::ToolChange_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::ToolChange_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__ToolChange_SendGoal_Request
    std::shared_ptr<arm_control::action::ToolChange_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__ToolChange_SendGoal_Request
    std::shared_ptr<arm_control::action::ToolChange_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToolChange_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToolChange_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToolChange_SendGoal_Request_

// alias to use template instance with default allocator
using ToolChange_SendGoal_Request =
  arm_control::action::ToolChange_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__ToolChange_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__ToolChange_SendGoal_Response __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ToolChange_SendGoal_Response_
{
  using Type = ToolChange_SendGoal_Response_<ContainerAllocator>;

  explicit ToolChange_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit ToolChange_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    arm_control::action::ToolChange_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::ToolChange_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__ToolChange_SendGoal_Response
    std::shared_ptr<arm_control::action::ToolChange_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__ToolChange_SendGoal_Response
    std::shared_ptr<arm_control::action::ToolChange_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToolChange_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToolChange_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToolChange_SendGoal_Response_

// alias to use template instance with default allocator
using ToolChange_SendGoal_Response =
  arm_control::action::ToolChange_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control

namespace arm_control
{

namespace action
{

struct ToolChange_SendGoal
{
  using Request = arm_control::action::ToolChange_SendGoal_Request;
  using Response = arm_control::action::ToolChange_SendGoal_Response;
};

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__ToolChange_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__ToolChange_GetResult_Request __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ToolChange_GetResult_Request_
{
  using Type = ToolChange_GetResult_Request_<ContainerAllocator>;

  explicit ToolChange_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit ToolChange_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    arm_control::action::ToolChange_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::ToolChange_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__ToolChange_GetResult_Request
    std::shared_ptr<arm_control::action::ToolChange_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__ToolChange_GetResult_Request
    std::shared_ptr<arm_control::action::ToolChange_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToolChange_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToolChange_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToolChange_GetResult_Request_

// alias to use template instance with default allocator
using ToolChange_GetResult_Request =
  arm_control::action::ToolChange_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'result'
// already included above
// #include "arm_control/action/detail/tool_change__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__ToolChange_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__ToolChange_GetResult_Response __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ToolChange_GetResult_Response_
{
  using Type = ToolChange_GetResult_Response_<ContainerAllocator>;

  explicit ToolChange_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit ToolChange_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    arm_control::action::ToolChange_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const arm_control::action::ToolChange_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::ToolChange_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::ToolChange_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__ToolChange_GetResult_Response
    std::shared_ptr<arm_control::action::ToolChange_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__ToolChange_GetResult_Response
    std::shared_ptr<arm_control::action::ToolChange_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToolChange_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToolChange_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToolChange_GetResult_Response_

// alias to use template instance with default allocator
using ToolChange_GetResult_Response =
  arm_control::action::ToolChange_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace arm_control

namespace arm_control
{

namespace action
{

struct ToolChange_GetResult
{
  using Request = arm_control::action::ToolChange_GetResult_Request;
  using Response = arm_control::action::ToolChange_GetResult_Response;
};

}  // namespace action

}  // namespace arm_control


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "arm_control/action/detail/tool_change__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arm_control__action__ToolChange_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__arm_control__action__ToolChange_FeedbackMessage __declspec(deprecated)
#endif

namespace arm_control
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ToolChange_FeedbackMessage_
{
  using Type = ToolChange_FeedbackMessage_<ContainerAllocator>;

  explicit ToolChange_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit ToolChange_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    arm_control::action::ToolChange_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const arm_control::action::ToolChange_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arm_control::action::ToolChange_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const arm_control::action::ToolChange_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arm_control::action::ToolChange_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arm_control::action::ToolChange_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arm_control::action::ToolChange_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arm_control::action::ToolChange_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arm_control__action__ToolChange_FeedbackMessage
    std::shared_ptr<arm_control::action::ToolChange_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arm_control__action__ToolChange_FeedbackMessage
    std::shared_ptr<arm_control::action::ToolChange_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToolChange_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToolChange_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToolChange_FeedbackMessage_

// alias to use template instance with default allocator
using ToolChange_FeedbackMessage =
  arm_control::action::ToolChange_FeedbackMessage_<std::allocator<void>>;

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

struct ToolChange
{
  /// The goal message defined in the action definition.
  using Goal = arm_control::action::ToolChange_Goal;
  /// The result message defined in the action definition.
  using Result = arm_control::action::ToolChange_Result;
  /// The feedback message defined in the action definition.
  using Feedback = arm_control::action::ToolChange_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = arm_control::action::ToolChange_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = arm_control::action::ToolChange_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = arm_control::action::ToolChange_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct ToolChange ToolChange;

}  // namespace action

}  // namespace arm_control

#endif  // ARM_CONTROL__ACTION__DETAIL__TOOL_CHANGE__STRUCT_HPP_
