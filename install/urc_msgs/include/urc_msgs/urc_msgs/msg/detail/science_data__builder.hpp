// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from urc_msgs:msg/ScienceData.idl
// generated code does not contain a copyright notice

#ifndef URC_MSGS__MSG__DETAIL__SCIENCE_DATA__BUILDER_HPP_
#define URC_MSGS__MSG__DETAIL__SCIENCE_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "urc_msgs/msg/detail/science_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace urc_msgs
{

namespace msg
{

namespace builder
{

class Init_ScienceData_detected_compounds
{
public:
  explicit Init_ScienceData_detected_compounds(::urc_msgs::msg::ScienceData & msg)
  : msg_(msg)
  {}
  ::urc_msgs::msg::ScienceData detected_compounds(::urc_msgs::msg::ScienceData::_detected_compounds_type arg)
  {
    msg_.detected_compounds = std::move(arg);
    return std::move(msg_);
  }

private:
  ::urc_msgs::msg::ScienceData msg_;
};

class Init_ScienceData_moisture
{
public:
  explicit Init_ScienceData_moisture(::urc_msgs::msg::ScienceData & msg)
  : msg_(msg)
  {}
  Init_ScienceData_detected_compounds moisture(::urc_msgs::msg::ScienceData::_moisture_type arg)
  {
    msg_.moisture = std::move(arg);
    return Init_ScienceData_detected_compounds(msg_);
  }

private:
  ::urc_msgs::msg::ScienceData msg_;
};

class Init_ScienceData_temperature
{
public:
  explicit Init_ScienceData_temperature(::urc_msgs::msg::ScienceData & msg)
  : msg_(msg)
  {}
  Init_ScienceData_moisture temperature(::urc_msgs::msg::ScienceData::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_ScienceData_moisture(msg_);
  }

private:
  ::urc_msgs::msg::ScienceData msg_;
};

class Init_ScienceData_ph_level
{
public:
  explicit Init_ScienceData_ph_level(::urc_msgs::msg::ScienceData & msg)
  : msg_(msg)
  {}
  Init_ScienceData_temperature ph_level(::urc_msgs::msg::ScienceData::_ph_level_type arg)
  {
    msg_.ph_level = std::move(arg);
    return Init_ScienceData_temperature(msg_);
  }

private:
  ::urc_msgs::msg::ScienceData msg_;
};

class Init_ScienceData_sample_location
{
public:
  explicit Init_ScienceData_sample_location(::urc_msgs::msg::ScienceData & msg)
  : msg_(msg)
  {}
  Init_ScienceData_ph_level sample_location(::urc_msgs::msg::ScienceData::_sample_location_type arg)
  {
    msg_.sample_location = std::move(arg);
    return Init_ScienceData_ph_level(msg_);
  }

private:
  ::urc_msgs::msg::ScienceData msg_;
};

class Init_ScienceData_sample_id
{
public:
  explicit Init_ScienceData_sample_id(::urc_msgs::msg::ScienceData & msg)
  : msg_(msg)
  {}
  Init_ScienceData_sample_location sample_id(::urc_msgs::msg::ScienceData::_sample_id_type arg)
  {
    msg_.sample_id = std::move(arg);
    return Init_ScienceData_sample_location(msg_);
  }

private:
  ::urc_msgs::msg::ScienceData msg_;
};

class Init_ScienceData_header
{
public:
  Init_ScienceData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ScienceData_sample_id header(::urc_msgs::msg::ScienceData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ScienceData_sample_id(msg_);
  }

private:
  ::urc_msgs::msg::ScienceData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::urc_msgs::msg::ScienceData>()
{
  return urc_msgs::msg::builder::Init_ScienceData_header();
}

}  // namespace urc_msgs

#endif  // URC_MSGS__MSG__DETAIL__SCIENCE_DATA__BUILDER_HPP_
