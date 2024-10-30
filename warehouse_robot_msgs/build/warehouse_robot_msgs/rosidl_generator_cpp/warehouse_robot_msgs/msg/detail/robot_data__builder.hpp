// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from warehouse_robot_msgs:msg/RobotData.idl
// generated code does not contain a copyright notice

#ifndef WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__BUILDER_HPP_
#define WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "warehouse_robot_msgs/msg/detail/robot_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace warehouse_robot_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotData_ar_tag_id
{
public:
  explicit Init_RobotData_ar_tag_id(::warehouse_robot_msgs::msg::RobotData & msg)
  : msg_(msg)
  {}
  ::warehouse_robot_msgs::msg::RobotData ar_tag_id(::warehouse_robot_msgs::msg::RobotData::_ar_tag_id_type arg)
  {
    msg_.ar_tag_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::warehouse_robot_msgs::msg::RobotData msg_;
};

class Init_RobotData_status
{
public:
  explicit Init_RobotData_status(::warehouse_robot_msgs::msg::RobotData & msg)
  : msg_(msg)
  {}
  Init_RobotData_ar_tag_id status(::warehouse_robot_msgs::msg::RobotData::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_RobotData_ar_tag_id(msg_);
  }

private:
  ::warehouse_robot_msgs::msg::RobotData msg_;
};

class Init_RobotData_odom
{
public:
  Init_RobotData_odom()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotData_status odom(::warehouse_robot_msgs::msg::RobotData::_odom_type arg)
  {
    msg_.odom = std::move(arg);
    return Init_RobotData_status(msg_);
  }

private:
  ::warehouse_robot_msgs::msg::RobotData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::warehouse_robot_msgs::msg::RobotData>()
{
  return warehouse_robot_msgs::msg::builder::Init_RobotData_odom();
}

}  // namespace warehouse_robot_msgs

#endif  // WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__BUILDER_HPP_
