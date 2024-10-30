// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from warehouse_robot_msgs:msg/RobotData.idl
// generated code does not contain a copyright notice

#ifndef WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__TRAITS_HPP_
#define WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "warehouse_robot_msgs/msg/detail/robot_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'odom'
#include "nav_msgs/msg/detail/odometry__traits.hpp"

namespace warehouse_robot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotData & msg,
  std::ostream & out)
{
  out << "{";
  // member: odom
  {
    out << "odom: ";
    to_flow_style_yaml(msg.odom, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: ar_tag_id
  {
    out << "ar_tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.ar_tag_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: odom
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "odom:\n";
    to_block_style_yaml(msg.odom, out, indentation + 2);
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: ar_tag_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ar_tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.ar_tag_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace warehouse_robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use warehouse_robot_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const warehouse_robot_msgs::msg::RobotData & msg,
  std::ostream & out, size_t indentation = 0)
{
  warehouse_robot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use warehouse_robot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const warehouse_robot_msgs::msg::RobotData & msg)
{
  return warehouse_robot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<warehouse_robot_msgs::msg::RobotData>()
{
  return "warehouse_robot_msgs::msg::RobotData";
}

template<>
inline const char * name<warehouse_robot_msgs::msg::RobotData>()
{
  return "warehouse_robot_msgs/msg/RobotData";
}

template<>
struct has_fixed_size<warehouse_robot_msgs::msg::RobotData>
  : std::integral_constant<bool, has_fixed_size<nav_msgs::msg::Odometry>::value> {};

template<>
struct has_bounded_size<warehouse_robot_msgs::msg::RobotData>
  : std::integral_constant<bool, has_bounded_size<nav_msgs::msg::Odometry>::value> {};

template<>
struct is_message<warehouse_robot_msgs::msg::RobotData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__TRAITS_HPP_
