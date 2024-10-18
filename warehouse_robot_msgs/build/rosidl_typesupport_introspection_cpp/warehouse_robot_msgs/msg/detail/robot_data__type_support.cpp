// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from warehouse_robot_msgs:msg/RobotData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "warehouse_robot_msgs/msg/detail/robot_data__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace warehouse_robot_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void RobotData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) warehouse_robot_msgs::msg::RobotData(_init);
}

void RobotData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<warehouse_robot_msgs::msg::RobotData *>(message_memory);
  typed_message->~RobotData();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RobotData_message_member_array[3] = {
  {
    "odom",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<nav_msgs::msg::Odometry>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(warehouse_robot_msgs::msg::RobotData, odom),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(warehouse_robot_msgs::msg::RobotData, status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "ar_tag_pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Pose>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(warehouse_robot_msgs::msg::RobotData, ar_tag_pose),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RobotData_message_members = {
  "warehouse_robot_msgs::msg",  // message namespace
  "RobotData",  // message name
  3,  // number of fields
  sizeof(warehouse_robot_msgs::msg::RobotData),
  RobotData_message_member_array,  // message members
  RobotData_init_function,  // function to initialize message memory (memory has to be allocated)
  RobotData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RobotData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RobotData_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace warehouse_robot_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<warehouse_robot_msgs::msg::RobotData>()
{
  return &::warehouse_robot_msgs::msg::rosidl_typesupport_introspection_cpp::RobotData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, warehouse_robot_msgs, msg, RobotData)() {
  return &::warehouse_robot_msgs::msg::rosidl_typesupport_introspection_cpp::RobotData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
