// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from warehouse_robot_msgs:msg/RobotData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "warehouse_robot_msgs/msg/detail/robot_data__rosidl_typesupport_introspection_c.h"
#include "warehouse_robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "warehouse_robot_msgs/msg/detail/robot_data__functions.h"
#include "warehouse_robot_msgs/msg/detail/robot_data__struct.h"


// Include directives for member types
// Member `odom`
#include "nav_msgs/msg/odometry.h"
// Member `odom`
#include "nav_msgs/msg/detail/odometry__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void warehouse_robot_msgs__msg__RobotData__rosidl_typesupport_introspection_c__RobotData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  warehouse_robot_msgs__msg__RobotData__init(message_memory);
}

void warehouse_robot_msgs__msg__RobotData__rosidl_typesupport_introspection_c__RobotData_fini_function(void * message_memory)
{
  warehouse_robot_msgs__msg__RobotData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember warehouse_robot_msgs__msg__RobotData__rosidl_typesupport_introspection_c__RobotData_message_member_array[3] = {
  {
    "odom",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(warehouse_robot_msgs__msg__RobotData, odom),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(warehouse_robot_msgs__msg__RobotData, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ar_tag_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(warehouse_robot_msgs__msg__RobotData, ar_tag_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers warehouse_robot_msgs__msg__RobotData__rosidl_typesupport_introspection_c__RobotData_message_members = {
  "warehouse_robot_msgs__msg",  // message namespace
  "RobotData",  // message name
  3,  // number of fields
  sizeof(warehouse_robot_msgs__msg__RobotData),
  warehouse_robot_msgs__msg__RobotData__rosidl_typesupport_introspection_c__RobotData_message_member_array,  // message members
  warehouse_robot_msgs__msg__RobotData__rosidl_typesupport_introspection_c__RobotData_init_function,  // function to initialize message memory (memory has to be allocated)
  warehouse_robot_msgs__msg__RobotData__rosidl_typesupport_introspection_c__RobotData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t warehouse_robot_msgs__msg__RobotData__rosidl_typesupport_introspection_c__RobotData_message_type_support_handle = {
  0,
  &warehouse_robot_msgs__msg__RobotData__rosidl_typesupport_introspection_c__RobotData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_warehouse_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, warehouse_robot_msgs, msg, RobotData)() {
  warehouse_robot_msgs__msg__RobotData__rosidl_typesupport_introspection_c__RobotData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav_msgs, msg, Odometry)();
  if (!warehouse_robot_msgs__msg__RobotData__rosidl_typesupport_introspection_c__RobotData_message_type_support_handle.typesupport_identifier) {
    warehouse_robot_msgs__msg__RobotData__rosidl_typesupport_introspection_c__RobotData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &warehouse_robot_msgs__msg__RobotData__rosidl_typesupport_introspection_c__RobotData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
