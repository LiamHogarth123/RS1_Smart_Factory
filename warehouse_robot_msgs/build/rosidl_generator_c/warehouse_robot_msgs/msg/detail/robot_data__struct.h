// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from warehouse_robot_msgs:msg/RobotData.idl
// generated code does not contain a copyright notice

#ifndef WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__STRUCT_H_
#define WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'odom'
#include "nav_msgs/msg/detail/odometry__struct.h"
// Member 'status'
#include "rosidl_runtime_c/string.h"
// Member 'ar_tag_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/RobotData in the package warehouse_robot_msgs.
/**
  * RobotData.msg
 */
typedef struct warehouse_robot_msgs__msg__RobotData
{
  /// Odometry data
  nav_msgs__msg__Odometry odom;
  /// Status string
  rosidl_runtime_c__String status;
  /// AR tag position (pose)
  geometry_msgs__msg__Pose ar_tag_pose;
} warehouse_robot_msgs__msg__RobotData;

// Struct for a sequence of warehouse_robot_msgs__msg__RobotData.
typedef struct warehouse_robot_msgs__msg__RobotData__Sequence
{
  warehouse_robot_msgs__msg__RobotData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} warehouse_robot_msgs__msg__RobotData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__STRUCT_H_
