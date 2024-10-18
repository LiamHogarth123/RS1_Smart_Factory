// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from warehouse_robot_msgs:msg/RobotData.idl
// generated code does not contain a copyright notice

#ifndef WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__STRUCT_HPP_
#define WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'odom'
#include "nav_msgs/msg/detail/odometry__struct.hpp"
// Member 'ar_tag_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__warehouse_robot_msgs__msg__RobotData __attribute__((deprecated))
#else
# define DEPRECATED__warehouse_robot_msgs__msg__RobotData __declspec(deprecated)
#endif

namespace warehouse_robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotData_
{
  using Type = RobotData_<ContainerAllocator>;

  explicit RobotData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : odom(_init),
    ar_tag_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
    }
  }

  explicit RobotData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : odom(_alloc, _init),
    status(_alloc),
    ar_tag_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
    }
  }

  // field types and members
  using _odom_type =
    nav_msgs::msg::Odometry_<ContainerAllocator>;
  _odom_type odom;
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;
  using _ar_tag_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _ar_tag_pose_type ar_tag_pose;

  // setters for named parameter idiom
  Type & set__odom(
    const nav_msgs::msg::Odometry_<ContainerAllocator> & _arg)
  {
    this->odom = _arg;
    return *this;
  }
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__ar_tag_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->ar_tag_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    warehouse_robot_msgs::msg::RobotData_<ContainerAllocator> *;
  using ConstRawPtr =
    const warehouse_robot_msgs::msg::RobotData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<warehouse_robot_msgs::msg::RobotData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<warehouse_robot_msgs::msg::RobotData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      warehouse_robot_msgs::msg::RobotData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<warehouse_robot_msgs::msg::RobotData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      warehouse_robot_msgs::msg::RobotData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<warehouse_robot_msgs::msg::RobotData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<warehouse_robot_msgs::msg::RobotData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<warehouse_robot_msgs::msg::RobotData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__warehouse_robot_msgs__msg__RobotData
    std::shared_ptr<warehouse_robot_msgs::msg::RobotData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__warehouse_robot_msgs__msg__RobotData
    std::shared_ptr<warehouse_robot_msgs::msg::RobotData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotData_ & other) const
  {
    if (this->odom != other.odom) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->ar_tag_pose != other.ar_tag_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotData_

// alias to use template instance with default allocator
using RobotData =
  warehouse_robot_msgs::msg::RobotData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace warehouse_robot_msgs

#endif  // WAREHOUSE_ROBOT_MSGS__MSG__DETAIL__ROBOT_DATA__STRUCT_HPP_
