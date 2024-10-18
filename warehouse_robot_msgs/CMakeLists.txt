cmake_minimum_required(VERSION 3.8)
project(warehouse_robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Now place rosidl_generate_interfaces after all dependencies
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotData.msg"
  DEPENDENCIES nav_msgs geometry_msgs std_msgs
)

ament_package()

