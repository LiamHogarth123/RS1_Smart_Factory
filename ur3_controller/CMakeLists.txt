cmake_minimum_required(VERSION 3.5)
project(ur3_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Include directories
include_directories(include)

# Declare the executable and link libraries
add_executable(ur3_control_node src/main.cpp src/controller.cpp)
ament_target_dependencies(ur3_control_node rclcpp moveit_ros_planning_interface)

install(TARGETS ur3_control_node DESTINATION lib/${PROJECT_NAME})


ament_package()