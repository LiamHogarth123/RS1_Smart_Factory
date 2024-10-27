cmake_minimum_required(VERSION 3.5)
project(turtlebot_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(marker_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(warehouse_robot_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


# Add executables and link libraries
add_executable(turtlebot_controller
  src/controller.cpp
  src/main.cpp
  # src/turtlebot.cpp
  src/turtlebot_sensorprocessing.cpp
  src/turtlebot_control_cal.cpp
)

# Link the executable to the necessary libraries
ament_target_dependencies(
  turtlebot_controller  # Correct the target name here
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  # marker_msgs
  tf2_geometry_msgs
  warehouse_robot_msgs
  visualization_msgs
)

# Install the executable
install(TARGETS
  turtlebot_controller
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
