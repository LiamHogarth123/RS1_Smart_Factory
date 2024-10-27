cmake_minimum_required(VERSION 3.8)
project(turtlebot_estop_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)

# List of source files
set(estop_SRCS
  src/main.cpp
  src/estop.cpp
)

# Define the executable target
add_executable(estop_ex ${estop_SRCS})

# Specify dependencies for the target
ament_target_dependencies(
  estop_ex  # The target name created by add_executable
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  OpenCV
  visualization_msgs
  nav_msgs
)

# Link against OpenCV libraries
target_link_libraries(estop_ex ${OpenCV_LIBS})

# Install the executable
install(TARGETS
  estop_ex
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
