# Multi-Robot Warehouse Automation System

This ROS 2 project provides a comprehensive solution for automating warehouse operations using TurtleBot robots. It includes global coordination, individual robot control, and visualization tools to efficiently manage tasks like package pickup and delivery.
Features
1. Global_Controller

    Task Allocation: Assigns tasks to multiple TurtleBots based on their current position and capabilities.
    Path Planning: Uses A* algorithm for generating optimal paths.
    Multi-Robot Coordination: Synchronizes operations across robots while avoiding conflicts.
    Emergency Stop: Implements an e-stop functionality for safety.


2. Turtlebot_Controller

    Path Execution: Processes trajectory data and dynamically navigates TurtleBots to assigned goals.
    Look-Ahead Point Calculation: Ensures smooth navigation and obstacle avoidance.
    AR Tag Detection: Rotates the robot to search for AR tags and validates packages.
    Visualization: Publishes markers and paths for real-time visualization in RViz.

Architecture

    Global Controller:
        Publishes trajectories for each TurtleBot.
        Manages task allocation and synchronization.

    Turtlebot_Controller:
        Executes trajectories.
        Uses PID control for precise movement.
        Publishes real-time robot status.

ROS Topics
Subscriptions

    /trajectory (nav_msgs/Path): Receives path commands for navigation.
    /odom (nav_msgs/Odometry): Robot odometry updates.
    /scan (sensor_msgs/LaserScan): LiDAR data for obstacle detection.
    /camera/rgb/image_raw and /camera/depth/image_raw (sensor_msgs/Image): Vision data for AR tag detection.

Publications

    /cmd_vel (geometry_msgs/Twist): Sends velocity commands to robots.
    /robot_data (warehouse_robot_msgs/RobotData): Publishes robot status, odometry, and AR tag info.
    /visualization_marker and /visualization_marker_array (visualization_msgs/Marker/MarkerArray): RViz visualization.

Getting Started
Prerequisites

    ROS 2 Humble or later.
    RViz for visualization.

Installation

    Clone the repository:

git clone <repository_url>

Build the workspace:

colcon build

Source the workspace:

    source install/setup.bash

Running the System

    Launch Global Controller: ros2 launch global_controller_single_robot global_controller_multi_launch.py



Launch RViz:

    ros2 launch global_controller_single_robot rviz_visualisation_launch.py 


Notes

    Namespace Isolation: Each TurtleBot operates in its own namespace for scalability.
    Visualization: Use RViz to monitor paths, trajectories, and look-ahead points.
    E-Stop Safety: Emergency stop can be triggered via the /e_stop topic.



# To do list

# global planner functionality
* Need to have predictive path planning consision aviodance. infrastructure is there functionality is not
* Test and improve lane functionality

* AR tag communication
* docking or better publishing when package has been delievered
  


# turtlebot functionality
* Ar tag detection
* On request odom publishing
* E-stop and resuming
* 


# Ur3 functionality
* we can spawn a ur3 and control it manually but are unable to using movit through code
* once Moveit works need to have it move repeativily on command to x,y,z position
* Need to setup Ur3_controller interactable with global planner
* Need to setup up proper launch files within this package

* could add a camera functionality to find package Ar tag and pick it up

# Ros launch files and packages
* Need to setup better clear launch files and packages







## File directory and code structure
- **global_controller_single_robot** loads the archived "path_planner_creation" map to generate paths
- **path_planner_creation** creates a graph to store the enviroment location
- **turtlebot_controller** controls a single robot to follow a given path
- **warehouse_robot_msgs** comunicates status from single robot to parent controller
- **World Files** gazebo files of the factory




if the single launch file doesn't work on your system run the following commands


turtlebot_driver
- ros2 launch turtlebot_controller turtlebot_controller_launch.py 

global controller
- ros2 run global_controller_single_robot global_controller_single_robot 

Gazebo sim
- ros2 launch turtlebot3_gazebo SmartFactory.launch.py

Nav
- ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/liam/git/RS1_Smart_Factory/global_controller_single_robot/map/gazebo_sf_map.yaml

