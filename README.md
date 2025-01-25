Multi-Robot Warehouse Automation System

This ROS 2 project provides a comprehensive solution for automating warehouse operations using TurtleBot robots. It includes global coordination, individual robot control, and visualization tools to efficiently manage tasks like package pickup and delivery.
Features
1. Global Controller

    Task Allocation: Assigns tasks to multiple TurtleBots based on their current position and capabilities.
    Path Planning: Uses A* algorithm for generating optimal paths.
    Multi-Robot Coordination: Synchronizes operations across robots while avoiding conflicts.
    Emergency Stop: Implements an e-stop functionality for safety.

2. TurtleBot Manager

    Trajectory Management: Receives trajectories and manages execution for individual robots.
    Robot Data Publishing: Shares odometry, speed, and AR tag information.
    Namespace Support: Operates in isolated namespaces for multi-robot environments.

3. Controller

    Path Execution: Processes trajectory data and dynamically navigates TurtleBots to assigned goals.
    Look-Ahead Point Calculation: Ensures smooth navigation and obstacle avoidance.
    AR Tag Detection: Rotates the robot to search for AR tags and validates packages.
    Visualization: Publishes markers and paths for real-time visualization in RViz.

Architecture

    Global Controller:
        Publishes trajectories for each TurtleBot.
        Manages task allocation and synchronization.
    TurtleBot Manager:
        Handles individual robot's state and odometry.
        Receives trajectory commands from the Global Controller.
    Controller:
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

    Launch Global Controller:

ros2 launch <package_name> global_controller.launch.py

Launch TurtleBot Nodes:

ros2 launch <package_name> turtlebot_manager.launch.py

Launch RViz:

    rviz2 -d <config_file>

Notes

    Namespace Isolation: Each TurtleBot operates in its own namespace for scalability.
    Visualization: Use RViz to monitor paths, trajectories, and look-ahead points.
    E-Stop Safety: Emergency stop can be triggered via the /e_stop topic.
Commands to run
ros2 launch global_controller_single_robot global_controller_multi_launch.py






> To do list




* Turtlebot drives to multiple goal. path planning could be optimised and it doesn't have collision aviodance alignment optimisation or Ar tag recon.
* multi turtlebot works with a modifed package that can't be pushed to git. For multi need updated task allo and path collision aviodance



UR3 jobs
- need the UR3 to be able to move to a x,y,z position to simulate picking up an object

Turtlebot jobs
- test and fix lanes
- Implement Ar tag detection
- setup status logging

Multi turtlebot jobs
- Find an optimised way to drive multi ansychously
- Implement multi robot path collision avoidance
- 

Top level system
- Set up ros action call to start system
- Gui setup for error loging, shutdown request, e-stop,
- 




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


Current state
