# Smart_Factory_System

need to installls

sudo apt install ros-haumble-apriltag
install ros

to set up you need to colcon buid warehouse msgs first in workspace before building other packages. 
You need to change a couple of file paths in the constructors of globabl_controller Prm2.cpp and taskallocation.cpp

Commands to run

turtlebot_driver
- ros2 launch turtlebot_controller turtlebot_controller_launch.py 

global controller
- ros2 run global_controller_single_robot global_controller_single_robot 

Gazebo sim
- ros2 launch turtlebot3_gazebo SmartFactory.launch.py

Nav
- ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/liam/git/RS1_Smart_Factory/global_controller_single_robot/map/gazebo_sf_map.yaml


Current state
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
