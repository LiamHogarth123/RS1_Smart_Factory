# Smart_Factory_System

need to installls
sudo apt install ros-haumble-apriltag

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




IMPORT TO FIX
- see if lanes will work else increase distance tolerance
- launch
- multi robot path aviodance

Jobs for teammates
- Setup ros action call so we can publish a message to start job
- setup package inspection at each package site (e.g turn look check for AR tag)
- Set up AR Tag recognision
- implement e-stop - need fixing - blocking method may have failed
- set up rostopic status publishing
- setup warehouse package logging
- cleanup path planner with lanes saved
- ur3 controller finish


Key functionality
- Ensure Global planner is publishing trajectory correctly - Done <- printed correctly in terminal
- Ensure turtlebot is driving correctly <- Done
- Ensure turtlebot is publishing its status and info correctly <- DONE 
- Ensure Launch file launches global Planner and turtlebot_controller - semi done with nav,gazebo,global but without turtle_controllr



## File directory and code structure
- **global_controller_single_robot** loads the archived "path_planner_creation" map to generate paths
- **path_planner_creation** creates a graph to store the enviroment location
- **turtlebot_controller** controls a single robot to follow a given path
- **warehouse_robot_msgs** comunicates status from single robot to parent controller
- **World Files** gazebo files of the factory
