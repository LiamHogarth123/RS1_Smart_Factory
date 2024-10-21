# Smart_Factory_System

need to installls
sudo apt install ros-humble-apriltag


Current state
* Global planner has been tested in developing paths and publishing it to the robot. 
* Turtlebot_controller has not been tested and is stil being cleaned up.
* About a month ago the system worked together with very limited features so therefore with minial work they should function together.


Important To do list first
- Ensure Global planner is publishing trajectory correctly - Done <- printed correctly in terminal
- Ensure turtlebot is driving correctly -
- Ensure turtlebot is publishing its status and info correctly - 
- Ensure Launch file launches global Planner and turtlebot_controller - semi done with nav,gazebo,global but without turtle_controllr

- Otimpise turtlebot driving with mid point
- Set up collision avoidance
- set up rostopic status publishing
- re organise environment for drop off sareas


Advance stuff
- set up multi robot functionality
- cleanup path planner with lanes saved
- implement path planning collision prediction aviodance
- Set up AR Tag recognision
- implement e-stop
- UR3 !!!!!!!!!!!!!!!!!


## File directory and code structure
- **global_controller_single_robot** loads the archived "path_planner_creation" map to generate paths
- **path_planner_creation** creates a graph to store the enviroment location
- **turtlebot_controller** controls a single robot to follow a given path
- **warehouse_robot_msgs** comunicates status from single robot to parent controller
- **World Files** gazebo files of the factory
