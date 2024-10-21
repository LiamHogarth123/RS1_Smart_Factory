# Smart_Factory_System

need to installls
sudo apt install ros-humble-apriltag


To do list
- Set up AR Tag recognision
- clean up path planner generation OR Set up 2d NAV GOAL
- Otimpise turtlebot driving with mid point
- Set up collision avoidance
- set up rostopic status publishing
- re organise environment for drop off areas
- implement e-stop

Advance stuff
- set up multi robot functionality
- implement path planning collision prediction aviodance
- UR3 !!!!!!!!!!!!!!!!!


## File directory and code structure
- **global_controller_single_robot** loads the archived "path_planner_creation" map to generate paths
- **path_planner_creation** creates a graph to store the enviroment location
- **turtlebot_controller** controls a single robot to follow a given path
- **warehouse_robot_msgs** comunicates status from single robot to parent controller
- **World Files** gazebo files of the factory