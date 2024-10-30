from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to launch files in your workspace
    turtlebot3_gazebo_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    turtlebot3_navigation2_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch')
    turtlebot_controller_launch_file_dir = os.path.join(get_package_share_directory('turtlebot_controller'), 'launch')
    map_file_path = os.path.join(get_package_share_directory('global_controller_single_robot'), 'map', 'map.yaml')
    smart_factory_launch_file = os.path.join(turtlebot3_gazebo_launch_file_dir, 'SmartFactory.launch.py')

    gazebo_multi_nav2_world_launch_file_dir = os.path.join(get_package_share_directory('smart_factory_multi_robot'), 'launch')
    gazebo_multi_nav2_world_launch_file = os.path.join(gazebo_multi_nav2_world_launch_file_dir, 'gazebo_multi_nav2_world.launch.py')

    # Declare the integer launch argument
    number_of_robots_arg = DeclareLaunchArgument(
        'number_of_robots',
        default_value='10',  # Default value for the parameter
        description='An integer parameter for the global planner'
    )

    # Fetch the LaunchConfiguration for the number_of_robots
    number_of_robots = LaunchConfiguration('number_of_robots')

    return LaunchDescription([
        # Declare the argument
        number_of_robots_arg,

        # Launch the global_controller node with the integer parameter
        ExecuteProcess(
            cmd=['ros2', 'run', 'global_controller_single_robot', 'global_controller_single_robot'],
            output='screen'
        ),

        # Wait 5 seconds before launching the other nodes
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(turtlebot_controller_launch_file_dir, 'turtlebot_controller_launch.py')),
                    launch_arguments={'namespace': 'tb1'}.items()
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(turtlebot_controller_launch_file_dir, 'turtlebot_controller_launch.py')),
                    launch_arguments={'namespace': 'tb2'}.items()
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(turtlebot_controller_launch_file_dir, 'turtlebot_controller_launch.py')),
                    launch_arguments={'namespace': 'tb3'}.items()
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(turtlebot_controller_launch_file_dir, 'turtlebot_controller_launch.py')),
                    launch_arguments={'namespace': 'tb4'}.items()
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(gazebo_multi_nav2_world_launch_file),
                ),
            ]
        ),
    ])
