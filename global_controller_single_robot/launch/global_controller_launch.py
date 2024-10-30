# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     # Paths to launch files in your workspace
#     turtlebot3_gazebo_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
#     turtlebot3_navigation2_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch')

#     # Declare the integer launch argument
#     number_of_robots_arg = DeclareLaunchArgument(
#         'number_of_robots',
#         default_value='10',  # Default value for the parameter
#         description='An integer parameter for the global planner'
#     )

#     # Fetch the LaunchConfiguration for the number_of_robots
#     number_of_robots = LaunchConfiguration('number_of_robots')

#     return LaunchDescription([
#         # Declare the argument
#         number_of_robots_arg,


#         # # Launch TurtleBot3 Navigation2
#         # IncludeLaunchDescription(
#         #     PythonLaunchDescriptionSource(os.path.join(turtlebot3_navigation2_launch_file_dir, 'navigation2.launch.py')),
#         #     launch_arguments={
#         #         'use_sim_time': 'true',
#         #     }.items()
#         # ),

#         # # Launch Gazebo with TurtleBot3
#         # IncludeLaunchDescription(
#         #     PythonLaunchDescriptionSource(os.path.join(turtlebot3_gazebo_launch_file_dir, 'turtlebot3_world.launch.py')),
#         #     launch_arguments={
#         #         'use_sim_time': 'true',
#         #     }.items()
#         # ),


#         # Launch the global_controller node with the integer parameter
#         Node(
#             package='global_controller_single_robot',
#             executable='global_controller',
#             name='global_controller',  # Explicitly ensure the node name is set correctly
#             output='screen',
#             parameters=[{
#                 'number_of_robots': number_of_robots
#             }]
#         ),
#     ])



from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Declare the namespace argument
    # namespace_arg = DeclareLaunchArgument(
    #     'namespace',
    #     default_value='',
    #     description='Namespace for the TurtleBot controller'
    # )

    # Return the launch description
    return LaunchDescription([
        # Include the namespace argument in the launch description
        # namespace_arg,
        
        # Define the global_controller node with namespace substitution
        ExecuteProcess(
            cmd=['ros2', 'run', 'global_controller_single_robot', 'global_controller_single_robot'],
            output='screen'
        ),
    ])