from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Namespace for the TurtleBot controller')
    
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml', 
        default_value='/home/liam',  # Provide the default path to your map.yaml file
        description='Full path to the map YAML file to load')

    return LaunchDescription([
        namespace_arg,
        Node(
            package='global_controller_single_robot',
            executable='global_controller',
            name=[LaunchConfiguration('namespace'), TextSubstitution(text='_controller')],
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=[LaunchConfiguration('namespace')],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('map_yaml')}]
        ),
    ])

