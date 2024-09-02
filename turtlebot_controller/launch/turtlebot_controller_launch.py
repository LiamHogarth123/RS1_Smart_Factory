from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Namespace for the TurtleBot controller')

    return LaunchDescription([
        namespace_arg,
        Node(
            package='turtlebot_controller',
            executable='turtlebot_controller',
            name=[LaunchConfiguration('namespace'), TextSubstitution(text='_controller')],
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=[LaunchConfiguration('namespace')],
        ),
    ])
