from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # Declare the namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the TurtleBot controller'
    )

    # Return the launch description
    return LaunchDescription([
        # Include the namespace argument in the launch description
        namespace_arg,
        
        # Define the global_controller node with namespace substitution
        Node(
            package='global_controller_single_robot',
            executable='global_controller',
            name=[LaunchConfiguration('namespace'), TextSubstitution(text='_controller')],
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=[LaunchConfiguration('namespace')],
        ),
    ])
