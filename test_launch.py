from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='can_messenger',
        #     executable='can_messenger',
        # ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            prefix="xterm -e",
            output="screen",
        ),

    ])