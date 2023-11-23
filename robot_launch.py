from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_main',
            executable='ros_main',
            prefix="xterm -e",
            output="screen",

        ),
        Node(
            package='can_messenger',
            executable='can_messenger',
            prefix="xterm -e",
            output="screen",

        ),
        Node(
            package='ros_image',
            executable='ros_image',
            # prefix="xterm -e",
            output="screen",

        ),
    ])