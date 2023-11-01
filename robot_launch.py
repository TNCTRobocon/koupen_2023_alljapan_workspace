from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_main',
            executable='ros_main',

        ),
        Node(
            package='can_messenger',
            executable='can_messenger',

        ),
    ])