from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_messenger',
            executable='can_messenger',
        ),
        Node(
            package='ros_servo',
            executable='ros_servo',
        ),
    ])