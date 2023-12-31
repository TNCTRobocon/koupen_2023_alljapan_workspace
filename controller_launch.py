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
        Node(
            package='ros_gui',
            executable='ros_gui',
            prefix="xterm -e",
            output="screen",
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            prefix="xterm -e",
            output="screen",
        ),
    ])