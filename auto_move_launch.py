from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            prefix="xterm -e",
            output="screen",
        ),
        Node(
            package='realsense_pub',
            executable='realsense_pub',
            prefix="xterm -e",
            output="screen",
        ),
        
        Node(
            package='image_transport',
            executable='republish',
            arguments=["raw"],
            remappings=[
                ('in','/image_topic'),
                ('out','/image_topic_compressed')
            ]
        ),
        Node(
            package='image_transport',
            executable='republish',
            arguments=["raw"],
            remappings=[
                ('in','/depth_topic'),
                ('out','/depth_topic_compressed')
            ]
        ),
        Node(
            package='ros_joy',
            executable='ros_joy',
            prefix="xterm -e",
            output="screen",
        ),
        Node(
            package='ros_cv',
            executable='ros_cv',
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
            package='rqt_image_view',
            executable='rqt_image_view',
        ),
        # Node(
        #     package='rqt_graph',
        #     executable='rqt_graph',
        # ),

    ])