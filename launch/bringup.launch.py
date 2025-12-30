from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Forward motion from /cmd_vel to /wirelesscontroller
        Node(
            package='modulr_unitree_go2',
            executable='fwd_motion',
            name='fwd_motion_node',
            output='screen',
        ),

        # Capture images from SDK and send using Zenoh
        Node(
            package='modulr_unitree_go2',
            executable='unitree_video',
            name='unitree_video_node',
            output='screen',
        ),
    ])
