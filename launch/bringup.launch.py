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

        # Forward images from SDK node to /camera/image_raw
        Node(
            package='modulr_unitree_go2',
            executable='fwd_camera',
            name='fwd_camera_node',
            output='screen',
        ),

        # Capture images from SDK and send over socket
        Node(
            package='modulr_unitree_go2',
            executable='unitree_video_zmq',  # replace with your executable name
            name='unitree_video_zmq_node',
            output='screen',
        ),
    ])
