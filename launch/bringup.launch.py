from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Forward motion from /cmd_vel to /api/sport/request via SportClient
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

        # Convert /sportmodestate to /odom and publish odom→base_link TF
        Node(
            package='modulr_unitree_go2',
            executable='odom_bridge',
            name='odom_bridge_node',
            output='screen',
        ),

        # Static TF: base_link → utlidar_lidar (Mid-360 mount position from Unitree docs)
        # Translation: x=0.1870, y=0.0, z=0.0803, Rotation: 13° around Y axis (pitch=0.2269 rad)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar',
            arguments=['0.1870', '0', '0.0803', '0', '0.2269', '0', 'base_link', 'utlidar_lidar'],
            output='screen',
        ),

        # Convert /utlidar/cloud_deskewed (motion-corrected PointCloud2 in odom frame)
        # to /scan (LaserScan) for Nav2 AMCL and costmap
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'target_frame': 'odom',
                'transform_tolerance': 0.01,
                'min_height': -0.5,
                'max_height': 0.5,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00872,
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 30.0,
                'use_inf': True,
            }],
            remappings=[
                ('cloud_in', '/utlidar/cloud_deskewed'),
                ('scan', '/scan'),
            ],
        ),
    ])
