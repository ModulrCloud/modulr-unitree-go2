from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RealSense D435I — depth + RGB + IMU
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            output='screen',
            parameters=[{
                'enable_depth': True,
                'enable_color': True,
                'enable_gyro': True,
                'enable_accel': True,
            }],
        ),

        # Static TF: base_link → camera_link
        # RealSense D435I on standard Unitree mount: front of robot, 30° pitch down
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=['0.32715', '0', '0.04297', '0', '-0.5236', '0', 'base_link', 'camera_link'],
            output='screen',
        ),

        # Logitech C270 
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='logitech_camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video6',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'camera_name': 'logitech_c270',
            }],
        ),


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

        # Convert /utlidar/cloud (raw PointCloud2 in lidar frame) to /scan (LaserScan)
        # target_frame: utlidar_lidar — scan stays in lidar frame, slam_toolbox uses TF tree
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'target_frame': 'utlidar_lidar',
                'transform_tolerance': 0.01,
                'queue_size': 5,
                'min_height': 0.05,
                'max_height': 0.4,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00872,
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 30.0,
                'use_inf': True,
            }],
            remappings=[
                ('cloud_in', '/utlidar/cloud'),
                ('scan', '/scan'),
            ],
        ),
    ])
