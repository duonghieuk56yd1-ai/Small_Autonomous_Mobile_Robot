import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg         = get_package_share_directory('wheel_odometry')
    odom_config = os.path.join(pkg, 'config', 'odom_params.yaml')

    # The ESP32 firmware publishes the LiDAR directly on /scan. The restamper
    # reads /scan_raw and republishes on /scan, so only enable it after
    # changing the firmware scan topic to "scan_raw"; otherwise it would sit
    # idle (or, with both running, two publishers would share /scan).
    use_scan_restamper = LaunchConfiguration('use_scan_restamper')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_scan_restamper',
            default_value='false',
            description='Restamp /scan_raw -> /scan with PC time '
                        '(requires firmware to publish on scan_raw)',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.04125',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'base_laser',
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=[
                '--x', '-0.0674', '--y', '0.006355', '--z', '-0.0053',
                '--roll', '0', '--pitch', '0', '--yaw', '-1.57',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link',
            ],
        ),

        Node(
            package='wheel_odometry',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            parameters=[odom_config],
            output='screen',
        ),

        Node(
            package='wheel_odometry',
            executable='scan_restamper',
            name='scan_restamper',
            output='screen',
            condition=IfCondition(use_scan_restamper),
        ),

    ])
