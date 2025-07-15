import os
import subprocess
from math import pi

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Unitree L1 driver
    unitree_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[
            {'port': '/dev/ttyUSB0'},#how do I know its actually usb0?
            {'rotate_yaw_bias': 0.0},
            {'range_scale': 0.001},
            {'range_bias': 0.0},
            {'range_max': 50.0},
            {'range_min': 0.0},
            {'cloud_frame': 'unilidar_lidar'},
            {'cloud_topic': 'unilidar/cloud'},
            {'cloud_scan_num': 18},
            {'imu_frame': 'unilidar_imu'},
            {'imu_topic': 'unilidar/imu'},
        ],
    )

    # PointCloud2 → LaserScan converter not sure if this even works though...
    pcl_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in', '/unilidar/cloud'),
            ('scan', 'scan'),        # publishes on /scan
        ],
        parameters=[{
            'transform_tolerance': 0.05,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -pi,
            'angle_max': pi,
            'angle_increment': pi / 360.0,  # 0.5° per beam
            'scan_time': 1.0 / 10.0,         # target 10 Hz
            'range_min': 0.1,
            'range_max': 50.0,
            'use_inf': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        unitree_node,
        # pcl_node,
    ])