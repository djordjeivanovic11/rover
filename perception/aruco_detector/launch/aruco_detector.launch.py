#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_detector')
    params = os.path.join(pkg_share, 'config', 'detector_params.yaml')

    return LaunchDescription([
        Node(package='aruco_detector',
             executable='aruco_detector_node',
             name='aruco_detector',
             output='screen',
             parameters=[params])
    ])
