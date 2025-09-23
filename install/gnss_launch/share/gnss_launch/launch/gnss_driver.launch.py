#!/usr/bin/env python3
"""
Starts the GNSS driver so /gnss/fix is available for navsat_transform.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('gnss_launch'),
        'params', 'ublox.yaml')

    gnss_node = Node(
        package='ublox',
        executable='ublox_node',
        name='ublox_gnss',
        output='screen',
        parameters=[params],
        remappings=[
            ('fix', '/gnss/fix')
        ])

    return LaunchDescription([gnss_node])
