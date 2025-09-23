#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = 'object_detection'
    cfg = os.path.join(
        get_package_share_directory(pkg),
        'config', 'detector_params.yaml'
    )

    return LaunchDescription([
        Node(
            package=pkg,
            executable='object_detection',
            name='object_detection',
            output='screen',
            parameters=[cfg]),
    ])
