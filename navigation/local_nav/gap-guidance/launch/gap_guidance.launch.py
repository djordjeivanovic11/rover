#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('gap_guidance')
    params  = os.path.join(pkg_dir, 'config', 'params.yaml')

    node = Node(
        package='gap_guidance',
        executable='gap_guidance_node',
        name='gap_guidance',
        output='screen',
        parameters=[params],
        remappings=[
            ('/pointcloud_in', '/zed2i/point_cloud/cloud_registered'),
            ('/gap_cmd',      '/gap_cmd'),
            ('/gap_diag',     '/gap_diag')
        ]
    )
    return LaunchDescription([node])
