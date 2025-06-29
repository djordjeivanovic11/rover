#!/usr/bin/env python3
"""
slam.launch.py
==============

One-shot launcher that spins up all perception for the rover:

  1.  zed2i_launch   → camera, depth, IMU, TF frames
  2.  loc_fusion     → dual EKF + navsat_transform ➜ map→odom TF
  3.  RTAB-Map SLAM  → global 3-D cloud (/rtabmap/cloud_map) and
                       2-D occupancy grid (/map)

Down-stream stacks (Nav2, MoveIt, mission control) can simply include
this launch description and inherit every required topic & TF.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def share(pkg: str, relpath: str) -> str:
    return os.path.join(get_package_share_directory(pkg), relpath)


def generate_launch_description():

    zed_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            share('zed2i_launch', 'launch/zed2i_driver.launch.py'))
    )

    fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            share('loc_fusion', 'launch/loc_fusion.launch.py'))
    )

    rtabmap = Node(
        package='rtabmap_launch',
        executable='rtabmap.launch.py',
        name='rtabmap',
        output='screen',
        parameters=[share('slam_launch', 'config/rtabmap_params.yaml')],
        remappings=[
            # RGB image + camera info
            ('rgb/image',         '/zed2i/left/image_rect_color'),
            ('rgb/camera_info',   '/zed2i/left/camera_info'),

            # Depth image + camera info
            ('depth/image',       '/zed2i/depth/depth_registered'),
            ('depth/camera_info', '/zed2i/depth/camera_info'),

            # External odometry from EKF
            ('odom',              '/odometry/filtered')
        ]
    )

    return LaunchDescription([zed_driver, fusion, rtabmap])
