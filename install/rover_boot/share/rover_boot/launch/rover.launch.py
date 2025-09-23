#!/usr/bin/env python3
"""
Boot the entire URC rover stack:
  • ZED2i driver
  • SLAM
  • GNSS↔EKF localization
  • Navigation2
  • Gap-guidance reflex
  • Supervisor mux
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_zed   = get_package_share_directory('zed2i_launch')
    pkg_slam  = get_package_share_directory('slam_launch')
    pkg_loc   = get_package_share_directory('loc_fusion')
    pkg_nav2  = get_package_share_directory('nav2_launch')
    pkg_gap   = get_package_share_directory('gap_guidance')
    pkg_boot  = get_package_share_directory('rover_boot')

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_zed,   'launch', 'zed2i_driver.launch.py')
        )
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam,  'launch', 'slam.launch.py')
        )
    )
    loc_fuse = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_loc,   'launch', 'loc_fusion.launch.py')
        )
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2,  'launch', 'nav2_bringup.launch.py')
        ),
        launch_arguments={'autostart':'true'}.items()
    )
    gap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gap,   'launch', 'gap_guidance.launch.py')
        )
    )
    supervisor = ExecuteProcess(
        cmd=['ros2', 'run', 'rover_boot', 'supervisor'],
        output='screen'
    )

    return LaunchDescription([zed, slam, loc_fuse, nav2, gap, supervisor])
