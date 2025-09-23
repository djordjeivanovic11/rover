#!/usr/bin/env python3
"""
Launches the full Nav2 stack:
  • map_server  (static map from RTAB-Map or YAML)
  • lifecycle_manager
  • planner_server (A*/NavfnPlanner)
  • controller_server (DWB)
  • behavior_server + bt_navigator
  • costmap_global and costmap_local
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_this = get_package_share_directory('nav2_launch')
    config_dir = os.path.join(pkg_this, 'config')

    # Arguments
    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(config_dir, 'map.yaml'),
        description='Full path to map file to load'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(config_dir, 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start all nav2 nodes'
    )

    # Include the standard nav2 launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'map':            LaunchConfiguration('map'),
            'use_sim_time':   LaunchConfiguration('use_sim_time'),
            'autostart':      LaunchConfiguration('autostart'),
            'params_file':    LaunchConfiguration('params_file')
        }.items()
    )

    return LaunchDescription([
        declare_map_yaml,
        declare_params_file,
        declare_use_sim_time,
        declare_autostart,
        nav2_launch
    ])
