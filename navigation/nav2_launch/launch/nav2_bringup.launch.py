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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
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
    
    # Drive control arguments
    declare_track_width = DeclareLaunchArgument(
        'track_width', default_value='0.42',
        description='Track width in meters (wheel center to wheel center)'
    )
    declare_wheel_radius = DeclareLaunchArgument(
        'wheel_radius', default_value='0.105',
        description='Wheel radius in meters'
    )

    # Include drive_control (converts /cmd_vel to motor commands)
    drive_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('drive_control'),
                'launch',
                'wheel_drive.launch.py'
            ])
        ]),
        launch_arguments={
            'transport':      'ros',  # Use micro-ROS firmware
            'track_width':    LaunchConfiguration('track_width'),
            'wheel_radius':   LaunchConfiguration('wheel_radius'),
        }.items()
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

    # Include GPS navigator (depends on Nav2 + localization being up)
    gps_navigator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gps_waypoint_navigator'),
                'launch',
                'gps_navigator.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        declare_map_yaml,
        declare_params_file,
        declare_use_sim_time,
        declare_autostart,
        declare_track_width,
        declare_wheel_radius,
        drive_control_launch,  # Motor control (converts Nav2 /cmd_vel to wheel RPM)
        nav2_launch,          # Nav2 stack (planning, control, costmaps)
        gps_navigator_launch  # GPS waypoint navigation bridge
    ])
