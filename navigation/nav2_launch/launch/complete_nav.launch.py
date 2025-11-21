#!/usr/bin/env python3
"""
Complete URC Rover Navigation Launch
Brings up Nav2 stack + motor control bridge
Expects perception stack (loc_fusion, pointcloud_tools) to be running separately
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = get_package_share_directory('nav2_launch')
    
    # Path to our custom params
    params_file = os.path.join(nav2_launch_dir, 'config', 'nav2_params.yaml')
    
    # Launch arguments
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to Nav2 params file'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup Nav2 stack'
    )
    
    declare_use_composition = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Use composed bringup (faster but harder to debug)'
    )
    
    declare_transport = DeclareLaunchArgument(
        'transport',
        default_value='ros',
        description='Transport mode: ros (micro-ROS topics), serial, or udp'
    )
    
    declare_teensy_port = DeclareLaunchArgument(
        'teensy_port',
        default_value='/dev/ttyACM1',
        description='Serial port for Teensy motor controller (serial mode only)'
    )
    
    declare_track_width = DeclareLaunchArgument(
        'track_width',
        default_value='0.42',
        description='Track width in meters'
    )
    
    declare_wheel_radius = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.105',
        description='Wheel radius in meters'
    )
    
    # Nav2 bringup (without localization - we use loc_fusion)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': LaunchConfiguration('params_file'),
            'use_composition': LaunchConfiguration('use_composition'),
        }.items()
    )
    
    # Motor control bridge (uses drive_control package)
    motor_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('drive_control'),
                'launch',
                'wheel_drive.launch.py'
            ])
        ]),
        launch_arguments={
            'transport': LaunchConfiguration('transport'),
            'serial_port': LaunchConfiguration('teensy_port'),
            'track_width': LaunchConfiguration('track_width'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
        }.items()
    )
    
    return LaunchDescription([
        declare_params_file,
        declare_use_sim_time,
        declare_autostart,
        declare_use_composition,
        declare_transport,
        declare_teensy_port,
        declare_track_width,
        declare_wheel_radius,
        
        nav2_bringup,
        motor_control,
    ])

