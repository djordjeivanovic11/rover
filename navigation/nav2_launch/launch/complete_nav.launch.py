#!/usr/bin/env python3
"""
Complete URC Rover Navigation Launch
Brings up Nav2 stack + Teensy bridge
Expects perception stack (loc_fusion, pointcloud_tools) to be running separately
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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
    
    declare_teensy_port = DeclareLaunchArgument(
        'teensy_port',
        default_value='/dev/ttyACM1',
        description='Serial port for Teensy motor controller'
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
    
    # Teensy bridge for motor control
    teensy_bridge = Node(
        package='nav2_teensy_bridge',
        executable='bridge',
        name='nav2_teensy_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('teensy_port'),
            'baudrate': 115200,
            'max_linear_vel': 0.6,
            'max_angular_vel': 1.8,
            'cmd_timeout': 1.0,
            'retry_connection': True,
        }]
    )
    
    return LaunchDescription([
        declare_params_file,
        declare_use_sim_time,
        declare_autostart,
        declare_use_composition,
        declare_teensy_port,
        
        nav2_bringup,
        teensy_bridge,
    ])

