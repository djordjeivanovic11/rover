#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """Launch complete perception with global localization and spatial mapping"""
    
    # Package directories
    pkg_share = get_package_share_directory('zed_integration')
    
    # Launch arguments
    args = [
        DeclareLaunchArgument(
            'enable_global_localization', default_value='true',
            description='Enable camera-GNSS fusion for global localization'
        ),
        DeclareLaunchArgument(
            'enable_spatial_mapping', default_value='true',
            description='Enable GPU-accelerated spatial mapping'
        ),
        DeclareLaunchArgument(
            'gnss_topic', default_value='/gps/fix',
            description='GNSS input topic'
        ),
        DeclareLaunchArgument(
            'mapping_resolution', default_value='0.05',
            description='Spatial mapping resolution (meters)'
        )
    ]
    
    # Global localization system
    global_localization_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_global_localization')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_share, 'launch', 'global_localization.launch.py')
                ]),
                launch_arguments={
                    'gnss_topic': LaunchConfiguration('gnss_topic'),
                    'enable_fusion': LaunchConfiguration('enable_global_localization')
                }.items()
            )
        ]
    )
    
    # Spatial mapping system
    spatial_mapping_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_spatial_mapping')),
        actions=[
            TimerAction(
                period=3.0,  # Start after camera initialization
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            os.path.join(pkg_share, 'launch', 'spatial_mapping.launch.py')
                        ]),
                        launch_arguments={
                            'mapping_resolution': LaunchConfiguration('mapping_resolution'),
                            'publish_occupancy_grid': 'true'
                        }.items()
                    )
                ]
            )
        ]
    )
    
    return LaunchDescription([
        *args,
        global_localization_group,
        spatial_mapping_group
    ])
