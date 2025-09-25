#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch GNSS driver with health monitoring and ZED integration"""
    
    # Package directory
    pkg_share = get_package_share_directory('gnss_launch')
    
    # Configuration files
    gnss_config = os.path.join(pkg_share, 'params', 'gnss_config.yaml')
    ublox_config = os.path.join(pkg_share, 'params', 'ublox.yaml')
    
    # Launch arguments
    args = [
        DeclareLaunchArgument(
            'gnss_device', default_value='/dev/ttyACM0',
            description='GNSS receiver serial device'
        ),
        DeclareLaunchArgument(
            'gnss_baudrate', default_value='115200',
            description='GNSS serial communication rate'
        ),
        DeclareLaunchArgument(
            'enable_diagnostics', default_value='true',
            description='Enable GNSS health monitoring and diagnostics'
        ),
        DeclareLaunchArgument(
            'output_topic', default_value='/gps/fix',
            description='GNSS fix output topic (for ZED fusion compatibility)'
        ),
        DeclareLaunchArgument(
            'gnss_frame_id', default_value='gnss_link',
            description='TF frame ID for GNSS antenna'
        )
    ]
    
    # Primary GNSS driver
    gnss_driver_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='gnss_driver',
        output='screen',
        parameters=[
            ublox_config,
            gnss_config,
            {
                'device': LaunchConfiguration('gnss_device'),
                'frame_id': LaunchConfiguration('gnss_frame_id'),
                'rate': 10,
                'nav_rate': 5,
                'enable_ppp': False,
                'dynamic_model': 'automotive'
            }
        ],
        remappings=[
            ('fix', LaunchConfiguration('output_topic')),
            ('fix_velocity', '/gnss/vel')
        ]
    )
    
    # GNSS Health Monitor
    gnss_health_node = Node(
        package='gnss_launch',
        executable='gnss_health_monitor',
        name='gnss_health_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_diagnostics')),
        parameters=[
            gnss_config,
            {
                'gnss_topic': LaunchConfiguration('output_topic'),
                'enable_quality_alerts': True
            }
        ]
    )
    
    # GNSS Signal Validator
    gnss_validator_node = Node(
        package='gnss_launch',
        executable='gnss_validator',
        name='gnss_validator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_diagnostics')),
        parameters=[{
            'gnss_topic': LaunchConfiguration('output_topic'),
            'min_satellites': 4,
            'max_hdop': 5.0,
            'validation_rate': 1.0
        }]
    )
    
    return LaunchDescription([
        *args,
        gnss_driver_node,
        gnss_health_node,
        gnss_validator_node
    ])
