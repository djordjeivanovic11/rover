#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Complete GNSS driver with multiple receiver support"""
    
    # Package directory
    pkg_share = get_package_share_directory('gnss_launch')
    
    # Configuration files
    ublox_config = os.path.join(pkg_share, 'params', 'ublox.yaml')
    gnss_config = os.path.join(pkg_share, 'params', 'gnss_config.yaml')
    
    # Launch arguments
    args = [
        DeclareLaunchArgument(
            'gnss_driver_type', default_value='ublox',
            description='GNSS driver type: ublox, nmea, septentrio'
        ),
        DeclareLaunchArgument(
            'gnss_device', default_value='/dev/ttyACM0',
            description='GNSS receiver serial device'
        ),
        DeclareLaunchArgument(
            'gnss_baudrate', default_value='115200',
            description='Serial communication baudrate'
        ),
        DeclareLaunchArgument(
            'output_topic', default_value='/gps/fix',
            description='GNSS fix output topic (ZED fusion compatible)'
        ),
        DeclareLaunchArgument(
            'enable_rtk', default_value='false',
            description='Enable RTK corrections for centimeter accuracy'
        ),
        DeclareLaunchArgument(
            'rtk_correction_topic', default_value='/rtk/corrections',
            description='RTK correction data topic'
        )
    ]
    
    # u-blox Driver (most common)
    ublox_driver_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('gnss_driver_type').equals('ublox')),
        actions=[
            Node(
                package='ublox_gps',
                executable='ublox_gps_node',
                name='gnss_driver',
                output='screen',
                parameters=[
                    ublox_config,
                    {
                        'device': LaunchConfiguration('gnss_device'),
                        'frame_id': 'gnss_link',
                        'rate': 10,
                        'nav_rate': 5,
                        'enable_ppp': False,
                        'dynamic_model': 'automotive'
                    }
                ],
                remappings=[
                    ('fix', LaunchConfiguration('output_topic')),
                    ('fix_velocity', '/gnss/velocity')
                ]
            )
        ]
    )
    
    # NMEA Driver (generic)
    nmea_driver_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('gnss_driver_type').equals('nmea')),
        actions=[
            Node(
                package='nmea_navsat_driver',
                executable='nmea_serial_driver',
                name='gnss_driver',
                output='screen',
                parameters=[{
                    'port': LaunchConfiguration('gnss_device'),
                    'baud': LaunchConfiguration('gnss_baudrate'),
                    'frame_id': 'gnss_link'
                }],
                remappings=[
                    ('fix', LaunchConfiguration('output_topic'))
                ]
            )
        ]
    )
    
    # Septentrio Driver (high-end)
    septentrio_driver_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('gnss_driver_type').equals('septentrio')),
        actions=[
            Node(
                package='septentrio_gnss_driver',
                executable='septentrio_gnss_driver_node',
                name='gnss_driver',
                output='screen',
                parameters=[{
                    'device': LaunchConfiguration('gnss_device'),
                    'frame_id': 'gnss_link',
                    'use_gnss_time': False,
                    'polling_period': 100  # 10 Hz
                }],
                remappings=[
                    ('fix', LaunchConfiguration('output_topic'))
                ]
            )
        ]
    )
    
    # GNSS Health Monitor (always active)
    gnss_health_node = Node(
        package='gnss_launch',
        executable='gnss_health_monitor',
        name='gnss_health_monitor',
        output='screen',
        parameters=[
            gnss_config,
            {
                'gnss_topic': LaunchConfiguration('output_topic')
            }
        ]
    )
    
    # GNSS Signal Validator (always active)
    gnss_validator_node = Node(
        package='gnss_launch',
        executable='gnss_validator',
        name='gnss_validator',
        output='screen',
        parameters=[{
            'gnss_topic': LaunchConfiguration('output_topic'),
            'min_satellites': 4,
            'max_hdop': 5.0,
            'validation_rate': 1.0
        }]
    )
    
    # RTK Corrections (optional)
    rtk_corrections_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_rtk')),
        actions=[
            Node(
                package='rtk_corrections',
                executable='ntrip_client',
                name='rtk_corrections',
                output='screen',
                parameters=[{
                    'correction_topic': LaunchConfiguration('rtk_correction_topic')
                }]
            )
        ]
    )
    
    return LaunchDescription([
        *args,
        ublox_driver_group,
        nmea_driver_group, 
        septentrio_driver_group,
        gnss_health_node,
        gnss_validator_node,
        rtk_corrections_group
    ])
