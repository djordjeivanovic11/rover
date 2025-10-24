#!/usr/bin/env python3
"""
Launch file for Nav2 Teensy bridge.
Converts Nav2 cmd_vel to Teensy serial commands.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM1',
            description='Serial port for Teensy (NOT same as GPS!)'
        ),
        
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Serial baudrate'
        ),
        
        DeclareLaunchArgument(
            'max_linear_vel',
            default_value='0.6',
            description='Maximum linear velocity (m/s)'
        ),
        
        DeclareLaunchArgument(
            'max_angular_vel',
            default_value='1.8',
            description='Maximum angular velocity (rad/s)'
        ),
        
        Node(
            package='nav2_teensy_bridge',
            executable='bridge',
            name='nav2_teensy_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'max_linear_vel': LaunchConfiguration('max_linear_vel'),
                'max_angular_vel': LaunchConfiguration('max_angular_vel'),
                'cmd_timeout': 1.0,
                'retry_connection': True,
            }]
        )
    ])

