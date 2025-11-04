#!/usr/bin/env python3
"""
Wheel-Speed Based Drive Control Launch File

Launches the complete wheel-speed control pipeline:
  Nav2 → /cmd_vel → twist_to_wheels → /cmd_wheels → wheel_bridge → UDP → Teensy

This replaces cmd_vel_bridge.launch.py with a cleaner architecture
that gives firmware explicit left/right wheel speeds.

Usage:
  ros2 launch drive_control wheel_drive.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Declare launch arguments (can be overridden)
    return LaunchDescription([
        DeclareLaunchArgument('track_width', default_value='0.42',
                             description='Track width in meters'),
        DeclareLaunchArgument('wheel_radius', default_value='0.105',
                             description='Wheel radius in meters'),
        DeclareLaunchArgument('max_wheel_speed', default_value='2.0',
                             description='Max wheel speed in m/s'),
        DeclareLaunchArgument('max_rpm', default_value='15000',
                             description='Max motor RPM'),
        DeclareLaunchArgument('transport', default_value='serial',
                             description='Transport: serial or udp'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM1',
                             description='Serial port for Teensy'),
        DeclareLaunchArgument('udp_host', default_value='192.168.0.10',
                             description='UDP host address'),
        DeclareLaunchArgument('udp_port', default_value='3000',
                             description='UDP port'),
        
        
        # Node 1: Convert Twist (body velocities) to wheel speeds
        Node(
            package='drive_control',
            executable='twist_to_wheels',
            name='twist_to_wheels',
            output='screen',
            parameters=[{
                'track_width_m': LaunchConfiguration('track_width'),
                'max_wheel_speed_mps': LaunchConfiguration('max_wheel_speed'),
            }]
        ),
        
        # Node 2: Convert wheel speeds (m/s) to RPM and send to Teensy
        Node(
            package='drive_control',
            executable='wheel_bridge',
            name='wheel_bridge',
            output='screen',
            parameters=[{
                'transport': LaunchConfiguration('transport'),
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': 115200,
                'udp_host': LaunchConfiguration('udp_host'),
                'udp_port': LaunchConfiguration('udp_port'),
                'rate_hz': 30.0,
                'timeout_ms': 250,
                'wheel_radius_m': LaunchConfiguration('wheel_radius'),
                'max_rpm': LaunchConfiguration('max_rpm'),
                'invert_left': False,
                'invert_right': False,
                'max_rpm_step': 500,
            }]
        ),
    ])

