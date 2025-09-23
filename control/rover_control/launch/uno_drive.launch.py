#!/usr/bin/env python3
"""
=============================================================================
ARDUINO UNO DRIVE LAUNCH FILE
=============================================================================
Simplified launch file for Arduino Uno drive system:
- Drive bridge for serial communication
- Joystick teleop only (no Nav2)
- Basic safety monitoring
- Designed for simplicity and reliability
=============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino communication'
    )
    
    enable_teleop_arg = DeclareLaunchArgument(
        'enable_teleop',
        default_value='true',
        description='Enable joystick teleop'
    )
    
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.48',
        description='Distance between left and right wheels (meters)'
    )
    
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='1.0',
        description='Maximum linear velocity (m/s)'
    )
    
    # Drive Bridge Node
    drive_bridge = Node(
        package='rover_control',
        executable='drive_bridge',
        name='drive_bridge',
        parameters=[
            {'serial_port': LaunchConfiguration('serial_port')},
            {'wheel_separation': LaunchConfiguration('wheel_separation')},
            {'max_velocity': LaunchConfiguration('max_velocity')},
            {'send_rate': 20.0}  # 20 Hz for simple operation
        ],
        output='screen'
    )
    
    # Joy Node for joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            {'dev': '/dev/input/js0'},
            {'deadzone': 0.1},
            {'autorepeat_rate': 20.0}
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_teleop'))
    )
    
    # Teleop Twist Joy Node
    teleop_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[
            {'axis_linear.x': 1},      # Left stick vertical
            {'axis_angular.yaw': 0},   # Left stick horizontal
            {'scale_linear.x': LaunchConfiguration('max_velocity')},
            {'scale_angular.yaw': 2.0},  # Max angular speed (rad/s)
            {'enable_button': 4},      # L1/LB button to enable
            {'enable_turbo_button': 5} # R1/RB button for turbo
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_teleop'))
    )
    
    return LaunchDescription([
        # Launch arguments
        serial_port_arg,
        enable_teleop_arg,
        wheel_separation_arg,
        max_velocity_arg,
        
        # Core nodes
        drive_bridge,
        
        # Teleop nodes (optional)
        joy_node,
        teleop_joy,
    ]) 