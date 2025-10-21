#!/usr/bin/env python3
"""
=============================================================================
ARM CONTROL LAUNCH FILE
=============================================================================
Launches the essential arm control nodes for URC missions:
- Gripper controller
- Action servers (GoToNamedPose, PickAndPlace)
- Safety monitor

Use this alongside moveit_config_rover for complete arm control.
=============================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Gripper Controller
    gripper_controller = Node(
        package='arm_control',
        executable='gripper_controller',
        name='gripper_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Action Servers
    action_servers = Node(
        package='arm_control',
        executable='action_servers',
        name='arm_action_servers',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Safety Monitor
    safety_monitor = Node(
        package='arm_control',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        gripper_controller,
        action_servers,
        safety_monitor,
    ])
