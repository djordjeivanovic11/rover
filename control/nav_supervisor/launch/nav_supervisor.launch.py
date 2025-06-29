#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_supervisor',
            executable='nav_supervisor',
            name='nav_supervisor',
            output='screen',
            remappings=[
                ('/cmd_vel',        '/cmd_vel'),         # final output
                ('/cmd_vel_nav2',   '/cmd_vel_nav2'),    # from Nav2
                ('/gap_cmd',        '/gap_cmd'),         # from gap guidance
            ]
        )
    ])
