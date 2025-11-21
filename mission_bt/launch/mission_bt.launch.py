#!/usr/bin/env python3
"""Launch the simple waypoint mission executor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_arg = DeclareLaunchArgument(
        'waypoint_sequence',
        default_value='start_zone,science_1,science_2,return_point',
        description='Comma-separated list of waypoint names to visit in order.',
    )

    retries_arg = DeclareLaunchArgument(
        'max_retries',
        default_value='2',
        description='Number of retries per waypoint.',
    )

    continue_arg = DeclareLaunchArgument(
        'continue_on_failure',
        default_value='false',
        description='Continue mission even if a waypoint fails.',
    )

    mission_node = Node(
        package='mission_bt',
        executable='mission_executor',
        name='mission_bt_executor',
        output='screen',
        parameters=[{
            'waypoint_sequence': LaunchConfiguration('waypoint_sequence'),
            'max_retries': LaunchConfiguration('max_retries'),
            'continue_on_failure': LaunchConfiguration('continue_on_failure'),
        }],
    )

    return LaunchDescription([
        waypoint_arg,
        retries_arg,
        continue_arg,
        mission_node,
    ])

