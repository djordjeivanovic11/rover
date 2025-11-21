#!/usr/bin/env python3
"""
Launch the GPS navigator node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gps_navigator = Node(
        package="gps_waypoint_navigator",
        executable="gps_navigator_node",
        name="gps_navigator",
        output="screen",
        parameters=[
            {
                "goal_timeout_sec": 120.0,
                "min_gps_quality_m": 10.0,
            }
        ],
    )

    return LaunchDescription([gps_navigator])


