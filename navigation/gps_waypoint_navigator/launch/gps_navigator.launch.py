#!/usr/bin/env python3
"""
Launch the GPS navigator nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    # Get package share directory for config path
    pkg_share = get_package_share_directory("gps_waypoint_navigator")
    waypoint_config = str(Path(pkg_share) / "config" / "waypoints.yaml")

    # Main GPS navigator node (NavigateToGPS action)
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

    # Waypoint sequencer (FollowGPSWaypoints action)
    waypoint_sequencer = Node(
        package="gps_waypoint_navigator",
        executable="waypoint_sequencer",
        name="waypoint_sequencer",
        output="screen",
    )

    # Named waypoint node (GoToNamedWaypoint action)
    named_waypoint = Node(
        package="gps_waypoint_navigator",
        executable="named_waypoint_node",
        name="named_waypoint_node",
        output="screen",
        parameters=[
            {
                "waypoint_config": waypoint_config,
            }
        ],
    )

    return LaunchDescription([
        gps_navigator,
        waypoint_sequencer,
        named_waypoint,
    ])


