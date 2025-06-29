#!/usr/bin/env python3
"""
Launches the D* Lite path planner node.
Subscribes to /map and /goal, publishes /planned_path.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('path_planner')
    cfg = os.path.join(pkg, 'config', 'cost_params.yaml')

    planner_node = Node(
        package='path_planner',
        executable='path_planner_node',
        name='path_planner',
        output='screen',
        parameters=[cfg],
        remappings=[
            ('/map', '/map'),
            ('/goal', '/goal'),
            ('/planned_path', '/planned_path')
        ]
    )
    return LaunchDescription([planner_node])
