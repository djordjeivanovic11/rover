#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Launch global localization with camera-GNSS fusion"""

    # Package directories
    pkg_share = get_package_share_directory('zed_integration')
    zed_wrapper_share = get_package_share_directory('zed_wrapper')

    # Configuration files
    global_config = os.path.join(
        pkg_share, 'config', 'global_localization.yaml')
    zed_config = os.path.join(zed_wrapper_share, 'config', 'zed2i.yaml')

    # Launch arguments
    args = [
        DeclareLaunchArgument(
            'camera_model', default_value='zed2i',
            description='Camera model for GNSS fusion'
        ),
        DeclareLaunchArgument(
            'gnss_topic', default_value='/gps/fix',
            description='GNSS fix input topic'
        ),
        DeclareLaunchArgument(
            'enable_fusion', default_value='true',
            description='Enable camera-GNSS fusion'
        ),
        DeclareLaunchArgument(
            'auto_set_origin', default_value='true',
            description='Automatically set map origin from first GPS fix'
        )
    ]

    # Camera with GNSS fusion enabled
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(zed_wrapper_share, 'launch', 'zed_camera.launch.py')
        ]),
        condition=IfCondition(LaunchConfiguration('enable_fusion')),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'config_common_path': zed_config,
            # Enable GNSS fusion
            'gnss_fusion.gnss_fusion_enabled': 'true',
            'gnss_fusion.gnss_fix_topic': LaunchConfiguration('gnss_topic'),
            'gnss_fusion.enable_reinitialization': 'true',
            'gnss_fusion.enable_rolling_calibration': 'true',
            'gnss_fusion.publish_utm_tf': 'true',
            # Disable conflicting features
            'pos_tracking.publish_tf': 'false',  # Let fusion handle transforms
        }.items()
    )

    # Global localization manager
    global_localization_node = Node(
        package='zed_integration',
        executable='global_localization',
        name='global_localization',
        output='screen',
        parameters=[
            global_config,
            {
                'gnss_topic': LaunchConfiguration('gnss_topic'),
                'enable_auto_fallback': True,
                'auto_set_origin': LaunchConfiguration('auto_set_origin')
            }
        ]
    )

    # Coordinate conversion service
    coordinate_converter_node = Node(
        package='zed_integration',
        executable='coordinate_converter',
        name='coordinate_converter',
        output='screen',
        parameters=[{
            'auto_set_origin': LaunchConfiguration('auto_set_origin')
        }]
    )

    return LaunchDescription([
        *args,
        camera_launch,
        global_localization_node,
        coordinate_converter_node
    ])
