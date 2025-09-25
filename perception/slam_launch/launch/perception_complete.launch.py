#!/usr/bin/env python3
"""
URC Complete Perception System
=============================

Master launch file for the complete rover perception stack.
Orchestrates all perception components with proper dependencies and monitoring.

Author: URC Perception Team
License: Apache-2.0
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, 
    GroupAction, TimerAction, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def get_package_path(package: str, *paths: str) -> str:
    """Get path within a ROS2 package"""
    return os.path.join(get_package_share_directory(package), *paths)


def generate_launch_description():
    """Generate the complete perception launch description"""
    
    # ============================================================================
    # LAUNCH ARGUMENTS
    # ============================================================================
    
    args = [
        DeclareLaunchArgument(
            'enable_object_detection', default_value='true',
            description='Enable YOLO object detection pipeline'
        ),
        DeclareLaunchArgument(
            'enable_aruco_detection', default_value='true', 
            description='Enable ArUco marker detection'
        ),
        DeclareLaunchArgument(
            'enable_pose_estimation', default_value='true',
            description='Enable 2D→3D pose estimation from detections'
        ),
        DeclareLaunchArgument(
            'enable_recording', default_value='false',
            description='Enable automatic data recording'
        ),
        DeclareLaunchArgument(
            'enable_monitoring', default_value='true',
            description='Enable system health monitoring'
        ),
        DeclareLaunchArgument(
            'debug_mode', default_value='false',
            description='Enable debug logging and visualization'
        ),
        DeclareLaunchArgument(
            'enable_semantic_mapping', default_value='true',
            description='Enable semantic mapping integration'
        ),
        DeclareLaunchArgument(
            'enable_sensor_fusion', default_value='true',
            description='Enable multi-modal sensor fusion'
        ),
        DeclareLaunchArgument(
            'enable_perception_guardian', default_value='true',
            description='Enable perception robustness and error handling'
        ),
        DeclareLaunchArgument(
            'enable_calibration_validation', default_value='true',
            description='Enable sensor calibration validation'
        ),
        DeclareLaunchArgument(
            'enable_rviz', default_value='true',
            description='Enable RViz visualization with rover model'
        )
    ]
    
    # ============================================================================
    # CORE PERCEPTION STACK (Always Active)
    # ============================================================================
    
    # Base SLAM system (ZED2i + Localization + Mapping)
    core_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_path('slam_launch', 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'debug': LaunchConfiguration('debug_mode')
        }.items()
    )
    
    # Pointcloud processing tools (scan + occupancy grid)
    pointcloud_processing = TimerAction(
        period=3.0,  # Wait for SLAM to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_path('pointcloud_tools', 'launch', 'pointcloud_tools.launch.py')
                )
            )
        ]
    )
    
    # ============================================================================
    # DETECTION COMPONENTS (Conditional)
    # ============================================================================
    
    # YOLO Object Detection
    object_detection_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_object_detection')),
        actions=[
            LogInfo(msg="Starting YOLO object detection..."),
            TimerAction(
                period=5.0,  # Wait for camera to stabilize
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            get_package_path('object_detection', 'launch', 'object_detection.launch.py')
                        )
                    )
                ]
            )
        ]
    )
    
    # ArUco Marker Detection  
    aruco_detection_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_aruco_detection')),
        actions=[
            LogInfo(msg="Starting ArUco marker detection..."),
            TimerAction(
                period=5.0,  # Wait for camera to stabilize
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            get_package_path('aruco_detector', 'launch', 'aruco_detector.launch.py')
                        )
                    )
                ]
            )
        ]
    )
    
    
    # ============================================================================
    # ADVANCED PERCEPTION FEATURES
    # ============================================================================
    
    # Multi-Object Tracking (replaces basic pose estimation)
    multi_object_tracking = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_pose_estimation')),
        actions=[
            LogInfo(msg="Starting multi-object tracking..."),
            TimerAction(
                period=8.0,  # Wait for detections to be available
                actions=[
                    Node(
                        package='detection_pose_estimator',
                        executable='pose_estimator',
                        name='pose_estimator',
                        output='screen',
                        parameters=[
                            get_package_path('detection_pose_estimator', 'config', 'estimator_params.yaml')
                        ]
                    )
                ]
            )
        ]
    )
    
    # Semantic Mapping
    semantic_mapping = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_semantic_mapping')),
        actions=[
            LogInfo(msg="Starting semantic mapping..."),
            TimerAction(
                period=10.0,  # Wait for tracking to be established
                actions=[
                    Node(
                        package='slam_launch',
                        executable='semantic_mapper.py',
                        name='semantic_mapper',
                        output='screen',
                        parameters=[{
                            'poses_topic': '/object_pose_array',
                            'tracking_ids_topic': '/object_tracking_ids',
                            'detections_topic': '/detected_objects'
                        }]
                    ),
                    # Object Selection Service
                    Node(
                        package='object_detection',
                        executable='select_object_service',
                        name='object_selection_service',
                        output='screen'
                    )
                ]
            )
        ]
    )
    
    # Multi-Modal Sensor Fusion
    sensor_fusion = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_sensor_fusion')),
        actions=[
            LogInfo(msg="Starting multi-modal sensor fusion..."),
            TimerAction(
                period=9.0,  # Wait for both detection systems
                actions=[
                    Node(
                        package='slam_launch',
                        executable='sensor_fusion.py',
                        name='sensor_fusion',
                        output='screen',
                        parameters=[{
                            'yolo_detections_topic': '/detected_objects',
                            'aruco_detections_topic': '/aruco_detections'
                        }]
                    )
                ]
            )
        ]
    )
    
    # Perception Guardian (Robustness & Error Handling)
    perception_guardian = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_perception_guardian')),
        actions=[
            LogInfo(msg="Starting perception guardian..."),
            TimerAction(
                period=2.0,  # Start early to monitor other components
                actions=[
                    Node(
                        package='slam_launch',
                        executable='perception_guardian.py',
                        name='perception_guardian',
                        output='screen',
                        parameters=[{
                            'enable_auto_recovery': True,
                            'quality_check_rate': 5.0,
                            'min_operational_sensors': 2
                        }]
                    )
                ]
            )
        ]
    )
    
    # Calibration Validation
    calibration_validation = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_calibration_validation')),
        actions=[
            LogInfo(msg="Starting calibration validation..."),
            TimerAction(
                period=3.0,  # Start after sensors are initialized
                actions=[
                    Node(
                        package='slam_launch',
                        executable='calibration_validator.py',
                        name='calibration_validator',
                        output='screen',
                        parameters=[{
                            'calibration_data_dir': '/home/rover/calibration_data',
                            'validation_frequency': 1.0,
                            'auto_save_calibration': True
                        }]
                    )
                ]
            )
        ]
    )
    
    # ============================================================================
    # SYSTEM MONITORING & UTILITIES
    # ============================================================================
    
    # Perception Health Monitor
    health_monitor = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_monitoring')),
        actions=[
            LogInfo(msg="Starting perception health monitoring..."),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package='slam_launch',
                        executable='perception_health_monitor.py',
                        name='perception_health_monitor',
                        output='screen',
                        parameters=[
                            get_package_path('slam_launch', 'config', 'monitoring_config.yaml')
                        ]
                    )
                ]
            )
        ]
    )
    
    # Data Recording System
    recording_system = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_recording')),
        actions=[
            LogInfo(msg="Starting perception data recording..."),
            TimerAction(
                period=10.0,  # Wait for all systems to be ready
                actions=[
                    Node(
                        package='slam_launch', 
                        executable='perception_recorder.py',
                        name='perception_data_recorder',
                        output='screen',
                        parameters=[
                            get_package_path('slam_launch', 'config', 'recording_config.yaml')
                        ]
                    )
                ]
            )
        ]
    )
    
    # System Status Publisher
    status_publisher = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='slam_launch',
                executable='system_status.py', 
                name='perception_status',
                output='screen',
                parameters=[{
                    'system_name': 'URC_Perception_Stack',
                    'publish_rate': 2.0,
                    'monitor_components': [
                        'zed2i_camera', 'rtabmap', 'object_detection', 
                        'aruco_detector', 'pose_estimator'
                    ]
                }]
            )
        ]
    )
    
    # RViz Visualization with Rover Model
    rviz_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
        actions=[
            LogInfo(msg="Starting RViz with rover model..."),
            TimerAction(
                period=5.0,  # Start after other components are ready
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            share('rover_description', 'launch/display.launch.py')
                        ]),
                        launch_arguments={
                            'use_sim_time': 'false'
                        }.items()
                    )
                ]
            )
        ]
    )
    
    # ============================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # ============================================================================
    
    return LaunchDescription([
        # Launch arguments
        *args,
        
        # Startup message
        LogInfo(msg="🚀 Starting URC Complete Perception System..."),
        
        # Core systems (sequential startup)
        core_slam,
        pointcloud_processing,
        
        # Detection components (parallel after core is ready)
        object_detection_group,
        aruco_detection_group, 
        
        # Advanced perception features
        multi_object_tracking,
        semantic_mapping,
        sensor_fusion,
        
        # Robustness and validation
        perception_guardian,
        calibration_validation,
        
        # System utilities
        health_monitor,
        status_publisher,
        recording_system,
        
        # Visualization
        rviz_group,
        
        # Ready message
        TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg="✅ URC Perception System fully operational!")
            ]
        )
    ])
