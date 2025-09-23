#!/usr/bin/env python3
"""
Configuration generation launch file for URC Rover Arm Template.
This launch file automatically generates URDF, SRDF, and MoveIt configuration files
from the arm_params.yaml file.

Usage:
  ros2 launch arm_template generate_configs.launch.py
  ros2 launch arm_template generate_configs.launch.py validate_only:=true  # Only validate
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Declare launch arguments
    declare_params_arg = DeclareLaunchArgument(
        'arm_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('arm_template'),
            'config',
            'arm_params.yaml'
        ]),
        description='Path to arm parameters file'
    )
    
    declare_validate_only_arg = DeclareLaunchArgument(
        'validate_only',
        default_value='false',
        description='Only validate parameters without generating files'
    )
    
    declare_clean_arg = DeclareLaunchArgument(
        'clean',
        default_value='false',
        description='Clean generated files before regenerating'
    )
    
    # Get launch configurations
    arm_params = LaunchConfiguration('arm_params')
    validate_only = LaunchConfiguration('validate_only')
    clean = LaunchConfiguration('clean')
    
    # Get package paths
    arm_template_share = FindPackageShare('arm_template')
    moveit_config_share = FindPackageShare('moveit_config_template')
    
    # Step 1: Validate parameters
    validate_params = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([arm_template_share, 'scripts', 'validate_params.py']),
            '--params', arm_params
        ],
        output='screen',
        name='validate_params'
    )
    
    # Step 2: Generate URDF (if not validation only)
    generate_urdf = LogInfo(
        msg='Generating URDF from parameters...',
        condition=UnlessCondition(validate_only)
    )
    
    # Step 3: Generate SRDF
    generate_srdf = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([arm_template_share, 'scripts', 'generate_srdf.py']),
            '--params', arm_params,
            '--output', PathJoinSubstitution([
                moveit_config_share, 'srdf', 'arm_template.srdf'
            ])
        ],
        output='screen',
        condition=UnlessCondition(validate_only),
        name='generate_srdf'
    )
    
    # Step 4: Generate MoveIt configuration files
    generate_moveit_config = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([arm_template_share, 'scripts', 'generate_moveit_config.py']),
            '--params', arm_params,
            '--output-dir', PathJoinSubstitution([
                moveit_config_share, 'config'
            ])
        ],
        output='screen',
        condition=UnlessCondition(validate_only),
        name='generate_moveit_config'
    )
    
    # Step 5: Test URDF generation
    test_urdf = ExecuteProcess(
        cmd=[
            'xacro',
            PathJoinSubstitution([arm_template_share, 'urdf', 'arm_template.urdf.xacro']),
            '--check'
        ],
        output='screen',
        condition=UnlessCondition(validate_only),
        name='test_urdf'
    )
    
    # Clean generated files (if requested)
    clean_files = ExecuteProcess(
        cmd=[
            'rm', '-rf',
            PathJoinSubstitution([moveit_config_share, 'config', '*.yaml']),
            PathJoinSubstitution([moveit_config_share, 'srdf', 'arm_template.srdf'])
        ],
        output='screen',
        condition=IfCondition(clean),
        name='clean_files'
    )
    
    # Success message
    success_message = LogInfo(
        msg='✅ Configuration generation complete! You can now run: ros2 launch arm_template demo.launch.py',
        condition=UnlessCondition(validate_only)
    )
    
    validation_success_message = LogInfo(
        msg='✅ Parameter validation complete!',
        condition=IfCondition(validate_only)
    )
    
    return LaunchDescription([
        declare_params_arg,
        declare_validate_only_arg,
        declare_clean_arg,
        clean_files,
        validate_params,
        generate_urdf,
        generate_srdf,
        generate_moveit_config,
        test_urdf,
        success_message,
        validation_success_message
    ]) 