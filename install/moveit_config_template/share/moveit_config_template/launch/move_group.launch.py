#!/usr/bin/env python3
"""
MoveIt Move Group launch file for URC Rover Arm Template.
This file launches the core MoveIt move_group node with all necessary configurations.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

def load_yaml_file(file_path):
    """Load a YAML file and return its content."""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        return {}

def generate_move_group_node(context, *args, **kwargs):
    """Generate the move_group node with all configurations."""
    
    # Get launch arguments
    arm_params_path = LaunchConfiguration('arm_params').perform(context)
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    
    # Load arm parameters
    arm_params = load_yaml_file(arm_params_path)
    if not arm_params:
        raise RuntimeError(f"Failed to load arm parameters from {arm_params_path}")
    
    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("urc_arm_template")
        .robot_description_file(
            PathJoinSubstitution([
                FindPackageShare('arm_template'),
                'urdf',
                'arm_template.urdf.xacro'
            ])
        )
        .robot_description_semantic_file(
            PathJoinSubstitution([
                FindPackageShare('moveit_config_template'),
                'srdf',
                'arm_template.srdf'
            ])
        )
        .trajectory_execution_file(
            PathJoinSubstitution([
                FindPackageShare('moveit_config_template'),
                'config',
                'trajectory_execution.yaml'
            ])
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .moveit_cpp_file(
            PathJoinSubstitution([
                FindPackageShare('moveit_config_template'),
                'config',
                'moveit_cpp.yaml'
            ])
        )
        .to_moveit_configs()
    )
    
    # Additional configuration files
    kinematics_yaml = load_yaml_file(
        PathJoinSubstitution([
            FindPackageShare('moveit_config_template'),
            'config',
            'kinematics.yaml'
        ]).perform(context)
    )
    
    joint_limits_yaml = load_yaml_file(
        PathJoinSubstitution([
            FindPackageShare('moveit_config_template'),
            'config',
            'joint_limits.yaml'
        ]).perform(context)
    )
    
    moveit_controllers_yaml = load_yaml_file(
        PathJoinSubstitution([
            FindPackageShare('moveit_config_template'),
            'config',
            'moveit_controllers.yaml'
        ]).perform(context)
    )
    
    planning_scene_monitor_yaml = load_yaml_file(
        PathJoinSubstitution([
            FindPackageShare('moveit_config_template'),
            'config',
            'planning_scene_monitor.yaml'
        ]).perform(context)
    )
    
    # Merge all configuration parameters
    move_group_parameters = [
        moveit_config.to_dict(),
        kinematics_yaml,
        joint_limits_yaml,
        moveit_controllers_yaml,
        planning_scene_monitor_yaml,
        {'use_sim_time': use_fake_hardware == 'true'}
    ]
    
    # MoveIt Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_parameters,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return [move_group_node]

def generate_launch_description():
    
    # Declare launch arguments
    declare_arm_params_arg = DeclareLaunchArgument(
        'arm_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('arm_template'),
            'config',
            'arm_params.yaml'
        ]),
        description='Path to arm parameters file'
    )
    
    declare_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use fake hardware interface for simulation'
    )
    
    # Generate move group node
    move_group_launch = OpaqueFunction(function=generate_move_group_node)
    
    return LaunchDescription([
        declare_arm_params_arg,
        declare_fake_hardware_arg,
        move_group_launch
    ]) 