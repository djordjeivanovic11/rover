#!/usr/bin/env python3
"""
Demo launch file for URC Rover Arm Template.
This launches the complete arm system with RViz for visualization and planning.

Usage:
  ros2 launch arm_template demo.launch.py
  ros2 launch arm_template demo.launch.py rviz:=false     # Without RViz
  ros2 launch arm_template demo.launch.py use_fake_hardware:=false  # Real hardware
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Declare launch arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz for visualization'
    )
    
    declare_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use fake hardware interface for simulation'
    )
    
    declare_arm_params_arg = DeclareLaunchArgument(
        'arm_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('arm_template'),
            'config',
            'arm_params.yaml'
        ]),
        description='Path to arm parameters file'
    )
    
    # Get launch configurations
    rviz = LaunchConfiguration('rviz')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    arm_params = LaunchConfiguration('arm_params')
    
    # Robot State Publisher
    robot_description_content = ExecuteProcess(
        cmd=['xacro', 
             PathJoinSubstitution([
                 FindPackageShare('arm_template'),
                 'urdf',
                 'arm_template.urdf.xacro'
             ])],
        output='screen'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': use_fake_hardware}
        ],
        output='screen'
    )
    
    # Joint State Publisher (for fake hardware)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[arm_params],
        condition=IfCondition(use_fake_hardware),
        output='screen'
    )
    
    # Joint State Publisher GUI (for manual control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_fake_hardware),
        output='screen'
    )
    
    # MoveIt Move Group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_config_template'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'use_fake_hardware': use_fake_hardware,
            'arm_params': arm_params,
        }.items()
    )
    
    # RViz with MoveIt plugin
    rviz_config = PathJoinSubstitution([
        FindPackageShare('moveit_config_template'),
        'rviz',
        'moveit.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': use_fake_hardware}
        ],
        condition=IfCondition(rviz),
        output='screen'
    )
    
    # Controller Manager (for real hardware)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[arm_params],
        condition=IfCondition(LaunchConfiguration('use_fake_hardware', default='false')),
        output='screen'
    )
    
    return LaunchDescription([
        declare_rviz_arg,
        declare_fake_hardware_arg,
        declare_arm_params_arg,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        move_group,
        rviz_node,
        controller_manager
    ]) 