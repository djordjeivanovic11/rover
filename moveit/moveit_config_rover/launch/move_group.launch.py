#!/usr/bin/env python3
"""
=============================================================================
MOVE GROUP LAUNCH FILE FOR ROVER ARM
=============================================================================
This launch file starts the MoveIt move_group node with all necessary
configurations for the rover arm. This is the core of the MoveIt system.
=============================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import xacro
import os
import yaml


def load_yaml(package_name, file_path):
    """Load a YAML file from a package"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        return None


def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    # Get package directories
    moveit_config_pkg = 'moveit_config_rover'
    
    # Load robot description
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory(moveit_config_pkg),
            'urdf',
            'rover_arm.urdf.xacro'
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Load SRDF
    robot_description_semantic_config = PathJoinSubstitution([
        FindPackageShare(moveit_config_pkg),
        'config',
        'rover_arm.srdf'
    ])
    
    with open(os.path.join(
        get_package_share_directory(moveit_config_pkg),
        'config',
        'rover_arm.srdf'
    ), 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # Load kinematics
    kinematics_yaml = load_yaml(moveit_config_pkg, 'config/kinematics.yaml')
    
    # Load joint limits
    joint_limits_yaml = load_yaml(moveit_config_pkg, 'config/joint_limits.yaml')
    
    # Load OMPL planning
    ompl_planning_yaml = load_yaml(moveit_config_pkg, 'config/ompl_planning.yaml')
    
    # Planning pipeline parameters
    planning_pipelines_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning_yaml
    }
    
    # Trajectory execution parameters
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    # Planning scene monitor parameters
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }
    
    # MoveGroup node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            planning_pipelines_config,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
        ],
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        move_group_node,
    ])

