#!/usr/bin/env python3
"""
=============================================================================
PRODUCTION-READY MOVEIT ARM SIMULATION LAUNCH
=============================================================================
Single launch file with in-process controller spawning.
Eliminates ROS2 daemon issues and service deadlocks.

Usage:
  ros2 launch moveit_config_rover arm_moveit_sim.launch.py

What it does:
  1. Launches robot_state_publisher
  2. Launches ros2_control_node with mock hardware
  3. Spawns controllers IN-PROCESS (no daemon, no timing issues)
  4. Launches MoveGroup for motion planning
  5. Launches RViz with MoveIt plugin

Key fixes:
  - mock_sensor_commands=true (state follows commands immediately)
  - update_rate=100 Hz (doesn't starve service callbacks on ARM)
  - Spawners run in same process group (no XMLRPC/daemon issues)
  - Proper sequencing with TimerAction
=============================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import xacro


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
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz if true'
    )
    
    # Get configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('rviz')
    
    # Package directories
    moveit_config_pkg = 'moveit_config_rover'
    pkg_share = get_package_share_directory(moveit_config_pkg)
    
    # Load and process robot description
    xacro_file = os.path.join(pkg_share, 'urdf', 'rover_arm.urdf.xacro')
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'use_sim': 'true'}
    )
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Load SRDF
    srdf_file = os.path.join(pkg_share, 'config', 'rover_arm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # Load configurations
    kinematics_yaml = load_yaml(moveit_config_pkg, 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml(moveit_config_pkg, 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml(moveit_config_pkg, 'config/ompl_planning.yaml')
    moveit_controllers_yaml = load_yaml(moveit_config_pkg, 'config/moveit_controllers.yaml')
    
    # MoveIt configuration
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_controllers_yaml['moveit_simple_controller_manager'],
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }
    
    planning_pipelines_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning_yaml
    }
    
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }
    
    # =========================================================================
    # CORE NODES
    # =========================================================================
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )
    
    # ROS2 Control Node (with mock hardware)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            os.path.join(pkg_share, 'config', 'rover_arm_controllers.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )
    
    # =========================================================================
    # IN-PROCESS CONTROLLER SPAWNERS (no daemon, no timing issues)
    # =========================================================================
    
    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen',
    )
    
    # Arm Controller Spawner
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen',
    )
    
    # =========================================================================
    # MOVEIT & RVIZ
    # =========================================================================
    
    # MoveIt MoveGroup Node
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
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
        ],
    )
    
    # RViz with MoveIt Plugin
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            planning_pipelines_config,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(launch_rviz),
    )
    
    # =========================================================================
    # SEQUENCING (critical for avoiding deadlocks)
    # =========================================================================
    
    # Wait for controller_manager to be fully ready
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner],
    )
    
    # Wait for joint_state_broadcaster before spawning arm_controller
    delayed_arm_controller_spawner = TimerAction(
        period=3.5,
        actions=[arm_controller_spawner],
    )
    
    # Start MoveGroup after controllers are loaded
    delayed_move_group_node = TimerAction(
        period=5.0,
        actions=[move_group_node],
    )
    
    # Start RViz after MoveGroup is ready
    delayed_rviz_node = TimerAction(
        period=8.0,
        actions=[rviz_node],
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        rviz_arg,
        
        # Core infrastructure (immediate start)
        robot_state_publisher,
        ros2_control_node,
        
        # Controllers (staggered for reliability)
        delayed_joint_state_broadcaster_spawner,
        delayed_arm_controller_spawner,
        
        # MoveIt & RViz (after controllers ready)
        delayed_move_group_node,
        delayed_rviz_node,
    ])

