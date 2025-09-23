#!/usr/bin/env python3
"""
=============================================================================
ARM CONTROL LAUNCH FILE
=============================================================================
WARNING: DO NOT RUN UNTIL ALL PHASE 1 CODE IS MERGED AND TESTED!

Main launch file for URC rover arm control system.
Launches all components: hardware interface, trajectory executor, 
safety monitor, gripper controller, tool manager, and action servers.
=============================================================================
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.actions import RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for arm control system"""
    
    # WARNING MESSAGE
    warning_msg = """
    
    ================================================================
    WARNING: ARM CONTROL SYSTEM LAUNCH
    ================================================================
    
    This launch file starts the complete arm control system including:
    - Hardware interface (250 Hz control loop)
    - Safety monitor (<150ms fault response)
    - Trajectory executor with real-time monitoring
    - Gripper controller with force feedback
    - Tool manager with hot URDF/SRDF reloading
    - Action servers for autonomous operation
    
    IMPORTANT SAFETY NOTES:
    1. Ensure E-stop is functional before launching
    2. Start in MOCK MODE for testing (mock_mode: true in config)
    3. Verify all safety limits in safety_params.yaml
    4. Test individual components before full system launch
    
    DO NOT RUN WITH REAL HARDWARE UNTIL:
    - All components have been individually tested
    - Safety systems have been validated
    - Hardware interfaces have been verified
    - Emergency procedures are established
    
    ================================================================
    """
    
    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_mode",
            default_value="true",
            description="Start hardware interface in mock mode (safe for testing)"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Start nodes with debug logging"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true", 
            description="Launch RViz for visualization"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_monitoring",
            default_value="true",
            description="Enable safety monitoring (disable only for testing)"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_management",
            default_value="true",
            description="Enable tool management system"
        )
    )
    
    # Get launch configurations
    mock_mode = LaunchConfiguration("mock_mode")
    debug = LaunchConfiguration("debug")
    launch_rviz = LaunchConfiguration("launch_rviz")
    safety_monitoring = LaunchConfiguration("safety_monitoring")
    tool_management = LaunchConfiguration("tool_management")
    
    # Package paths
    arm_control_share = FindPackageShare("arm_control")
    arm_template_share = FindPackageShare("arm_template")
    
    # Config file paths
    arm_params_file = PathJoinSubstitution([
        arm_template_share, "config", "arm_params.yaml"
    ])
    controller_config_file = PathJoinSubstitution([
        arm_control_share, "config", "controller_config.yaml"
    ])
    safety_params_file = PathJoinSubstitution([
        arm_control_share, "config", "safety_params.yaml"
    ])
    
    # Log level based on debug flag
    log_level = "DEBUG"  # Always use DEBUG for now during development
    
    # =========================================================================
    # SAFETY MONITOR (First to start - highest priority)
    # =========================================================================
    safety_monitor_node = Node(
        package="arm_control",
        executable="safety_monitor",
        name="safety_monitor",
        parameters=[
            arm_params_file,
            safety_params_file,
            {"use_sim_time": False}
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(safety_monitoring)
    )
    
    # =========================================================================
    # HARDWARE INTERFACE (Second priority - low-level control)
    # =========================================================================
    hardware_interface_node = Node(
        package="arm_control",
        executable="hardware_interface",
        name="arm_hardware_interface",
        parameters=[
            arm_params_file,
            controller_config_file,
            {"mock_mode": mock_mode}
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level]
    )
    
    # =========================================================================
    # ROS 2 CONTROL MANAGER
    # =========================================================================
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[
            arm_params_file,
            controller_config_file
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level]
    )
    
    # =========================================================================
    # JOINT STATE BROADCASTER
    # =========================================================================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )
    
    # =========================================================================
    # JOINT TRAJECTORY CONTROLLER
    # =========================================================================
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen"
    )
    
    # =========================================================================
    # TRAJECTORY EXECUTOR (Third priority - motion coordination)
    # =========================================================================
    trajectory_executor_node = Node(
        package="arm_control",
        executable="trajectory_executor",
        name="trajectory_executor",
        parameters=[
            arm_params_file,
            safety_params_file,
            {"use_sim_time": False}
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level]
    )
    
    # =========================================================================
    # GRIPPER CONTROLLER
    # =========================================================================
    gripper_controller_node = Node(
        package="arm_control",
        executable="gripper_controller",
        name="gripper_controller",
        parameters=[
            arm_params_file,
            controller_config_file,
            {"use_sim_time": False}
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level]
    )
    
    # =========================================================================
    # TOOL MANAGER
    # =========================================================================
    tool_manager_node = Node(
        package="arm_control",
        executable="tool_manager",
        name="tool_manager",
        parameters=[
            arm_params_file,
            {"use_sim_time": False}
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(tool_management)
    )
    
    # =========================================================================
    # ACTION SERVERS (High-level mission interface)
    # =========================================================================
    action_servers_node = Node(
        package="arm_control",
        executable="action_servers",
        name="arm_action_servers",
        parameters=[
            arm_params_file,
            safety_params_file,
            {"use_sim_time": False}
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level]
    )
    
    # =========================================================================
    # ROBOT STATE PUBLISHER (for transforms)
    # =========================================================================
    robot_description_content = ""
    try:
        # Load URDF for robot state publisher
        urdf_path = os.path.join(
            get_package_share_directory("arm_template"),
            "urdf", "arm_template.urdf.xacro"
        )
        # TODO: Process xacro to get URDF content
        # For now, use placeholder
        robot_description_content = "<!-- URDF content will be loaded here -->"
    except:
        robot_description_content = "<!-- URDF not found -->"
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": False}
        ],
        output="screen"
    )
    
    # =========================================================================
    # RVIZ (Optional visualization)
    # =========================================================================
    rviz_config_file = PathJoinSubstitution([
        arm_control_share, "rviz", "arm_control.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        output="screen"
    )
    
    # =========================================================================
    # LAUNCH SEQUENCE WITH DELAYS
    # =========================================================================
    
    # Start safety monitor first (immediate)
    safety_monitor_action = safety_monitor_node
    
    # Start hardware interface after 1 second
    hardware_interface_action = TimerAction(
        period=1.0,
        actions=[hardware_interface_node]
    )
    
    # Start controller manager after hardware interface
    controller_manager_action = RegisterEventHandler(
        OnProcessStart(
            target_action=hardware_interface_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[controller_manager_node]
                )
            ]
        )
    )
    
    # Start controllers after controller manager
    controllers_action = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[
                        joint_state_broadcaster_spawner,
                        joint_trajectory_controller_spawner
                    ]
                )
            ]
        )
    )
    
    # Start high-level nodes after controllers
    high_level_nodes_action = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        trajectory_executor_node,
                        gripper_controller_node,
                        tool_manager_node,
                        action_servers_node,
                        robot_state_publisher_node
                    ]
                )
            ]
        )
    )
    
    # Start RViz last
    rviz_action = RegisterEventHandler(
        OnProcessStart(
            target_action=trajectory_executor_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[rviz_node]
                )
            ]
        )
    )
    
    # =========================================================================
    # LAUNCH DESCRIPTION
    # =========================================================================
    
    return LaunchDescription([
        # Warning message
        LogInfo(msg=warning_msg),
        
        # Launch arguments
        *declared_arguments,
        
        # Launch sequence
        safety_monitor_action,
        hardware_interface_action,
        controller_manager_action,
        controllers_action,
        high_level_nodes_action,
        rviz_action,
    ]) 