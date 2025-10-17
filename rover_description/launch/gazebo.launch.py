#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def _prepend_env(var, value):
    """Helper to prepend value to existing environment variable"""
    old = os.environ.get(var, '')
    return value if old == '' else value + os.pathsep + old

def generate_launch_description():
    # Get the package directory
    pkg_path = get_package_share_directory('rover_description')
    model_root = os.path.join(pkg_path, 'models')  # Contains rover_description/ model
    
    # Combine both paths for Gazebo resource resolution
    # Includes both model directory and package root
    combined_gz_path = _prepend_env('GZ_SIM_RESOURCE_PATH', model_root + os.pathsep + pkg_path)
    combined_ign_path = _prepend_env('IGN_GAZEBO_RESOURCE_PATH', model_root + os.pathsep + pkg_path)
    
    # Set environment variables for Gazebo to find model resources
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=combined_gz_path
    )
    
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=combined_ign_path
    )
    
    # Graphics configuration for Gazebo GUI
    set_qt_platform = SetEnvironmentVariable(
        name='QT_QPA_PLATFORM',
        value='xcb'
    )
    
    # Set the path to the URDF file
    urdf_file = os.path.join(pkg_path, 'urdf', 'rover.urdf')
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'ign_args',
        default_value='',
        description='Arguments for Ignition Gazebo'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Ignition Gazebo launch
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'ign_args': LaunchConfiguration('ign_args')
        }.items()
    )
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Spawn ground plane in Ignition Gazebo
    ground_plane_sdf = os.path.join(pkg_path, 'models', 'ground_plane', 'model.sdf')
    spawn_ground_plane = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_ground_plane',
        output='screen',
        arguments=[
            '-file', ground_plane_sdf,
            '-name', 'ground_plane',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Spawn robot in Ignition Gazebo
    spawn_entity_node = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_rover',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rover',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '2.0'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Bridge for joint states
    joint_state_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        arguments=[
            '/world/default/model/rover/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    # Bridge for clock
    clock_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # Set resource paths first
        set_gz_resource_path,
        set_ign_resource_path,
        # Set Graphics environment
        set_qt_platform,
        # Launch arguments
        world_arg,
        use_sim_time_arg,
        # Launch Gazebo and nodes
        ignition_gazebo,
        robot_state_publisher_node,
        spawn_ground_plane,
        spawn_entity_node,
        joint_state_bridge,
        clock_bridge
    ])
