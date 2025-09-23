#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_path = get_package_share_directory('rover_description')
    
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
            '-z', '0.5'
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
        world_arg,
        use_sim_time_arg,
        ignition_gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        joint_state_bridge,
        clock_bridge
    ])
