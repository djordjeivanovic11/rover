#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit



def _prepend_env(var, value):
    """Helper to prepend value to existing environment variable"""
    old = os.environ.get(var, '')
    return value if old == '' else value + os.pathsep + old


def generate_launch_description():
    # --------------------------------------------------------------------------
    # Paths
    # --------------------------------------------------------------------------
    pkg_path = get_package_share_directory('rover_description')
    model_root = os.path.join(pkg_path, 'models')
    urdf_file = os.path.join(pkg_path, 'urdf', 'rover.urdf')

    # --------------------------------------------------------------------------
    # Gazebo resource paths
    # --------------------------------------------------------------------------
    combined_gz_path = _prepend_env(
        'GZ_SIM_RESOURCE_PATH',
        model_root + os.pathsep + pkg_path
    )
    combined_ign_path = _prepend_env(
        'IGN_GAZEBO_RESOURCE_PATH',
        model_root + os.pathsep + pkg_path
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=combined_gz_path
    )

    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=combined_ign_path
    )

    # Needed on many systems for Gazebo GUI
    set_qt_platform = SetEnvironmentVariable(
        name='QT_QPA_PLATFORM',
        value='xcb'
    )

    # --------------------------------------------------------------------------
    # Launch arguments
    # --------------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        'ign_args',
        default_value='-r empty.sdf',
        description='Ignition Gazebo world file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # --------------------------------------------------------------------------
    # Robot description
    # --------------------------------------------------------------------------
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # --------------------------------------------------------------------------
    # Ignition Gazebo
    # --------------------------------------------------------------------------
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'ign_args': LaunchConfiguration('ign_args')
        }.items()
    )

    # --------------------------------------------------------------------------
    # Robot State Publisher
    # --------------------------------------------------------------------------
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

    # --------------------------------------------------------------------------
    # Spawn ground plane
    # --------------------------------------------------------------------------
    ground_plane_sdf = os.path.join(
        pkg_path, 'models', 'ground_plane', 'model.sdf'
    )

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

    # --------------------------------------------------------------------------
    # Spawn rover
    # --------------------------------------------------------------------------
    spawn_rover = Node(
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

    # --------------------------------------------------------------------------
    # ros2_control controller spawners
    # --------------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller'],
        output='screen',
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_rover,
            on_exit=[
                joint_state_broadcaster_spawner,
                diff_drive_spawner,
            ],
        )
    )

    # --------------------------------------------------------------------------
    # Clock bridge (still required)
    # --------------------------------------------------------------------------
    clock_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # --------------------------------------------------------------------------
    # Launch description
    # --------------------------------------------------------------------------
    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        set_qt_platform,

        world_arg,
        use_sim_time_arg,

        ignition_gazebo,
        robot_state_publisher_node,
        spawn_ground_plane,
        spawn_rover,

        # ros2_control
        spawn_controllers,

        # time
        clock_bridge,
    ])
