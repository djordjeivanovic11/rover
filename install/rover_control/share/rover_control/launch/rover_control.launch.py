#!/usr/bin/env python3
"""
=============================================================================
ROVER CONTROL LAUNCH FILE
=============================================================================
Complete rover control system launch with proper node sequencing.
Starts safety monitor first, then hardware interface, controllers,
and finally high-level navigation components.

Launch with mock hardware for safe testing:
ros2 launch rover_control rover_control.launch.py mock_hardware:=true

Launch with real hardware:
ros2 launch rover_control rover_control.launch.py mock_hardware:=false
=============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate launch description for complete rover control system.
    """
    
    # =======================================================================
    # LAUNCH ARGUMENTS
    # =======================================================================
    
    # Mock hardware flag for safe testing
    mock_hardware_arg = DeclareLaunchArgument(
        'mock_hardware',
        default_value='true',
        description='Use mock hardware for testing (true/false)'
    )
    
    # Launch RViz for visualization
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz for visualization (true/false)'
    )
    
    # Mission mode
    mission_mode_arg = DeclareLaunchArgument(
        'mission_mode',
        default_value='exploration',
        description='Initial mission mode (exploration/science/return_to_base/manual)'
    )
    
    # Enable visual servoing
    visual_servoing_arg = DeclareLaunchArgument(
        'visual_servoing',
        default_value='false',
        description='Enable visual servoing for tennis ball approach (true/false)'
    )
    
    # Use GPS for odometry
    use_gps_arg = DeclareLaunchArgument(
        'use_gps',
        default_value='false',
        description='Use GPS for odometry correction (true/false)'
    )
    
    # =======================================================================
    # CONFIGURATION FILES
    # =======================================================================
    
    rover_control_share = FindPackageShare('rover_control')
    
    # Configuration file paths
    drive_params_file = PathJoinSubstitution([
        rover_control_share, 'config', 'drive_params.yaml'
    ])
    
    safety_params_file = PathJoinSubstitution([
        rover_control_share, 'config', 'safety_params.yaml'
    ])
    
    controller_config_file = PathJoinSubstitution([
        rover_control_share, 'config', 'controller_config.yaml'
    ])
    
    # =======================================================================
    # CORE NODES
    # =======================================================================
    
    # Safety Monitor - HIGHEST PRIORITY (starts immediately)
    safety_monitor_node = Node(
        package='rover_control',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[
            safety_params_file,
            {
                'use_sim_time': False,
                'mission_mode': LaunchConfiguration('mission_mode'),
            }
        ],
        remappings=[
            ('/rover/estop_sw', '/rover/estop_sw'),
            ('/rover/estop_hw', '/rover/estop_hw'),
            ('/rover/faults', '/rover/faults'),
            ('/rover/safety_status', '/rover/safety_status'),
        ]
    )
    
    # Hardware Interface - starts after 1 second delay
    hardware_interface_node = Node(
        package='rover_control',
        executable='hardware_interface',
        name='rover_hardware_interface',
        output='screen',
        parameters=[
            controller_config_file,
            {
                'use_sim_time': False,
                'mock_hardware': LaunchConfiguration('mock_hardware'),
                'update_rate': 50.0,
                'diagnostic_updater_rate': 10.0,
            }
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
            ('/joint_states', '/joint_states'),
            ('/rover/motor_currents', '/rover/motor_currents'),
            ('/rover/motor_temperatures', '/rover/motor_temperatures'),
            ('/rover/estop_hw', '/rover/estop_hw'),
        ]
    )
    
    # Controller Manager (ROS 2 Control)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            controller_config_file,
            {
                'use_sim_time': False,
            }
        ],
        remappings=[
            ('/dynamic_joint_states', '/dynamic_joint_states'),
            ('/joint_states', '/joint_states'),
        ]
    )
    
    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Differential Drive Controller
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='diff_drive_controller_spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # =======================================================================
    # HIGH-LEVEL NAVIGATION NODES
    # =======================================================================
    
    # Action Servers - high-level navigation actions
    action_servers_node = Node(
        package='rover_control',
        executable='action_servers',
        name='rover_action_servers',
        output='screen',
        parameters=[
            drive_params_file,
            safety_params_file,
            {
                'use_sim_time': False,
                'default_mission_mode': LaunchConfiguration('mission_mode'),
                'visual_servoing_enable': LaunchConfiguration('visual_servoing'),
                'max_autonomy_time': 60.0,
            }
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
            ('/rover/status', '/rover/status'),
            ('/rover/safety_ok', '/rover/safety_ok'),
            ('/rover/stuck_detected', '/rover/stuck_detected'),
            ('/rover/autonomy_time_remaining', '/rover/autonomy_time_remaining'),
        ]
    )
    
    # Path Follower - pure pursuit path following
    path_follower_node = Node(
        package='rover_control',
        executable='path_follower',
        name='rover_path_follower',
        output='screen',
        parameters=[
            drive_params_file,
            {
                'use_sim_time': False,
                'control_frequency': 10.0,
                'min_lookahead_distance': 0.5,
                'max_lookahead_distance': 3.0,
                'obstacle_avoidance_enable': True,
                'trajectory_smoothing_enable': True,
            }
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
            ('/scan', '/scan'),
            ('/rover/path', '/rover/path'),
            ('/rover/safety_ok', '/rover/safety_ok'),
            ('/rover/stuck_detected', '/rover/stuck_detected'),
            ('/rover/status', '/rover/status'),
        ]
    )
    
    # Odometry Publisher - sensor fusion and transforms
    odometry_publisher_node = Node(
        package='rover_control',
        executable='odometry_publisher',
        name='rover_odometry_publisher',
        output='screen',
        parameters=[
            drive_params_file,
            {
                'use_sim_time': False,
                'publish_frequency': 50.0,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'slip_detection_enable': True,
                'slip_compensation_enable': True,
                'encoder_weight': 0.7,
                'imu_weight': 0.2,
                'gps_weight': 0.1 if LaunchConfiguration('use_gps') else 0.0,
            }
        ],
        remappings=[
            ('/joint_states', '/joint_states'),
            ('/cmd_vel', '/cmd_vel'),
            ('/imu/data', '/imu/data'),
            ('/gps/fix', '/gps/fix'),
            ('/odom', '/odom'),
            ('/rover/slip_detected', '/rover/slip_detected'),
            ('/rover/slip_ratio', '/rover/slip_ratio'),
        ],
        condition=UnlessCondition(LaunchConfiguration('mock_hardware'))
    )
    
    # =======================================================================
    # OPTIONAL NODES
    # =======================================================================
    
    # RViz for visualization
    rviz_config_file = PathJoinSubstitution([
        rover_control_share, 'rviz', 'rover_control.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen'
    )
    
    # Robot State Publisher (for URDF visualization)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            # 'robot_description': Command(['xacro ', robot_description_file])
        }],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )
    
    # =======================================================================
    # LAUNCH SEQUENCE WITH PROPER TIMING
    # =======================================================================
    
    # Start safety monitor immediately (highest priority)
    safety_monitor_action = safety_monitor_node
    
    # Start hardware interface after 1 second
    hardware_interface_action = TimerAction(
        period=1.0,
        actions=[hardware_interface_node]
    )
    
    # Start controller manager after hardware interface is ready
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
    
    # Start controllers after controller manager is ready
    controllers_action = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        joint_state_broadcaster_spawner,
                        TimerAction(
                            period=1.0,
                            actions=[diff_drive_controller_spawner]
                        )
                    ]
                )
            ]
        )
    )
    
    # Start high-level nodes after controllers are ready
    high_level_nodes_action = RegisterEventHandler(
        OnProcessStart(
            target_action=diff_drive_controller_spawner,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        action_servers_node,
                        TimerAction(
                            period=1.0,
                            actions=[path_follower_node]
                        ),
                        TimerAction(
                            period=2.0,
                            actions=[odometry_publisher_node]
                        )
                    ]
                )
            ]
        )
    )
    
    # Start visualization nodes last
    visualization_action = RegisterEventHandler(
        OnProcessStart(
            target_action=action_servers_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[
                        robot_state_publisher_node,
                        TimerAction(
                            period=1.0,
                            actions=[rviz_node]
                        )
                    ]
                )
            ]
        )
    )
    
    # =======================================================================
    # EMERGENCY SHUTDOWN HANDLING
    # =======================================================================
    
    # If safety monitor exits, shutdown everything
    emergency_shutdown_action = RegisterEventHandler(
        OnProcessExit(
            target_action=safety_monitor_node,
            on_exit=[
                # Log emergency shutdown
                # Could add emergency shutdown logic here
            ]
        )
    )
    
    # =======================================================================
    # RETURN LAUNCH DESCRIPTION
    # =======================================================================
    
    return LaunchDescription([
        # Launch arguments
        mock_hardware_arg,
        launch_rviz_arg,
        mission_mode_arg,
        visual_servoing_arg,
        use_gps_arg,
        
        # Core system launch sequence
        safety_monitor_action,              # t=0s: Safety monitor (immediate)
        hardware_interface_action,          # t=1s: Hardware interface
        controller_manager_action,          # t=3s: Controller manager
        controllers_action,                 # t=5s: ROS 2 Control controllers
        high_level_nodes_action,           # t=7s: High-level navigation
        visualization_action,               # t=10s: Visualization
        
        # Emergency handling
        emergency_shutdown_action,
    ])


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def get_robot_description():
    """Get robot description from URDF file"""
    # This would load the rover URDF file
    # For now, return a simple placeholder
    return """
    <?xml version="1.0"?>
    <robot name="rover">
        <link name="base_link">
            <visual>
                <geometry>
                    <box size="0.6 0.4 0.2"/>
                </geometry>
            </visual>
        </link>
    </robot>
    """


# =============================================================================
# USAGE EXAMPLES
# =============================================================================

"""
# Launch with mock hardware (safe for testing):
ros2 launch rover_control rover_control.launch.py mock_hardware:=true

# Launch with real hardware:
ros2 launch rover_control rover_control.launch.py mock_hardware:=false

# Launch with RViz visualization:
ros2 launch rover_control rover_control.launch.py launch_rviz:=true

# Launch in science mission mode:
ros2 launch rover_control rover_control.launch.py mission_mode:=science

# Launch with visual servoing enabled:
ros2 launch rover_control rover_control.launch.py visual_servoing:=true

# Launch with GPS odometry correction:
ros2 launch rover_control rover_control.launch.py use_gps:=true

# Full competition configuration:
ros2 launch rover_control rover_control.launch.py \
    mock_hardware:=false \
    mission_mode:=exploration \
    visual_servoing:=true \
    use_gps:=true \
    launch_rviz:=false
""" 