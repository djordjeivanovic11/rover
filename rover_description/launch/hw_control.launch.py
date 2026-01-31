from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rover_description"), "config", "rover.rviz"]
    )

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("rover_description"), "urdf", "rover.urdf.xacro"
        ]),
        " ",
        "use_gazebo:=false",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description},
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            PathJoinSubstitution([
                FindPackageShare("rover_description"),
                "config",
                "rover_controllers.yaml",
            ]),
        ],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rover_drive_controller'],
        output='screen',
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[
                joint_state_broadcaster,
                drive_controller,
            ],
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "--display-config", rviz_config_file,
            "--fixed-frame", "odom"
        ],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        spawn_controllers,
        rviz,
    ])