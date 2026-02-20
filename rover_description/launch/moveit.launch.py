from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("rover_moveit_config"), '/launch/move_group.launch.py'])
    )

    return LaunchDescription([
        move_group,
    ])