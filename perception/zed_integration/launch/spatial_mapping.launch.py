import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Launch GPU-accelerated spatial mapping"""

    # Package directories
    pkg_share = get_package_share_directory('zed_integration')
    zed_wrapper_share = get_package_share_directory('zed_wrapper')

    # Configuration files
    mapping_config = os.path.join(pkg_share, 'config', 'spatial_mapping.yaml')
    zed_config = os.path.join(zed_wrapper_share, 'config', 'zed2i.yaml')

    # Launch arguments
    args = [
        DeclareLaunchArgument(
            'camera_model', default_value='zed2i',
            description='Camera model for spatial mapping'
        ),
        DeclareLaunchArgument(
            'mapping_resolution', default_value='0.05',
            description='Mapping resolution in meters'
        ),
        DeclareLaunchArgument(
            'mapping_range', default_value='10.0',
            description='Maximum mapping range in meters'
        ),
        DeclareLaunchArgument(
            'enable_plane_detection', default_value='true',
            description='Enable automatic plane detection'
        ),
        DeclareLaunchArgument(
            'publish_occupancy_grid', default_value='true',
            description='Publish occupancy grid for Nav2 integration'
        )
    ]

    # Camera with spatial mapping enabled
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(zed_wrapper_share, 'launch', 'zed_camera.launch.py')
        ]),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'config_common_path': zed_config,
            # Enable spatial mapping
            'mapping.mapping_enabled': 'true',
            'mapping.resolution': LaunchConfiguration('mapping_resolution'),
            'mapping.max_mapping_range': LaunchConfiguration('mapping_range'),
            'mapping.fused_pointcloud_freq': '2.0',
            # Plane detection
            'mapping.pd_max_distance_threshold': '0.15',
            'mapping.pd_normal_similarity_threshold': '15.0',
        }.items()
    )

    # Spatial mapping manager
    spatial_mapping_node = Node(
        package='zed_integration',
        executable='spatial_mapping',
        name='spatial_mapping',
        output='screen',
        parameters=[
            mapping_config,
            {
                'mapping_resolution': LaunchConfiguration('mapping_resolution'),
                'mapping_range': LaunchConfiguration('mapping_range'),
                'enable_performance_monitoring': True
            }
        ]
    )

    # Occupancy grid converter (if enabled)
    grid_converter_node = Node(
        package='zed_integration',
        executable='occupancy_grid_converter',
        name='occupancy_grid_converter',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_occupancy_grid')),
        parameters=[{
            'input_cloud_topic': '/camera/mapping/fused_cloud',
            'output_grid_topic': '/map',
            'grid_resolution': LaunchConfiguration('mapping_resolution')
        }]
    )

    return LaunchDescription([
        *args,
        camera_launch,
        spatial_mapping_node,
        grid_converter_node
    ])
