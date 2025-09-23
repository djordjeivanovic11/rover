#!/usr/bin/env python3
"""
Launches the Stereolabs zed_wrapper node with parameters tuned for the ZED 2i.
==========================================================================

By default (with the YAML provided) the wrapper publishes:

┌───────────────────────────────────────────┬────────────────────────────┐
│ TOPIC                                     │ MESSAGE TYPE               │
├───────────────────────────────────────────┼────────────────────────────┤
│ /zed2i/left/image_rect_color              │ sensor_msgs/Image          │
│ /zed2i/left/camera_info                   │ sensor_msgs/CameraInfo     │
│ /zed2i/right/image_rect_color             │ sensor_msgs/Image          │
│ /zed2i/right/camera_info                  │ sensor_msgs/CameraInfo     │
│ /zed2i/depth/depth_registered             │ sensor_msgs/Image          │
│ /zed2i/depth/camera_info                  │ sensor_msgs/CameraInfo     │
│ /zed2i/confidence/confidence_map          │ sensor_msgs/Image          │
│ /zed2i/point_cloud/cloud_registered       │ sensor_msgs/PointCloud2    │
│ /zed2i/imu/data                           │ sensor_msgs/Imu            │
│ /zed2i/imu/data_raw                       │ sensor_msgs/Imu            │
│ /zed2i/imu/mag                            │ sensor_msgs/MagneticField  │
│ /zed2i/odom                               │ nav_msgs/Odometry          │
│ /zed2i/pose                               │ geometry_msgs/PoseStamped  │
│ /zed2i/pose_with_covariance               │ geometry_msgs/PoseWithCov… │
│ /zed2i/rgb/camera_info                    │ sensor_msgs/CameraInfo     │
│ /zed2i/temperature/imu                    │ sensor_msgs/Temperature    │
│ /zed2i/diagnostics                        │ diagnostic_msgs/Diagnostic │
│ (optional) /zed2i/objects/…               │ vision_msgs/* (if enabled) │
└───────────────────────────────────────────┴────────────────────────────┘

Key streams used by *loc_fusion*:
  • /zed2i/odom            — 6-DoF visual-inertial odometry
  • /zed2i/imu/data        — orientation + accel/gyro at ~200 Hz
  • /zed2i/point_cloud/cloud_registered — XYZRGBA point cloud for mapping

All topics share the TF frame tree rooted at:
  map (if ZED SLAM active) → odom (zed) → base_link → zed2i_camera_center …

You can disable or throttle individual outputs by editing params/zed2i.yaml.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('zed2i_launch'),
        'params', 'zed2i.yaml')

    zed_node = Node(
        package='zed_wrapper',
        executable='zed_camera',
        name='zed2i_camera',
        output='screen',
        parameters=[params_file])

    return LaunchDescription([zed_node])
