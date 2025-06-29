#!/usr/bin/env python3

"""
Depth-to-Scan node – turns the ZED 2-i’s forward point cloud into a fake 2-D LaserScan.

The node subscribes to a registered XYZRGBA cloud (default `/zed2i/point_cloud/cloud_registered`)
and keeps only the points that lie inside a configurable ground-slice “window”
(`min_z` to `max_z`).
It bins those points into evenly spaced angular buckets (default 1°) across a
user-defined horizontal field-of-view (e.g. ±60°) and records the **closest** range per
bucket, mimicking the output of a planar LiDAR.
Everything closer than 0.2 m, further than `range_max`, or outside the slice is ignored,
so the resulting LaserScan is clean enough for Nav2’s obstacle layer.
The scan is published on `/scan` in the `base_link` frame, meaning any ROS
stack that expects a traditional laser sensor (RViz, DWB, AMCL) can consume it without modification.
"""

import rclpy, math, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2

class DepthToScan(Node):
    def __init__(self):
        super().__init__('depth_to_scan')

        p = self.declare_parameters(
            namespace='',
            parameters=[
                ('frame_id', 'base_link'),
                ('range_max', 12.0),
                ('fov_deg', 120.0),
                ('scan_height', 0.3),
                ('min_z', -0.1),
                ('max_z', 0.30),
                ('angular_res_deg', 1.0),
                ('pointcloud_topic', '/zed2i/point_cloud/cloud_registered'),
                ('scan_topic', '/scan')
            ])

        self._fov       = math.radians(self.get_parameter('fov_deg').value)
        self._angle_inc = math.radians(self.get_parameter('angular_res_deg').value)
        self._min_z     = self.get_parameter('min_z').value
        self._max_z     = self.get_parameter('max_z').value
        self._range_max = self.get_parameter('range_max').value
        self._frame_id  = self.get_parameter('frame_id').value

        self._nbins = int(self._fov / self._angle_inc)
        self._pub = self.create_publisher(LaserScan,
                                          self.get_parameter('scan_topic').value, 10)

        self.create_subscription(PointCloud2,
                                 self.get_parameter('pointcloud_topic').value,
                                 self.pc_callback, 10)

    def pc_callback(self, cloud: PointCloud2):
        ranges = [float('inf')] * self._nbins
        angle_min = -self._fov / 2.0

        for x, y, z, *_ in pc2.read_points(cloud, skip_nans=True):
            if not (self._min_z <= z <= self._max_z):
                continue
            r = math.hypot(x, y)
            if r > self._range_max or r < 0.2:
                continue
            theta = math.atan2(y, x)
            if abs(theta) > self._fov / 2:
                continue
            idx = int((theta - angle_min) / self._angle_inc)
            if 0 <= idx < self._nbins:
                ranges[idx] = min(ranges[idx], r)

        scan = LaserScan()
        scan.header.stamp = cloud.header.stamp
        scan.header.frame_id = self._frame_id
        scan.angle_min = angle_min
        scan.angle_max = -angle_min
        scan.angle_increment = self._angle_inc
        scan.time_increment = 0.0
        scan.range_min = 0.2
        scan.range_max = self._range_max
        scan.ranges = ranges
        self._pub.publish(scan)

def main():
    rclpy.init()
    node = DepthToScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
