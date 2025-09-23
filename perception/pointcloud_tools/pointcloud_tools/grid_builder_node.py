#!/usr/bin/env python3

"""
Grid-Builder node – converts an accumulated 3-D point cloud into a rolling 2-D occupancy grid.

It listens to a dense global cloud (default `/rtabmap/cloud_map`) and uses TF to
transform every point into the chosen grid frame (default `map`).
The grid is a square window (e.g. 20 × 20 m) centred on the map origin, discretised at
`resolution` metres per cell; points higher than `z_thresh` are marked occupied,
points in the ground slice become free, and unseen cells stay unknown.
Because it rolls and updates with each cloud it acts as a high-refresh “local map”
that complements the slower, larger RTAB-Map occupancy grid.
Nav2’s local costmap, gap-guidance, or any custom reactive planner can subscribe to
`/local_map` (nav_msgs/OccupancyGrid) for fast, flattened obstacle information
without recomputing heavy 3-D projections themselves.
The node relies only on NumPy for rasterisation, so it stays lightweight even on a Jetson-class CPU.
"""


import rclpy, math, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
try:
    from tf2_geometry_msgs import do_transform_point
except ImportError:
    # Fallback for older ROS2 versions
    import tf2_py
    do_transform_point = tf2_py.do_transform_point
import geometry_msgs.msg as geo

class GridBuilder(Node):
    def __init__(self):
        super().__init__('grid_builder')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('frame_id', 'map'),
                ('resolution', 0.05),
                ('size_xy', 20.0),
                ('z_thresh', 0.15),
                ('range_max', 15.0),
                ('cloud_topic', '/rtabmap/cloud_map'),
                ('grid_topic', '/local_map')
            ])

        self._frame = self.get_parameter('frame_id').value
        self._res   = self.get_parameter('resolution').value
        self._cells = int(self.get_parameter('size_xy').value / self._res)
        self._z_th  = self.get_parameter('z_thresh').value
        self._rng   = self.get_parameter('range_max').value

        # Prepare static OccupancyGrid header/info
        self._grid_msg = OccupancyGrid()
        self._grid_msg.header.frame_id = self._frame
        self._grid_msg.info = MapMetaData()
        self._grid_msg.info.resolution = self._res
        self._grid_msg.info.width  = self._cells
        self._grid_msg.info.height = self._cells
        half = self._cells * self._res / 2.0
        self._grid_msg.info.origin.position.x = -half
        self._grid_msg.info.origin.position.y = -half
        self._grid_msg.info.origin.orientation.w = 1.0

        self._pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter('grid_topic').value, 1)

        self._tfbuf = tf2_ros.Buffer()
        self._tfl   = tf2_ros.TransformListener(self._tfbuf, self)

        self.create_subscription(PointCloud2,
                                 self.get_parameter('cloud_topic').value,
                                 self.cb_cloud, 1)

    # ------------------------------------------------------------
    def cb_cloud(self, cloud: PointCloud2):
        try:
            tf = self._tfbuf.lookup_transform(
                self._frame,
                cloud.header.frame_id,
                cloud.header.stamp, rclpy.duration.Duration(seconds=0.1))
        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException):
            self.get_logger().warning("TF unavailable")
            return

        grid = np.full((self._cells, self._cells), -1, dtype=np.int8)

        for x, y, z, *_ in pc2.read_points(cloud, skip_nans=True):
            # Transform point to map frame
            pt = geo.PointStamped()
            pt.header = cloud.header
            pt.point.x, pt.point.y, pt.point.z = x, y, z
            pt2 = do_transform_point(pt, tf).point

            if math.isnan(pt2.x) or math.isnan(pt2.y):
                continue
            if math.hypot(pt2.x, pt2.y) > self._rng:
                continue

            col = int((pt2.x + self._cells*self._res/2) / self._res)
            row = int((pt2.y + self._cells*self._res/2) / self._res)
            if 0 <= row < self._cells and 0 <= col < self._cells:
                if pt2.z > self._z_th:
                    grid[row, col] = 100        # occupied
                elif grid[row, col] == -1:
                    grid[row, col] = 0          # free

        self._grid_msg.header.stamp = cloud.header.stamp
        self._grid_msg.data = grid.flatten().tolist()
        self._pub.publish(self._grid_msg)

def main():
    rclpy.init()
    node = GridBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
