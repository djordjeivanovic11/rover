#!/usr/bin/env python3
"""
GapGuidanceNode:
  • subscribes PointCloud2, selects ROI in front of rover,
  • bins points into lateral stripes,
  • finds widest contiguous free gap,
  • publishes urc_msgs/GapCmd with steering + clear distance.
"""

import rclpy, math, numpy as np
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from urc_msgs.msg import GapCmd, GapDiag
from .filters import radius_outlier_filter
from .diagnostics import FPSCounter


class GapGuidanceNode(Node):
    def __init__(self):
        super().__init__('gap_guidance')

        # ---- parameters ----
        p = self.declare_parameters(
            '', [
                ('forward_dist',   3.0),
                ('lateral_half',   1.0),
                ('min_clear_z',    0.10),
                ('n_stripes',      21),
                ('density_thresh', 5),
                ('steer_gain',     1.0),
                ('alpha',          0.3),
                ('pointcloud_topic','/pointcloud_in'),
                ('cmd_topic',      '/gap_cmd'),
                ('diag_topic',     '/gap_diag')
            ])

        self.FWD   = p[0].value
        self.HALF  = p[1].value
        self.ZMIN  = p[2].value
        self.N     = int(p[3].value) | 1
        self.TH    = p[4].value
        self.K     = p[5].value
        self.ALPHA = p[6].value

        # publishers / subscribers
        self.pub_cmd  = self.create_publisher(GapCmd,  p[8].value, 1)
        self.pub_diag = self.create_publisher(GapDiag, p[9].value, 1)
        self.create_subscription(
            PointCloud2, p[7].value, self.pc_callback, 5)

        # internal
        self.prev_steer = 0.0
        self.fps = FPSCounter()

    def pc_callback(self, msg: PointCloud2):
        self.fps.tick()

        pts = np.array([ (x,y,z) for x,y,z,_ in pc2.read_points(
            msg, field_names=['x','y','z'], skip_nans=True)
            if 0.0 < x < self.FWD and abs(y) < self.HALF and z > self.ZMIN],
            dtype=np.float32)
        if pts.size == 0:
            return

        pts = radius_outlier_filter(pts, radius=0.25, min_neighbors=2)

        stripes = np.zeros(self.N, dtype=int)
        stripe_width = 2*self.HALF / self.N
        for x,y,z in pts:
            idx = int( (y + self.HALF) / stripe_width )
            idx = max(0, min(self.N-1, idx))
            stripes[idx] += 1

        # find contiguous free runs
        free = stripes < self.TH
        runs = []
        run_start = None
        for i, is_free in enumerate(free.tolist()+[False]):   # sentinel
            if is_free and run_start is None:
                run_start = i
            elif not is_free and run_start is not None:
                runs.append((run_start, i-1))
                run_start = None
        if not runs:
            self.get_logger().warn("No valid gap – publish stop")
            self.publish_cmd(0.0, 0.0)
            return

        # pick widest run, then centroid
        best = max(runs, key=lambda r: r[1]-r[0])
        centroid_idx = (best[0]+best[1])/2.0
        y_offset = (centroid_idx + 0.5)*stripe_width - self.HALF

        steer = self.K * y_offset   # rad
        steer = self.ALPHA*steer + (1-self.ALPHA)*self.prev_steer
        self.prev_steer = steer

        clear_dist = max(0.5,  self.FWD * (best[1]-best[0]+1)/self.N)

        self.publish_cmd(steer, clear_dist)
        self.publish_diag(stripes.tolist())

    def publish_cmd(self, steer, clear_dist):
        msg = GapCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.steer_angle  = float(steer)
        msg.clear_dist   = float(clear_dist)
        self.pub_cmd.publish(msg)

    def publish_diag(self, stripe_counts):
        if self.pub_diag.get_subscription_count() == 0:
            return
        d = GapDiag()
        d.header.stamp = self.get_clock().now().to_msg()
        d.stripe_counts = stripe_counts
        d.fps = float(self.fps.fps)
        self.pub_diag.publish(d)


def main(args=None):
    rclpy.init(args=args)
    node = GapGuidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
