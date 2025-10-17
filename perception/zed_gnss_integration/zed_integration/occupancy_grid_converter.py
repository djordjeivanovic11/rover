#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from typing import Optional


class OccupancyGridConverter(Node):
    """Convert fused point cloud to occupancy grid for Nav2"""
    
    def __init__(self):
        super().__init__('occupancy_grid_converter')
        
        # Parameters
        self.declare_parameter('input_cloud_topic', '/camera/mapping/fused_cloud')
        self.declare_parameter('output_grid_topic', '/map')
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('grid_width', 2048)
        self.declare_parameter('grid_height', 2048)
        self.declare_parameter('height_threshold', 0.3)
        self.declare_parameter('publish_rate', 1.0)
        
        # Get parameters
        input_topic = self.get_parameter('input_cloud_topic').value
        output_topic = self.get_parameter('output_grid_topic').value
        self.resolution = self.get_parameter('grid_resolution').value
        self.width = self.get_parameter('grid_width').value
        self.height = self.get_parameter('grid_height').value
        self.height_threshold = self.get_parameter('height_threshold').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # State
        self.latest_cloud: Optional[PointCloud2] = None
        
        # ROS2 setup
        self.cloud_sub = self.create_subscription(PointCloud2, input_topic, self.cb_point_cloud, 5)
        self.grid_pub = self.create_publisher(OccupancyGrid, output_topic, 1)
        
        # Processing timer
        self.create_timer(1.0 / publish_rate, self.process_and_publish)
        
        self.get_logger().info(f"🗺️ Occupancy Grid Converter initialized")
        self.get_logger().info(f"   Input: {input_topic}")
        self.get_logger().info(f"   Output: {output_topic}")
        self.get_logger().info(f"   Resolution: {self.resolution}m")
    
    def cb_point_cloud(self, msg: PointCloud2):
        """Store latest point cloud"""
        self.latest_cloud = msg
    
    def process_and_publish(self):
        """Convert point cloud to occupancy grid"""
        if not self.latest_cloud:
            return
        
        try:
            grid = self.convert_cloud_to_grid(self.latest_cloud)
            if grid:
                self.grid_pub.publish(grid)
                self.get_logger().debug("Published occupancy grid")
                
        except Exception as e:
            self.get_logger().error(f"Grid conversion failed: {e}")
    
    def convert_cloud_to_grid(self, cloud_msg: PointCloud2) -> Optional[OccupancyGrid]:
        """Convert point cloud to occupancy grid"""
        try:
            # Initialize grid
            grid = OccupancyGrid()
            grid.header = Header()
            grid.header.stamp = self.get_clock().now().to_msg()
            grid.header.frame_id = cloud_msg.header.frame_id
            
            # Grid metadata
            grid.info = MapMetaData()
            grid.info.resolution = self.resolution
            grid.info.width = self.width
            grid.info.height = self.height
            
            # Grid origin (centered)
            grid.info.origin = Pose()
            grid.info.origin.position.x = -self.width * self.resolution / 2.0
            grid.info.origin.position.y = -self.height * self.resolution / 2.0
            grid.info.origin.position.z = 0.0
            grid.info.origin.orientation.w = 1.0
            
            # Initialize grid data (unknown = -1)
            grid_data = np.full(self.width * self.height, -1, dtype=np.int8)
            
            # Process point cloud
            points = list(pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")))
            
            for x, y, z in points:
                # Convert world coordinates to grid indices
                grid_x = int((x - grid.info.origin.position.x) / self.resolution)
                grid_y = int((y - grid.info.origin.position.y) / self.resolution)
                
                # Check bounds
                if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                    grid_idx = grid_y * self.width + grid_x
                    
                    # Mark as occupied if above height threshold
                    if z > self.height_threshold:
                        grid_data[grid_idx] = 100  # Occupied
                    elif grid_data[grid_idx] == -1:  # Unknown
                        grid_data[grid_idx] = 0   # Free space
            
            grid.data = grid_data.tolist()
            return grid
            
        except Exception as e:
            self.get_logger().error(f"Grid conversion error: {e}")
            return None


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        converter = OccupancyGridConverter()
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
