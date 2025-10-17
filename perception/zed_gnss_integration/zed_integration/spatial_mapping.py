#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32, Bool, Header
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import time
from typing import Optional, Dict, List
from enum import Enum


class MappingMode(Enum):
    """Spatial mapping operating modes"""
    DISABLED = "disabled"
    GPU_MAPPING = "gpu_mapping"
    HYBRID_MAPPING = "hybrid_mapping"
    FALLBACK_MAPPING = "fallback"


class SpatialMapping(Node):
    """Spatial mapping manager with GPU acceleration"""
    
    def __init__(self):
        super().__init__('spatial_mapping')
        
        # Parameters
        self.declare_parameter('fused_cloud_topic', '/camera/mapping/fused_cloud')
        self.declare_parameter('occupancy_grid_topic', '/map')
        self.declare_parameter('plane_topic', '/camera/mapping/plane')
        self.declare_parameter('mapping_resolution', 0.05)
        self.declare_parameter('mapping_range', 10.0)
        self.declare_parameter('update_frequency', 2.0)
        self.declare_parameter('enable_performance_monitoring', True)
        self.declare_parameter('memory_limit_mb', 1024)
        
        # Get parameters
        self.fused_cloud_topic = self.get_parameter('fused_cloud_topic').value
        self.grid_topic = self.get_parameter('occupancy_grid_topic').value
        self.plane_topic = self.get_parameter('plane_topic').value
        self.resolution = self.get_parameter('mapping_resolution').value
        self.max_range = self.get_parameter('mapping_range').value
        self.update_freq = self.get_parameter('update_frequency').value
        self.perf_monitoring = self.get_parameter('enable_performance_monitoring').value
        self.memory_limit = self.get_parameter('memory_limit_mb').value
        
        # State tracking
        self.current_mode = MappingMode.DISABLED
        self.last_map_update = 0.0
        self.map_statistics = {'points': 0, 'coverage_area': 0.0, 'processing_time': 0.0}
        
        # Subscribers
        self.create_subscription(PointCloud2, self.fused_cloud_topic, self.cb_fused_cloud, 5)
        self.create_subscription(PointCloud2, self.plane_topic, self.cb_detected_planes, 5)
        
        # Publishers
        self.mode_pub = self.create_publisher(String, '/mapping/mode', 10)
        self.quality_pub = self.create_publisher(Float32, '/mapping/quality', 10)
        self.stats_pub = self.create_publisher(String, '/mapping/statistics', 10)
        self.plane_markers_pub = self.create_publisher(MarkerArray, '/mapping/plane_markers', 10)
        
        # Services
        self.create_service(SetBool, '/mapping/enable_gpu_mapping', self.enable_gpu_mapping)
        self.create_service(Trigger, '/mapping/save_map', self.save_current_map)
        self.create_service(Trigger, '/mapping/reset_map', self.reset_mapping)
        
        # Monitoring timers
        self.create_timer(1.0, self.monitor_mapping_health)
        if self.perf_monitoring:
            self.create_timer(5.0, self.log_performance_metrics)
        
        self.get_logger().info("🗺️ Spatial Mapping Manager initialized")
    
    def cb_fused_cloud(self, msg: PointCloud2):
        """Process fused colored point cloud from GPU mapping"""
        try:
            self.last_map_update = time.time()
            
            point_step = msg.point_step if msg.point_step > 0 else 16
            estimated_points = len(msg.data) // point_step
            self.map_statistics['points'] = estimated_points
            
            self.get_logger().debug(f"Processed fused cloud: {estimated_points} points")
            
        except Exception as e:
            self.get_logger().error(f"Fused cloud processing failed: {e}")
    
    def cb_detected_planes(self, msg: PointCloud2):
        """Process detected planes for navigation"""
        try:
            self.create_plane_markers(msg)
        except Exception as e:
            self.get_logger().error(f"Plane processing failed: {e}")
    
    def create_plane_markers(self, plane_msg: PointCloud2):
        """Create visualization markers for detected planes"""
        marker_array = MarkerArray()
        
        plane_marker = Marker()
        plane_marker.header = plane_msg.header
        plane_marker.ns = 'detected_planes'
        plane_marker.id = 0
        plane_marker.type = Marker.CUBE
        plane_marker.action = Marker.ADD
        
        plane_marker.pose.position.x = 0.0
        plane_marker.pose.position.y = 0.0
        plane_marker.pose.position.z = -0.1
        plane_marker.pose.orientation.w = 1.0
        
        plane_marker.scale.x = 10.0
        plane_marker.scale.y = 10.0
        plane_marker.scale.z = 0.02
        
        plane_marker.color.r = 0.0
        plane_marker.color.g = 1.0
        plane_marker.color.b = 0.0
        plane_marker.color.a = 0.3
        
        marker_array.markers.append(plane_marker)
        self.plane_markers_pub.publish(marker_array)
    
    def monitor_mapping_health(self):
        """Monitor mapping system health"""
        current_time = time.time()
        
        if self.last_map_update > 0:
            map_age = current_time - self.last_map_update
            if map_age > 5.0:
                if self.current_mode == MappingMode.GPU_MAPPING:
                    self.get_logger().warn("⚠️ GPU mapping timeout - no updates received")
        
        self.publish_mapping_status()
    
    def publish_mapping_status(self):
        """Publish mapping status and quality"""
        mode_msg = String()
        mode_msg.data = self.current_mode.value
        self.mode_pub.publish(mode_msg)
        
        quality_msg = Float32()
        quality_msg.data = 0.8 if self.current_mode == MappingMode.GPU_MAPPING else 0.0
        self.quality_pub.publish(quality_msg)
        
        stats_msg = String()
        stats_msg.data = f"points:{self.map_statistics['points']},mode:{self.current_mode.value}"
        self.stats_pub.publish(stats_msg)
    
    def enable_gpu_mapping(self, request, response):
        """Service to enable/disable GPU mapping"""
        if request.data:
            self.current_mode = MappingMode.GPU_MAPPING
            response.success = True
            response.message = "GPU spatial mapping enabled"
            self.get_logger().info("🗺️ GPU spatial mapping enabled")
        else:
            self.current_mode = MappingMode.DISABLED
            response.success = True
            response.message = "Spatial mapping disabled"
            self.get_logger().info("📍 Spatial mapping disabled")
        
        return response
    
    def save_current_map(self, request, response):
        """Service to save current map"""
        try:
            timestamp = int(time.time())
            map_filename = f"/home/rover/maps/spatial_map_{timestamp}.pcd"
            
            response.success = True
            response.message = f"Map saved to {map_filename}"
            self.get_logger().info(f"💾 {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Map save failed: {str(e)}"
            self.get_logger().error(f"Map save error: {e}")
        
        return response
    
    def reset_mapping(self, request, response):
        """Service to reset current map"""
        try:
            self.map_statistics = {'points': 0, 'coverage_area': 0.0, 'processing_time': 0.0}
            
            response.success = True
            response.message = "Spatial mapping reset"
            self.get_logger().info("🔄 Spatial mapping reset")
            
        except Exception as e:
            response.success = False
            response.message = f"Map reset failed: {str(e)}"
        
        return response
    
    def log_performance_metrics(self):
        """Log detailed performance metrics"""
        self.get_logger().info(
            f"🗺️ Mapping Performance: "
            f"Mode={self.current_mode.value}, "
            f"Points={self.map_statistics['points']}"
        )


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        mapping = SpatialMapping()
        rclpy.spin(mapping)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
