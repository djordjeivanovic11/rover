#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import SetBool
from zed_integration.srv import NavigateToGPS
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math
from typing import Optional, Tuple
from enum import Enum


class LocalizationMode(Enum):
    """Localization operating modes"""
    LOCAL_ONLY = "local_only"
    GNSS_FUSION = "gnss_fusion"
    GNSS_LOST = "gnss_lost"
    GLOBAL_READY = "global_ready"


class GlobalLocalization(Node):
    """Global localization manager with camera-GNSS fusion"""
    
    def __init__(self):
        super().__init__('global_localization')
        
        # Parameters
        self.declare_parameter('gnss_topic', '/gps/fix')
        self.declare_parameter('fused_pose_topic', '/camera/gnss/pose')
        self.declare_parameter('geo_pose_topic', '/camera/geo_pose')
        self.declare_parameter('local_pose_topic', '/odometry/filtered')
        self.declare_parameter('gnss_timeout', 10.0)
        self.declare_parameter('fusion_quality_threshold', 0.8)
        self.declare_parameter('enable_auto_fallback', True)
        
        # Get parameters
        self.gnss_topic = self.get_parameter('gnss_topic').value
        self.fused_pose_topic = self.get_parameter('fused_pose_topic').value
        self.geo_pose_topic = self.get_parameter('geo_pose_topic').value
        self.local_pose_topic = self.get_parameter('local_pose_topic').value
        self.gnss_timeout = self.get_parameter('gnss_timeout').value
        self.fusion_threshold = self.get_parameter('fusion_quality_threshold').value
        self.auto_fallback = self.get_parameter('enable_auto_fallback').value
        
        # State tracking
        self.current_mode = LocalizationMode.LOCAL_ONLY
        self.last_gnss_time = 0.0
        self.fusion_quality = 0.0
        self.global_origin: Optional[GeoPoseStamped] = None
        self.current_global_pose: Optional[GeoPoseStamped] = None
        
        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.create_subscription(NavSatFix, self.gnss_topic, self.cb_gnss_fix, 10)
        self.create_subscription(PoseStamped, self.fused_pose_topic, self.cb_fused_pose, 10)
        self.create_subscription(GeoPoseStamped, self.geo_pose_topic, self.cb_geo_pose, 10)
        self.create_subscription(Odometry, self.local_pose_topic, self.cb_local_pose, 10)
        
        # Publishers
        self.mode_pub = self.create_publisher(String, '/localization/mode', 10)
        self.quality_pub = self.create_publisher(Float32, '/localization/quality', 10)
        self.global_ready_pub = self.create_publisher(Bool, '/localization/global_ready', 10)
        self.waypoint_pose_pub = self.create_publisher(PoseStamped, '/navigation/waypoint_target', 10)
        
        # Services
        self.create_service(SetBool, '/localization/enable_global', self.enable_global_localization)
        self.create_service(NavigateToGPS, '/navigation/goto_gps', self.navigate_to_gps)
        self.create_service(SetBool, '/localization/set_origin', self.set_global_origin)
        
        # Monitoring timer
        self.create_timer(1.0, self.monitor_localization_health)
        
        self.get_logger().info("🌍 Global Localization Manager initialized")
    
    def cb_gnss_fix(self, msg: NavSatFix):
        """Monitor GNSS fix quality"""
        self.last_gnss_time = self.get_clock().now().nanoseconds() / 1e9
        
        if msg.status.status >= 0:
            pos_cov = msg.position_covariance[0] + msg.position_covariance[4] + msg.position_covariance[8]
            self.fusion_quality = max(0.0, min(1.0, 1.0 / (1.0 + pos_cov)))
        else:
            self.fusion_quality = 0.0
    
    def cb_fused_pose(self, msg: PoseStamped):
        """Handle fused VIO+GNSS pose"""
        if self.current_mode in [LocalizationMode.GNSS_FUSION, LocalizationMode.GLOBAL_READY]:
            self.get_logger().debug("Fused pose received")
    
    def cb_geo_pose(self, msg: GeoPoseStamped):
        """Handle geographic pose (lat/lon/alt)"""
        self.current_global_pose = msg
        
        if self.global_origin is None:
            self.global_origin = msg
            self.get_logger().info(f"🌍 Global origin set: {msg.pose.position.latitude:.6f}, {msg.pose.position.longitude:.6f}")
    
    def cb_local_pose(self, msg: Odometry):
        """Handle local VIO-only pose"""
        if self.current_mode == LocalizationMode.LOCAL_ONLY:
            self.get_logger().debug("Local pose received")
    
    def monitor_localization_health(self):
        """Monitor and manage localization mode"""
        current_time = self.get_clock().now().nanoseconds() / 1e9
        gnss_age = current_time - self.last_gnss_time
        
        previous_mode = self.current_mode
        
        if gnss_age > self.gnss_timeout:
            if self.current_mode in [LocalizationMode.GNSS_FUSION, LocalizationMode.GLOBAL_READY]:
                self.current_mode = LocalizationMode.GNSS_LOST
        elif self.fusion_quality > self.fusion_threshold:
            if self.current_global_pose:
                self.current_mode = LocalizationMode.GLOBAL_READY
            else:
                self.current_mode = LocalizationMode.GNSS_FUSION
        elif self.fusion_quality > 0.0:
            self.current_mode = LocalizationMode.GNSS_FUSION
        else:
            self.current_mode = LocalizationMode.LOCAL_ONLY
        
        if self.current_mode != previous_mode:
            self.handle_mode_transition(previous_mode, self.current_mode)
        
        self.publish_localization_status()
    
    def handle_mode_transition(self, old_mode: LocalizationMode, new_mode: LocalizationMode):
        """Handle localization mode transitions"""
        self.get_logger().info(f"🔄 Localization mode: {old_mode.value} → {new_mode.value}")
        
        if new_mode == LocalizationMode.GLOBAL_READY:
            self.get_logger().info("🌍 Global localization ready - GPS waypoint navigation available")
        elif new_mode == LocalizationMode.GNSS_LOST:
            self.get_logger().warn("⚠️ GNSS signal lost - continuing with VIO")
        elif new_mode == LocalizationMode.LOCAL_ONLY:
            self.get_logger().warn("📍 Local localization only - no GPS waypoint navigation")
    
    def publish_localization_status(self):
        """Publish current localization status"""
        mode_msg = String()
        mode_msg.data = self.current_mode.value
        self.mode_pub.publish(mode_msg)
        
        quality_msg = Float32()
        quality_msg.data = self.fusion_quality
        self.quality_pub.publish(quality_msg)
        
        ready_msg = Bool()
        ready_msg.data = (self.current_mode == LocalizationMode.GLOBAL_READY)
        self.global_ready_pub.publish(ready_msg)
    
    def enable_global_localization(self, request, response):
        """Service to enable/disable global localization"""
        if request.data:
            response.success = True
            response.message = "Global localization enabled"
            self.get_logger().info("🌍 Global localization enabled via service")
        else:
            self.current_mode = LocalizationMode.LOCAL_ONLY
            response.success = True
            response.message = "Global localization disabled - using local only"
            self.get_logger().info("📍 Switched to local localization")
        
        return response
    
    def navigate_to_gps(self, request, response):
        """Service to navigate to GPS coordinates"""
        if self.current_mode != LocalizationMode.GLOBAL_READY:
            response.success = False
            response.message = f"Global localization not ready (mode: {self.current_mode.value})"
            return response
        
        try:
            target_pose = self.gps_to_map_pose(request.latitude, request.longitude, request.altitude)
            
            if target_pose:
                self.waypoint_pose_pub.publish(target_pose)
                
                response.success = True
                response.message = f"Waypoint set: {request.latitude:.6f}, {request.longitude:.6f}"
                response.target_pose = target_pose
                
                self.get_logger().info(f"🎯 GPS waypoint: ({request.latitude:.6f}, {request.longitude:.6f}) → Map({target_pose.pose.position.x:.2f}, {target_pose.pose.position.y:.2f})")
            else:
                response.success = False
                response.message = "Failed to convert GPS coordinates to map frame"
        
        except Exception as e:
            response.success = False
            response.message = f"GPS navigation failed: {str(e)}"
            self.get_logger().error(f"GPS navigation error: {e}")
        
        return response
    
    def set_global_origin(self, request, response):
        """Service to set current position as global origin"""
        if self.current_global_pose:
            self.global_origin = self.current_global_pose
            response.success = True
            response.message = f"Global origin set to current position: {self.global_origin.pose.position.latitude:.6f}, {self.global_origin.pose.position.longitude:.6f}"
            self.get_logger().info(f"🌍 {response.message}")
        else:
            response.success = False
            response.message = "No global pose available to set as origin"
        
        return response
    
    def gps_to_map_pose(self, lat: float, lon: float, alt: float) -> Optional[PoseStamped]:
        """Convert GPS coordinates to map frame pose"""
        if not self.global_origin:
            self.get_logger().error("No global origin set for coordinate conversion")
            return None
        
        try:
            origin_lat = self.global_origin.pose.position.latitude
            origin_lon = self.global_origin.pose.position.longitude
            origin_alt = self.global_origin.pose.position.altitude
            
            lat_diff = lat - origin_lat
            lon_diff = lon - origin_lon
            alt_diff = alt - origin_alt
            
            meters_per_degree_lat = 111000.0
            meters_per_degree_lon = 111000.0 * math.cos(math.radians(origin_lat))
            
            x_meters = lon_diff * meters_per_degree_lon
            y_meters = lat_diff * meters_per_degree_lat
            z_meters = alt_diff
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            
            pose_msg.pose.position.x = x_meters
            pose_msg.pose.position.y = y_meters
            pose_msg.pose.position.z = z_meters
            pose_msg.pose.orientation.w = 1.0
            
            return pose_msg
            
        except Exception as e:
            self.get_logger().error(f"GPS to map conversion failed: {e}")
            return None


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        localization = GlobalLocalization()
        rclpy.spin(localization)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
