#!/usr/bin/env python3
"""
=============================================================================
ROVER ODOMETRY PUBLISHER
=============================================================================
Publishes rover odometry information and transforms for navigation.
Integrates wheel encoder data, IMU, and GPS for robust pose estimation.
Includes slip detection and compensation for accurate odometry.
=============================================================================
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformBroadcaster

import numpy as np
import math
import time
from typing import Optional
from dataclasses import dataclass

from geometry_msgs.msg import TransformStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu, NavSatFix
from std_msgs.msg import Float32MultiArray, Bool, Float32
from rover_control.msg import RoverStatus


@dataclass
class OdometryConfig:
    """Odometry configuration parameters"""
    wheel_radius: float = 0.125         # Wheel radius (m)
    wheel_separation: float = 0.48      # Wheel separation (m)
    wheelbase: float = 0.52             # Wheelbase (m)
    gear_ratio: float = 20.0            # Motor to wheel gear ratio
    encoder_resolution: int = 2048      # Encoder counts per revolution
    
    # Covariance matrices
    pose_covariance: list = None
    twist_covariance: list = None
    
    # Slip detection and compensation
    slip_detection_enable: bool = True
    slip_threshold: float = 0.1         # Slip threshold for detection
    slip_compensation_enable: bool = True
    slip_compensation_factor: float = 0.9
    
    # Sensor fusion weights
    encoder_weight: float = 0.7         # Weight for encoder odometry
    imu_weight: float = 0.2             # Weight for IMU data
    gps_weight: float = 0.1             # Weight for GPS data


class RoverOdometryPublisher(Node):
    """
    Rover odometry publisher with multi-sensor fusion.
    
    Features:
    - Differential drive odometry from wheel encoders
    - IMU integration for improved orientation
    - GPS integration for absolute positioning
    - Slip detection and compensation
    - Transform publishing (odom → base_link)
    - Covariance estimation based on motion and sensors
    """
    
    def __init__(self):
        super().__init__("rover_odometry_publisher")
        
        # Load configuration
        self._load_configuration()
        
        # Initialize odometry state
        self.odom_pose_x = 0.0
        self.odom_pose_y = 0.0
        self.odom_pose_theta = 0.0
        
        self.odom_vel_linear = 0.0
        self.odom_vel_angular = 0.0
        
        # Previous encoder positions for integration
        self.prev_left_encoder = 0.0
        self.prev_right_encoder = 0.0
        self.prev_time = time.time()
        
        # Wheel velocities and slip detection
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        self.commanded_left_velocity = 0.0
        self.commanded_right_velocity = 0.0
        self.slip_detected = False
        self.slip_ratio = 0.0
        
        # IMU data
        self.imu_angular_velocity = 0.0
        self.imu_linear_acceleration = np.zeros(3)
        self.imu_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion [x, y, z, w]
        self.imu_available = False
        
        # GPS data
        self.gps_pose_x = 0.0
        self.gps_pose_y = 0.0
        self.gps_available = False
        self.gps_origin_set = False
        self.gps_origin_lat = 0.0
        self.gps_origin_lon = 0.0
        
        # Configuration
        self.config_obj = OdometryConfig()
        self._update_configuration()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.slip_status_pub = self.create_publisher(Bool, "/rover/slip_detected", 10)
        self.slip_ratio_pub = self.create_publisher(Float32, "/rover/slip_ratio", 10)
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, "/imu/data", self._imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self._gps_callback, 10)
        self.rover_status_sub = self.create_subscription(
            RoverStatus, "/rover/status", self._rover_status_callback, 10)
            
        # Odometry publishing timer
        self.odom_timer = self.create_timer(0.02, self._publish_odometry_callback)  # 50 Hz
        
        self.get_logger().info("Rover Odometry Publisher initialized")
        
    def _load_configuration(self):
        """Load configuration from parameters"""
        param_defaults = {
            'wheel_radius': 0.125,
            'wheel_separation': 0.48,
            'wheelbase': 0.52,
            'gear_ratio': 20.0,
            'encoder_resolution': 2048,
            'publish_frequency': 50.0,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'slip_detection_enable': True,
            'slip_threshold': 0.1,
            'slip_compensation_enable': True,
            'slip_compensation_factor': 0.9,
            'encoder_weight': 0.7,
            'imu_weight': 0.2,
            'gps_weight': 0.1,
            'pose_covariance_diagonal': [0.001, 0.001, 0.001, 0.001, 0.001, 0.01],
            'twist_covariance_diagonal': [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
        }
        
        self.config = {}
        for param_name, default_value in param_defaults.items():
            self.declare_parameter(param_name, default_value)
            self.config[param_name] = self.get_parameter(param_name).value
            
    def _update_configuration(self):
        """Update configuration object"""
        self.config_obj.wheel_radius = self.config['wheel_radius']
        self.config_obj.wheel_separation = self.config['wheel_separation']
        self.config_obj.wheelbase = self.config['wheelbase']
        self.config_obj.gear_ratio = self.config['gear_ratio']
        self.config_obj.encoder_resolution = self.config['encoder_resolution']
        
        self.config_obj.slip_detection_enable = self.config['slip_detection_enable']
        self.config_obj.slip_threshold = self.config['slip_threshold']
        self.config_obj.slip_compensation_enable = self.config['slip_compensation_enable']
        self.config_obj.slip_compensation_factor = self.config['slip_compensation_factor']
        
        self.config_obj.encoder_weight = self.config['encoder_weight']
        self.config_obj.imu_weight = self.config['imu_weight']
        self.config_obj.gps_weight = self.config['gps_weight']
        
        # Set up covariance matrices
        pose_diag = self.config['pose_covariance_diagonal']
        twist_diag = self.config['twist_covariance_diagonal']
        
        self.config_obj.pose_covariance = [
            pose_diag[0], 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pose_diag[1], 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, pose_diag[2], 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, pose_diag[3], 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, pose_diag[4], 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, pose_diag[5]
        ]
        
        self.config_obj.twist_covariance = [
            twist_diag[0], 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, twist_diag[1], 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, twist_diag[2], 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, twist_diag[3], 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, twist_diag[4], 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, twist_diag[5]
        ]
        
    def _publish_odometry_callback(self):
        """Main odometry publishing callback"""
        try:
            current_time = time.time()
            dt = current_time - self.prev_time
            
            if dt <= 0:
                return
                
            # Update odometry based on available sensors
            self._update_odometry(dt)
            
            # Publish odometry message
            self._publish_odometry_message(current_time)
            
            # Publish transform
            self._publish_transform(current_time)
            
            # Publish slip status
            self._publish_slip_status()
            
            self.prev_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"Odometry publishing error: {str(e)}")
            
    def _update_odometry(self, dt: float):
        """Update odometry state using sensor fusion"""
        # Primary odometry from wheel encoders
        encoder_linear_vel, encoder_angular_vel = self._calculate_encoder_odometry(dt)
        
        # Detect and compensate for slip
        if self.config_obj.slip_detection_enable:
            slip_factor = self._detect_slip(encoder_linear_vel, encoder_angular_vel)
            if self.config_obj.slip_compensation_enable and slip_factor > 0:
                encoder_linear_vel *= (1.0 - slip_factor * self.config_obj.slip_compensation_factor)
                
        # Sensor fusion
        fused_linear_vel = encoder_linear_vel * self.config_obj.encoder_weight
        fused_angular_vel = encoder_angular_vel * self.config_obj.encoder_weight
        
        # Add IMU contribution
        if self.imu_available:
            imu_angular_vel = self.imu_angular_velocity
            fused_angular_vel += imu_angular_vel * self.config_obj.imu_weight
            
        # Normalize weights
        total_weight = self.config_obj.encoder_weight
        if self.imu_available:
            total_weight += self.config_obj.imu_weight
            
        if total_weight > 0:
            fused_linear_vel /= total_weight
            fused_angular_vel /= total_weight
            
        # Update velocities
        self.odom_vel_linear = fused_linear_vel
        self.odom_vel_angular = fused_angular_vel
        
        # Integrate to get pose
        self._integrate_odometry(dt, fused_linear_vel, fused_angular_vel)
        
        # GPS correction (if available and enabled)
        if self.gps_available and self.config_obj.gps_weight > 0:
            self._apply_gps_correction()
            
    def _calculate_encoder_odometry(self, dt: float) -> tuple:
        """Calculate odometry from wheel encoders"""
        # Calculate wheel distances from encoder differences
        left_distance = (self.left_wheel_velocity * dt * 
                        self.config_obj.wheel_radius)
        right_distance = (self.right_wheel_velocity * dt * 
                         self.config_obj.wheel_radius)
        
        # Differential drive kinematics
        center_distance = (left_distance + right_distance) / 2.0
        angular_distance = (right_distance - left_distance) / self.config_obj.wheel_separation
        
        # Calculate velocities
        linear_velocity = center_distance / dt if dt > 0 else 0.0
        angular_velocity = angular_distance / dt if dt > 0 else 0.0
        
        return linear_velocity, angular_velocity
        
    def _detect_slip(self, measured_linear_vel: float, measured_angular_vel: float) -> float:
        """Detect wheel slip and return slip ratio"""
        # Calculate expected velocities from commands
        expected_linear = (self.commanded_left_velocity + self.commanded_right_velocity) / 2.0
        expected_linear *= self.config_obj.wheel_radius
        
        # Calculate slip ratio
        if abs(expected_linear) > 0.1:  # Only check when moving
            slip_ratio = abs(expected_linear - measured_linear_vel) / abs(expected_linear)
            self.slip_ratio = slip_ratio
            
            if slip_ratio > self.config_obj.slip_threshold:
                self.slip_detected = True
                return slip_ratio
            else:
                self.slip_detected = False
                return 0.0
        else:
            self.slip_detected = False
            self.slip_ratio = 0.0
            return 0.0
            
    def _integrate_odometry(self, dt: float, linear_vel: float, angular_vel: float):
        """Integrate velocity to get pose"""
        # Update orientation
        self.odom_pose_theta += angular_vel * dt
        
        # Normalize angle
        self.odom_pose_theta = self._normalize_angle(self.odom_pose_theta)
        
        # Update position (using midpoint integration for better accuracy)
        mid_theta = self.odom_pose_theta - (angular_vel * dt) / 2.0
        
        dx = linear_vel * math.cos(mid_theta) * dt
        dy = linear_vel * math.sin(mid_theta) * dt
        
        self.odom_pose_x += dx
        self.odom_pose_y += dy
        
    def _apply_gps_correction(self):
        """Apply GPS correction to odometry"""
        if not self.gps_origin_set:
            return
            
        # Simple GPS correction - blend GPS position with odometry
        gps_weight = self.config_obj.gps_weight
        odom_weight = 1.0 - gps_weight
        
        self.odom_pose_x = (self.odom_pose_x * odom_weight + 
                           self.gps_pose_x * gps_weight)
        self.odom_pose_y = (self.odom_pose_y * odom_weight + 
                           self.gps_pose_y * gps_weight)
                           
    def _publish_odometry_message(self, current_time: float):
        """Publish odometry message"""
        odom_msg = Odometry()
        
        # Header
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.config['odom_frame_id']
        odom_msg.child_frame_id = self.config['base_frame_id']
        
        # Pose
        odom_msg.pose.pose.position.x = self.odom_pose_x
        odom_msg.pose.pose.position.y = self.odom_pose_y
        odom_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        quat = self._yaw_to_quaternion(self.odom_pose_theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # Pose covariance
        odom_msg.pose.covariance = self.config_obj.pose_covariance
        
        # Twist
        odom_msg.twist.twist.linear.x = self.odom_vel_linear
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.odom_vel_angular
        
        # Twist covariance
        odom_msg.twist.covariance = self.config_obj.twist_covariance
        
        # Adjust covariance based on slip detection
        if self.slip_detected:
            # Increase uncertainty when slip is detected
            for i in range(len(odom_msg.pose.covariance)):
                odom_msg.pose.covariance[i] *= 2.0
            for i in range(len(odom_msg.twist.covariance)):
                odom_msg.twist.covariance[i] *= 2.0
                
        self.odom_pub.publish(odom_msg)
        
    def _publish_transform(self, current_time: float):
        """Publish odom → base_link transform"""
        transform = TransformStamped()
        
        # Header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.config['odom_frame_id']
        transform.child_frame_id = self.config['base_frame_id']
        
        # Translation
        transform.transform.translation.x = self.odom_pose_x
        transform.transform.translation.y = self.odom_pose_y
        transform.transform.translation.z = 0.0
        
        # Rotation
        quat = self._yaw_to_quaternion(self.odom_pose_theta)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(transform)
        
    def _publish_slip_status(self):
        """Publish slip detection status"""
        # Slip detected status
        slip_msg = Bool()
        slip_msg.data = self.slip_detected
        self.slip_status_pub.publish(slip_msg)
        
        # Slip ratio
        slip_ratio_msg = Float32()
        slip_ratio_msg.data = self.slip_ratio
        self.slip_ratio_pub.publish(slip_ratio_msg)
        
    def _yaw_to_quaternion(self, yaw: float) -> list:
        """Convert yaw angle to quaternion [x, y, z, w]"""
        half_yaw = yaw * 0.5
        return [0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)]
        
    def _quaternion_to_yaw(self, quaternion: list) -> float:
        """Convert quaternion [x, y, z, w] to yaw angle"""
        return 2.0 * math.atan2(quaternion[2], quaternion[3])
        
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def _lat_lon_to_xy(self, lat: float, lon: float) -> tuple:
        """Convert GPS lat/lon to local XY coordinates"""
        if not self.gps_origin_set:
            return 0.0, 0.0
            
        # Simple equirectangular projection
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.gps_origin_lat)
        origin_lon_rad = math.radians(self.gps_origin_lon)
        
        earth_radius = 6371000.0  # meters
        
        x = earth_radius * (lon_rad - origin_lon_rad) * math.cos(origin_lat_rad)
        y = earth_radius * (lat_rad - origin_lat_rad)
        
        return x, y
        
    # Callback functions
    def _joint_state_callback(self, msg: JointState):
        """Process joint state updates for wheel encoders"""
        # Extract wheel velocities from joint states
        wheel_names = ['left_front_wheel', 'left_rear_wheel', 
                      'right_front_wheel', 'right_rear_wheel']
        
        left_velocities = []
        right_velocities = []
        
        for i, name in enumerate(msg.name):
            if name in wheel_names:
                idx = msg.name.index(name)
                if len(msg.velocity) > idx:
                    if 'left' in name:
                        left_velocities.append(msg.velocity[idx])
                    elif 'right' in name:
                        right_velocities.append(msg.velocity[idx])
                        
        # Average left and right wheel velocities
        if left_velocities:
            self.left_wheel_velocity = sum(left_velocities) / len(left_velocities)
        if right_velocities:
            self.right_wheel_velocity = sum(right_velocities) / len(right_velocities)
            
    def _cmd_vel_callback(self, msg: Twist):
        """Process velocity commands for slip detection"""
        # Convert cmd_vel to wheel velocities for comparison
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        self.commanded_left_velocity = ((linear_vel - angular_vel * 
                                       self.config_obj.wheel_separation / 2.0) / 
                                      self.config_obj.wheel_radius)
        self.commanded_right_velocity = ((linear_vel + angular_vel * 
                                        self.config_obj.wheel_separation / 2.0) / 
                                       self.config_obj.wheel_radius)
                                       
    def _imu_callback(self, msg: Imu):
        """Process IMU data"""
        self.imu_angular_velocity = msg.angular_velocity.z
        self.imu_linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        self.imu_orientation = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        self.imu_available = True
        
    def _gps_callback(self, msg: NavSatFix):
        """Process GPS data"""
        if msg.status.status >= 0:  # Valid GPS fix
            if not self.gps_origin_set:
                # Set GPS origin on first valid fix
                self.gps_origin_lat = msg.latitude
                self.gps_origin_lon = msg.longitude
                self.gps_origin_set = True
                self.get_logger().info(f"GPS origin set: {self.gps_origin_lat:.6f}, {self.gps_origin_lon:.6f}")
            else:
                # Convert GPS to local coordinates
                self.gps_pose_x, self.gps_pose_y = self._lat_lon_to_xy(
                    msg.latitude, msg.longitude)
                self.gps_available = True
                
    def _rover_status_callback(self, msg: RoverStatus):
        """Process rover status updates"""
        # Could adjust odometry parameters based on mission mode
        pass
        
    # Public interface methods
    def reset_odometry(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Reset odometry to specified pose"""
        self.odom_pose_x = x
        self.odom_pose_y = y
        self.odom_pose_theta = theta
        self.get_logger().info(f"Odometry reset to: ({x:.3f}, {y:.3f}, {theta:.3f})")
        
    def get_current_pose(self) -> tuple:
        """Get current odometry pose"""
        return self.odom_pose_x, self.odom_pose_y, self.odom_pose_theta
        
    def get_current_velocity(self) -> tuple:
        """Get current odometry velocity"""
        return self.odom_vel_linear, self.odom_vel_angular
        
    def is_slip_detected(self) -> bool:
        """Check if wheel slip is currently detected"""
        return self.slip_detected
        
    def get_slip_ratio(self) -> float:
        """Get current slip ratio"""
        return self.slip_ratio


def main(args=None):
    """Main entry point for the odometry publisher node"""
    rclpy.init(args=args)
    
    try:
        node = RoverOdometryPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in odometry publisher: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 