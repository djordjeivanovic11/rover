#!/usr/bin/env python3
"""
=============================================================================
ROVER PATH FOLLOWER
=============================================================================
Pure pursuit path following implementation for URC rover navigation.
Provides smooth trajectory execution with obstacle avoidance, slip
compensation, and mission-specific parameter adaptation.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import math
import time
from typing import List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

from geometry_msgs.msg import Twist, PoseStamped, Point, PointStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from rover_control.msg import RoverStatus, DriveCommand


class PathFollowerState(Enum):
    """Path follower states"""
    IDLE = "IDLE"
    FOLLOWING = "FOLLOWING"
    AVOIDING_OBSTACLE = "AVOIDING_OBSTACLE"
    STUCK_RECOVERY = "STUCK_RECOVERY"
    GOAL_REACHED = "GOAL_REACHED"
    FAILED = "FAILED"


@dataclass
class PurePursuitConfig:
    """Pure pursuit configuration parameters"""
    min_lookahead: float = 0.5          # Minimum lookahead distance (m)
    max_lookahead: float = 3.0          # Maximum lookahead distance (m)
    lookahead_ratio: float = 0.3        # Lookahead = velocity * ratio + min_lookahead
    velocity_kp: float = 1.0            # Velocity proportional gain
    angular_kp: float = 2.0             # Angular velocity proportional gain
    goal_tolerance: float = 0.1         # Goal reaching tolerance (m)
    path_tolerance: float = 0.5         # Maximum deviation from path (m)
    wheelbase: float = 0.52             # Wheelbase for bicycle model (m)


@dataclass
class ObstacleAvoidanceConfig:
    """Obstacle avoidance configuration"""
    enable: bool = True                 # Enable obstacle avoidance
    safety_margin: float = 0.3          # Safety margin around obstacles (m)
    avoidance_distance: float = 2.0     # Distance to start avoidance (m)
    recovery_distance: float = 0.5      # Distance to clear before resuming path
    velocity_scaling: float = 0.7       # Velocity scaling during avoidance


class RoverPathFollower(Node):
    """
    Pure pursuit path follower for rover navigation.
    
    Features:
    - Adaptive lookahead distance based on velocity
    - Obstacle avoidance integration
    - Slip compensation
    - Mission mode parameter adaptation
    - Trajectory smoothing
    - Progress tracking and feedback
    """
    
    def __init__(self):
        super().__init__("rover_path_follower")
        
        # Load configuration
        self._load_configuration()
        
        # Path following state
        self.state = PathFollowerState.IDLE
        self.current_path = None
        self.current_pose = PoseStamped()
        self.current_velocity = Twist()
        self.target_velocity = Twist()
        
        # Path tracking
        self.closest_point_index = 0
        self.lookahead_point = None
        self.path_progress = 0.0
        self.distance_to_goal = 0.0
        self.cross_track_error = 0.0
        
        # Mission parameters
        self.mission_mode = "exploration"
        self.max_velocity = 1.8
        self.max_acceleration = 1.2
        self.current_lookahead_distance = self.pure_pursuit_config.min_lookahead
        
        # Safety and monitoring
        self.safety_ok = True
        self.stuck_detected = False
        self.obstacle_detected = False
        self.obstacle_avoidance_active = False
        
        # Performance metrics
        self.path_start_time = None
        self.distance_traveled = 0.0
        self.max_cross_track_error = 0.0
        self.average_velocity = 0.0
        
        # Pure pursuit configuration
        self.pure_pursuit_config = PurePursuitConfig()
        self.obstacle_config = ObstacleAvoidanceConfig()
        
        # Trajectory smoothing
        self.velocity_filter_alpha = 0.8
        self.filtered_velocity = Twist()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.drive_command_pub = self.create_publisher(DriveCommand, "/rover/drive_cmd", 10)
        self.lookahead_pub = self.create_publisher(PointStamped, "/rover/lookahead_point", 10)
        self.path_progress_pub = self.create_publisher(Float32, "/rover/path_progress", 10)
        self.cross_track_error_pub = self.create_publisher(Float32, "/rover/cross_track_error", 10)
        self.state_pub = self.create_publisher(String, "/rover/path_follower_state", 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_callback, 10)
        self.path_sub = self.create_subscription(
            Path, "/rover/path", self._path_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, "/scan", self._laser_callback, 10)
        self.safety_sub = self.create_subscription(
            Bool, "/rover/safety_ok", self._safety_callback, 10)
        self.stuck_sub = self.create_subscription(
            Bool, "/rover/stuck_detected", self._stuck_callback, 10)
        self.rover_status_sub = self.create_subscription(
            RoverStatus, "/rover/status", self._rover_status_callback, 10)
            
        # Control loop timer
        self.control_timer = self.create_timer(0.1, self._control_loop_callback)  # 10 Hz
        
        # Status publishing timer
        self.status_timer = self.create_timer(0.5, self._publish_status_callback)  # 2 Hz
        
        self.get_logger().info("Rover Path Follower initialized")
        
    def _load_configuration(self):
        """Load configuration from parameters"""
        param_defaults = {
            'control_frequency': 10.0,
            'min_lookahead_distance': 0.5,
            'max_lookahead_distance': 3.0,
            'lookahead_ratio': 0.3,
            'velocity_kp': 1.0,
            'angular_kp': 2.0,
            'goal_tolerance': 0.1,
            'path_tolerance': 0.5,
            'wheelbase': 0.52,
            'obstacle_avoidance_enable': True,
            'safety_margin': 0.3,
            'avoidance_distance': 2.0,
            'velocity_filter_alpha': 0.8,
            'trajectory_smoothing_enable': True
        }
        
        self.config = {}
        for param_name, default_value in param_defaults.items():
            self.declare_parameter(param_name, default_value)
            self.config[param_name] = self.get_parameter(param_name).value
            
        # Update configuration objects
        self.pure_pursuit_config.min_lookahead = self.config['min_lookahead_distance']
        self.pure_pursuit_config.max_lookahead = self.config['max_lookahead_distance']
        self.pure_pursuit_config.lookahead_ratio = self.config['lookahead_ratio']
        self.pure_pursuit_config.velocity_kp = self.config['velocity_kp']
        self.pure_pursuit_config.angular_kp = self.config['angular_kp']
        self.pure_pursuit_config.goal_tolerance = self.config['goal_tolerance']
        self.pure_pursuit_config.path_tolerance = self.config['path_tolerance']
        self.pure_pursuit_config.wheelbase = self.config['wheelbase']
        
        self.obstacle_config.enable = self.config['obstacle_avoidance_enable']
        self.obstacle_config.safety_margin = self.config['safety_margin']
        self.obstacle_config.avoidance_distance = self.config['avoidance_distance']
        
        self.velocity_filter_alpha = self.config['velocity_filter_alpha']
        
    def _control_loop_callback(self):
        """Main control loop for path following"""
        if self.state == PathFollowerState.IDLE:
            return
            
        try:
            # Check safety conditions
            if not self.safety_ok:
                self._handle_safety_stop()
                return
                
            # Update path tracking
            self._update_path_tracking()
            
            # State machine
            if self.state == PathFollowerState.FOLLOWING:
                self._execute_path_following()
            elif self.state == PathFollowerState.AVOIDING_OBSTACLE:
                self._execute_obstacle_avoidance()
            elif self.state == PathFollowerState.STUCK_RECOVERY:
                self._execute_stuck_recovery()
                
            # Publish control commands
            self._publish_control_commands()
            
        except Exception as e:
            self.get_logger().error(f"Control loop error: {str(e)}")
            self._stop_rover()
            
    def _update_path_tracking(self):
        """Update path tracking variables"""
        if not self.current_path or not self.current_path.poses:
            return
            
        # Find closest point on path
        self.closest_point_index = self._find_closest_point_on_path()
        
        # Calculate cross-track error
        self.cross_track_error = self._calculate_cross_track_error()
        self.max_cross_track_error = max(self.max_cross_track_error, abs(self.cross_track_error))
        
        # Calculate distance to goal
        goal_pose = self.current_path.poses[-1]
        self.distance_to_goal = self._calculate_distance_to_pose(goal_pose)
        
        # Calculate path progress
        self.path_progress = self._calculate_path_progress()
        
        # Check if goal is reached
        if self.distance_to_goal <= self.pure_pursuit_config.goal_tolerance:
            self.state = PathFollowerState.GOAL_REACHED
            self._stop_rover()
            self.get_logger().info("Goal reached!")
            
    def _execute_path_following(self):
        """Execute pure pursuit path following"""
        # Check for obstacles
        if self.obstacle_config.enable and self.obstacle_detected:
            self.state = PathFollowerState.AVOIDING_OBSTACLE
            self.obstacle_avoidance_active = True
            return
            
        # Check for stuck condition
        if self.stuck_detected:
            self.state = PathFollowerState.STUCK_RECOVERY
            return
            
        # Calculate adaptive lookahead distance
        self._update_lookahead_distance()
        
        # Find lookahead point
        self.lookahead_point = self._find_lookahead_point()
        
        if self.lookahead_point:
            # Calculate control commands using pure pursuit
            self.target_velocity = self._calculate_pure_pursuit_control()
            
            # Apply velocity limits and smoothing
            self.target_velocity = self._apply_velocity_limits(self.target_velocity)
            self.target_velocity = self._apply_velocity_smoothing(self.target_velocity)
            
        else:
            # No valid lookahead point, stop rover
            self.target_velocity = Twist()
            
    def _execute_obstacle_avoidance(self):
        """Execute obstacle avoidance behavior"""
        if not self.obstacle_detected:
            # Obstacle cleared, return to path following
            self.state = PathFollowerState.FOLLOWING
            self.obstacle_avoidance_active = False
            return
            
        # Simple obstacle avoidance using potential fields
        avoidance_velocity = self._calculate_obstacle_avoidance_velocity()
        
        # Scale velocity for safety
        avoidance_velocity.linear.x *= self.obstacle_config.velocity_scaling
        avoidance_velocity.angular.z *= self.obstacle_config.velocity_scaling
        
        self.target_velocity = avoidance_velocity
        
    def _execute_stuck_recovery(self):
        """Execute stuck recovery behavior"""
        if not self.stuck_detected:
            # No longer stuck, return to path following
            self.state = PathFollowerState.FOLLOWING
            return
            
        # Simple stuck recovery: reverse for a short time
        recovery_velocity = Twist()
        recovery_velocity.linear.x = -0.5  # Reverse at 0.5 m/s
        
        self.target_velocity = recovery_velocity
        
    def _update_lookahead_distance(self):
        """Update adaptive lookahead distance based on velocity"""
        current_speed = math.sqrt(
            self.current_velocity.linear.x**2 + 
            self.current_velocity.linear.y**2
        )
        
        # Adaptive lookahead: distance increases with velocity
        adaptive_lookahead = (current_speed * self.pure_pursuit_config.lookahead_ratio + 
                             self.pure_pursuit_config.min_lookahead)
        
        self.current_lookahead_distance = min(
            self.pure_pursuit_config.max_lookahead,
            max(self.pure_pursuit_config.min_lookahead, adaptive_lookahead)
        )
        
    def _find_closest_point_on_path(self) -> int:
        """Find the closest point on the path to current position"""
        if not self.current_path or not self.current_path.poses:
            return 0
            
        min_distance = float('inf')
        closest_index = 0
        
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        
        for i, pose in enumerate(self.current_path.poses):
            dx = pose.pose.position.x - current_x
            dy = pose.pose.position.y - current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < min_distance:
                min_distance = distance
                closest_index = i
                
        return closest_index
        
    def _find_lookahead_point(self) -> Optional[Point]:
        """Find lookahead point on the path"""
        if not self.current_path or not self.current_path.poses:
            return None
            
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        
        # Start searching from closest point
        for i in range(self.closest_point_index, len(self.current_path.poses)):
            pose = self.current_path.poses[i]
            dx = pose.pose.position.x - current_x
            dy = pose.pose.position.y - current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance >= self.current_lookahead_distance:
                # Found lookahead point
                lookahead_point = Point()
                lookahead_point.x = pose.pose.position.x
                lookahead_point.y = pose.pose.position.y
                lookahead_point.z = pose.pose.position.z
                return lookahead_point
                
        # If no point found at lookahead distance, use the last point
        if self.current_path.poses:
            last_pose = self.current_path.poses[-1]
            lookahead_point = Point()
            lookahead_point.x = last_pose.pose.position.x
            lookahead_point.y = last_pose.pose.position.y
            lookahead_point.z = last_pose.pose.position.z
            return lookahead_point
            
        return None
        
    def _calculate_pure_pursuit_control(self) -> Twist:
        """Calculate pure pursuit control commands"""
        if not self.lookahead_point:
            return Twist()
            
        # Get current position and orientation
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        
        # Extract yaw from quaternion (simplified)
        current_yaw = self._extract_yaw_from_quaternion(self.current_pose.pose.orientation)
        
        # Calculate vector to lookahead point
        dx = self.lookahead_point.x - current_x
        dy = self.lookahead_point.y - current_y
        lookahead_distance = math.sqrt(dx**2 + dy**2)
        
        # Calculate angle to lookahead point
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - current_yaw
        
        # Normalize angle error
        angle_error = self._normalize_angle(angle_error)
        
        # Pure pursuit equations
        alpha = angle_error
        curvature = 2.0 * math.sin(alpha) / lookahead_distance
        
        # Calculate velocity commands
        velocity_cmd = Twist()
        
        # Linear velocity (proportional to distance to goal, with limits)
        target_speed = min(self.max_velocity, 
                          lookahead_distance * self.pure_pursuit_config.velocity_kp)
        velocity_cmd.linear.x = target_speed
        
        # Angular velocity from curvature
        velocity_cmd.angular.z = curvature * target_speed
        
        # Limit angular velocity
        max_angular_velocity = 1.0  # rad/s
        velocity_cmd.angular.z = max(-max_angular_velocity, 
                                    min(max_angular_velocity, velocity_cmd.angular.z))
                                    
        return velocity_cmd
        
    def _calculate_obstacle_avoidance_velocity(self) -> Twist:
        """Calculate obstacle avoidance velocity using simple potential fields"""
        # Simplified obstacle avoidance - in real implementation would use laser scan
        avoidance_velocity = Twist()
        
        # Turn away from obstacles (simplified)
        avoidance_velocity.linear.x = 0.3  # Slow forward motion
        avoidance_velocity.angular.z = 0.5  # Turn right to avoid obstacle
        
        return avoidance_velocity
        
    def _apply_velocity_limits(self, velocity: Twist) -> Twist:
        """Apply mission-specific velocity limits"""
        limited_velocity = Twist()
        
        # Linear velocity limits
        linear_speed = math.sqrt(velocity.linear.x**2 + velocity.linear.y**2)
        if linear_speed > self.max_velocity:
            scale_factor = self.max_velocity / linear_speed
            limited_velocity.linear.x = velocity.linear.x * scale_factor
            limited_velocity.linear.y = velocity.linear.y * scale_factor
        else:
            limited_velocity.linear.x = velocity.linear.x
            limited_velocity.linear.y = velocity.linear.y
            limited_velocity.linear.z = velocity.linear.z
            
        # Angular velocity limits
        max_angular_velocity = 1.0  # rad/s
        limited_velocity.angular.x = velocity.angular.x
        limited_velocity.angular.y = velocity.angular.y
        limited_velocity.angular.z = max(-max_angular_velocity,
                                        min(max_angular_velocity, velocity.angular.z))
                                        
        return limited_velocity
        
    def _apply_velocity_smoothing(self, velocity: Twist) -> Twist:
        """Apply velocity smoothing using exponential filter"""
        if not self.config['trajectory_smoothing_enable']:
            return velocity
            
        alpha = self.velocity_filter_alpha
        
        smoothed_velocity = Twist()
        smoothed_velocity.linear.x = (alpha * self.filtered_velocity.linear.x + 
                                     (1 - alpha) * velocity.linear.x)
        smoothed_velocity.linear.y = (alpha * self.filtered_velocity.linear.y + 
                                     (1 - alpha) * velocity.linear.y)
        smoothed_velocity.angular.z = (alpha * self.filtered_velocity.angular.z + 
                                      (1 - alpha) * velocity.angular.z)
                                      
        self.filtered_velocity = smoothed_velocity
        return smoothed_velocity
        
    def _calculate_cross_track_error(self) -> float:
        """Calculate cross-track error from path"""
        if not self.current_path or len(self.current_path.poses) < 2:
            return 0.0
            
        # Get current position
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        
        # Get closest path segment
        if self.closest_point_index >= len(self.current_path.poses) - 1:
            closest_point_index = len(self.current_path.poses) - 2
        else:
            closest_point_index = self.closest_point_index
            
        p1 = self.current_path.poses[closest_point_index].pose.position
        p2 = self.current_path.poses[closest_point_index + 1].pose.position
        
        # Calculate cross-track error using point-to-line distance
        # Vector from p1 to p2
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        
        # Vector from p1 to current position
        px = current_x - p1.x
        py = current_y - p1.y
        
        # Cross product gives signed distance
        cross_track_error = (dx * py - dy * px) / math.sqrt(dx**2 + dy**2)
        
        return cross_track_error
        
    def _calculate_distance_to_pose(self, pose: PoseStamped) -> float:
        """Calculate distance to a pose"""
        dx = pose.pose.position.x - self.current_pose.pose.position.x
        dy = pose.pose.position.y - self.current_pose.pose.position.y
        return math.sqrt(dx**2 + dy**2)
        
    def _calculate_path_progress(self) -> float:
        """Calculate progress along path (0.0 to 1.0)"""
        if not self.current_path or not self.current_path.poses:
            return 0.0
            
        total_length = self._calculate_path_length()
        if total_length == 0:
            return 1.0
            
        # Calculate length up to closest point
        progress_length = 0.0
        for i in range(self.closest_point_index):
            if i + 1 < len(self.current_path.poses):
                p1 = self.current_path.poses[i].pose.position
                p2 = self.current_path.poses[i + 1].pose.position
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                progress_length += math.sqrt(dx**2 + dy**2)
                
        return min(1.0, progress_length / total_length)
        
    def _calculate_path_length(self) -> float:
        """Calculate total path length"""
        if not self.current_path or len(self.current_path.poses) < 2:
            return 0.0
            
        total_length = 0.0
        for i in range(len(self.current_path.poses) - 1):
            p1 = self.current_path.poses[i].pose.position
            p2 = self.current_path.poses[i + 1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            total_length += math.sqrt(dx**2 + dy**2)
            
        return total_length
        
    def _extract_yaw_from_quaternion(self, quaternion) -> float:
        """Extract yaw angle from quaternion"""
        # Simplified yaw extraction
        return 2.0 * math.atan2(quaternion.z, quaternion.w)
        
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def _publish_control_commands(self):
        """Publish control commands"""
        # Publish velocity command
        self.cmd_vel_pub.publish(self.target_velocity)
        
        # Publish drive command with additional information
        drive_cmd = DriveCommand()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.velocity = self.target_velocity
        drive_cmd.control_mode = "velocity"
        drive_cmd.mission_mode = self.mission_mode
        drive_cmd.max_velocity = self.max_velocity
        drive_cmd.source = "path_follower"
        
        self.drive_command_pub.publish(drive_cmd)
        
    def _stop_rover(self):
        """Stop the rover"""
        stop_velocity = Twist()
        self.cmd_vel_pub.publish(stop_velocity)
        self.target_velocity = stop_velocity
        
    def _handle_safety_stop(self):
        """Handle safety stop condition"""
        self._stop_rover()
        self.state = PathFollowerState.FAILED
        self.get_logger().warn("Safety stop triggered - path following stopped")
        
    def _publish_status_callback(self):
        """Publish path following status"""
        # Publish lookahead point
        if self.lookahead_point:
            lookahead_msg = PointStamped()
            lookahead_msg.header.stamp = self.get_clock().now().to_msg()
            lookahead_msg.header.frame_id = "map"
            lookahead_msg.point = self.lookahead_point
            self.lookahead_pub.publish(lookahead_msg)
            
        # Publish path progress
        progress_msg = Float32()
        progress_msg.data = self.path_progress
        self.path_progress_pub.publish(progress_msg)
        
        # Publish cross-track error
        error_msg = Float32()
        error_msg.data = self.cross_track_error
        self.cross_track_error_pub.publish(error_msg)
        
        # Publish state
        state_msg = String()
        state_msg.data = self.state.value
        self.state_pub.publish(state_msg)
        
    # Callback functions
    def _odom_callback(self, msg: Odometry):
        """Process odometry updates"""
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        self.current_velocity = msg.twist.twist
        
        # Update distance traveled
        if self.path_start_time:
            dt = 0.1  # Assuming 10 Hz odometry
            linear_speed = math.sqrt(
                self.current_velocity.linear.x**2 + 
                self.current_velocity.linear.y**2
            )
            self.distance_traveled += linear_speed * dt
            
            # Update average velocity
            elapsed_time = time.time() - self.path_start_time
            if elapsed_time > 0:
                self.average_velocity = self.distance_traveled / elapsed_time
                
    def _path_callback(self, msg: Path):
        """Process new path"""
        self.current_path = msg
        self.closest_point_index = 0
        self.path_progress = 0.0
        self.distance_traveled = 0.0
        self.max_cross_track_error = 0.0
        self.path_start_time = time.time()
        
        if msg.poses:
            self.state = PathFollowerState.FOLLOWING
            self.get_logger().info(f"Received new path with {len(msg.poses)} waypoints")
        else:
            self.state = PathFollowerState.IDLE
            
    def _laser_callback(self, msg: LaserScan):
        """Process laser scan for obstacle detection"""
        if not self.obstacle_config.enable:
            return
            
        # Simple obstacle detection: check if any range is below threshold
        min_safe_distance = self.obstacle_config.safety_margin
        obstacle_detected = False
        
        # Check front sector of laser scan
        front_start = len(msg.ranges) // 3
        front_end = 2 * len(msg.ranges) // 3
        
        for i in range(front_start, front_end):
            if (msg.range_min <= msg.ranges[i] <= min_safe_distance):
                obstacle_detected = True
                break
                
        self.obstacle_detected = obstacle_detected
        
    def _safety_callback(self, msg: Bool):
        """Process safety status"""
        self.safety_ok = msg.data
        
    def _stuck_callback(self, msg: Bool):
        """Process stuck detection"""
        self.stuck_detected = msg.data
        
    def _rover_status_callback(self, msg: RoverStatus):
        """Process rover status updates"""
        self.mission_mode = msg.mission_mode
        
        # Update mission-specific parameters
        mission_configs = {
            'exploration': {'max_velocity': 1.8, 'max_acceleration': 1.2},
            'science': {'max_velocity': 0.8, 'max_acceleration': 0.6},
            'return_to_base': {'max_velocity': 2.2, 'max_acceleration': 1.5},
            'manual': {'max_velocity': 1.5, 'max_acceleration': 2.0}
        }
        
        if self.mission_mode in mission_configs:
            config = mission_configs[self.mission_mode]
            self.max_velocity = config['max_velocity']
            self.max_acceleration = config['max_acceleration']
            
    # Public interface methods
    def start_path_following(self, path: Path):
        """Start following a new path"""
        self._path_callback(path)
        
    def stop_path_following(self):
        """Stop path following"""
        self.state = PathFollowerState.IDLE
        self._stop_rover()
        
    def get_path_progress(self) -> float:
        """Get current path progress (0.0 to 1.0)"""
        return self.path_progress
        
    def get_cross_track_error(self) -> float:
        """Get current cross-track error"""
        return self.cross_track_error
        
    def is_goal_reached(self) -> bool:
        """Check if goal has been reached"""
        return self.state == PathFollowerState.GOAL_REACHED


def main(args=None):
    """Main entry point for the path follower node"""
    rclpy.init(args=args)
    
    try:
        node = RoverPathFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in path follower: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 