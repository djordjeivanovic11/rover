#!/usr/bin/env python3
"""
=============================================================================
ROVER ACTION SERVERS
=============================================================================
Action servers for autonomous rover operation with URC-specific features.

Exposes four main actions:
- DriveToPose: Navigate to a target pose with path planning
- FollowPath: Execute a predefined waypoint path
- SetVelocity: Direct velocity control for teleoperation
- CancelMotion: Emergency stop and motion cancellation

Each action includes safety monitoring, progress tracking, and URC
competition-specific features like autonomy timer and visual servoing.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus

import numpy as np
import math
import time
import threading
from typing import Dict, List, Optional, Tuple
from enum import Enum
from dataclasses import dataclass

from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, String
from rover_control.action import DriveToPose, FollowPath, SetVelocity, CancelMotion
from rover_control.msg import RoverStatus, DriveCommand


class ActionState(Enum):
    """Action execution states"""
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    EXECUTING = "EXECUTING"
    APPROACHING = "APPROACHING"
    VISUAL_SERVOING = "VISUAL_SERVOING"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


class ResultCode(Enum):
    """Standardized result codes"""
    SUCCESS = 0
    CANCELLED = 1
    TIMEOUT = 2
    STUCK = 3
    OBSTACLE = 4
    UNREACHABLE = 5
    SAFETY_VIOLATION = 6
    AUTONOMY_TIMEOUT = 7
    HARDWARE_FAILURE = 8
    UNKNOWN_ERROR = 9


@dataclass
class MissionModeConfig:
    """Mission mode configuration"""
    max_velocity: float
    max_acceleration: float
    position_tolerance: float
    orientation_tolerance: float
    lookahead_distance: float


class RoverActionServers(Node):
    """
    Action servers for autonomous rover operation.
    
    Provides high-level navigation and control actions with:
    - Path planning and execution
    - Safety monitoring integration
    - Progress tracking and feedback
    - URC competition features
    - Visual servoing for precision approaches
    """
    
    def __init__(self):
        super().__init__("rover_action_servers")
        
        # Load configuration
        self._load_configuration()
        
        # System state
        self.current_action = "none"
        self.action_in_progress = False
        self.current_pose = PoseStamped()
        self.current_velocity = Twist()
        self.mission_mode = "exploration"
        self.safety_ok = True
        self.autonomy_timer_active = False
        self.autonomy_time_remaining = 60.0
        
        # Mission mode configurations
        self.mission_configs = {
            'exploration': MissionModeConfig(1.8, 1.2, 0.1, 0.1, 1.5),
            'science': MissionModeConfig(0.8, 0.6, 0.05, 0.05, 0.8),
            'return_to_base': MissionModeConfig(2.2, 1.5, 0.2, 0.15, 2.0),
            'manual': MissionModeConfig(1.5, 2.0, 0.1, 0.1, 1.0)
        }
        
        # Action execution state
        self.action_state = ActionState.IDLE
        self.action_start_time = None
        self.goal_tolerance_met = False
        self.stuck_detected = False
        self.visual_servoing_active = False
        
        # Path following state
        self.current_path = None
        self.current_waypoint_index = 0
        self.waypoints_completed = 0
        self.distance_traveled = 0.0
        self.path_start_pose = None
        
        # Velocity control state
        self.target_velocity = Twist()
        self.velocity_start_time = None
        self.velocity_duration = 0.0
        
        # Callback groups for concurrent actions
        self.action_callback_group = ReentrantCallbackGroup()
        self.client_callback_group = ReentrantCallbackGroup()
        
        # Action servers
        self.drive_to_pose_server = ActionServer(
            self,
            DriveToPose,
            "/rover/drive_to_pose",
            self._execute_drive_to_pose,
            callback_group=self.action_callback_group
        )
        
        self.follow_path_server = ActionServer(
            self,
            FollowPath,
            "/rover/follow_path",
            self._execute_follow_path,
            callback_group=self.action_callback_group
        )
        
        self.set_velocity_server = ActionServer(
            self,
            SetVelocity,
            "/rover/set_velocity",
            self._execute_set_velocity,
            callback_group=self.action_callback_group
        )
        
        self.cancel_motion_server = ActionServer(
            self,
            CancelMotion,
            "/rover/cancel_motion",
            self._execute_cancel_motion,
            callback_group=self.action_callback_group
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.drive_command_pub = self.create_publisher(DriveCommand, "/rover/drive_cmd", 10)
        self.status_pub = self.create_publisher(RoverStatus, "/rover/status", 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_callback, 10)
        self.safety_sub = self.create_subscription(
            Bool, "/rover/safety_ok", self._safety_callback, 10)
        self.stuck_sub = self.create_subscription(
            Bool, "/rover/stuck_detected", self._stuck_callback, 10)
        self.autonomy_timer_sub = self.create_subscription(
            Float32, "/rover/autonomy_time_remaining", self._autonomy_timer_callback, 10)
        
        # Status publishing timer
        self.status_timer = self.create_timer(1.0, self._publish_status_callback)
        
        self.get_logger().info("Rover Action Servers initialized")
        
    def _load_configuration(self):
        """Load configuration from parameters"""
        param_defaults = {
            'default_mission_mode': 'exploration',
            'max_autonomy_time': 60.0,
            'stuck_recovery_enable': True,
            'visual_servoing_enable': False,
            'visual_servoing_distance': 3.0,
            'path_following_frequency': 10.0,
            'goal_check_frequency': 20.0,
            'safety_check_frequency': 50.0
        }
        
        self.config = {}
        for param_name, default_value in param_defaults.items():
            self.declare_parameter(param_name, default_value)
            self.config[param_name] = self.get_parameter(param_name).value
            
    def _execute_drive_to_pose(self, goal_handle):
        """Execute DriveToPose action"""
        self.get_logger().info("Executing DriveToPose action")
        
        goal = goal_handle.request
        feedback = DriveToPose.Feedback()
        result = DriveToPose.Result()
        
        # Initialize action state
        self.action_in_progress = True
        self.current_action = "drive_to_pose"
        self.action_state = ActionState.PLANNING
        self.action_start_time = time.time()
        
        # Start autonomy timer if needed
        if goal.mission_mode != "manual":
            self._start_autonomy_timer()
            
        try:
            # Validate goal
            if not self._validate_drive_to_pose_goal(goal):
                result.success = False
                result.result_code = ResultCode.UNREACHABLE.value
                result.error_message = "Invalid or unreachable goal"
                goal_handle.abort()
                return result
                
            # Set mission mode
            self.mission_mode = goal.mission_mode
            config = self.mission_configs.get(self.mission_mode, self.mission_configs['exploration'])
            
            # Plan path to target
            self.action_state = ActionState.PLANNING
            feedback.current_state = self.action_state.value
            feedback.progress_percentage = 0.0
            goal_handle.publish_feedback(feedback)
            
            # Simple direct path planning (in real implementation, use navigation stack)
            target_pose = goal.target_pose
            path = self._plan_path_to_pose(target_pose)
            
            if not path:
                result.success = False
                result.result_code = ResultCode.UNREACHABLE.value
                result.error_message = "Path planning failed"
                goal_handle.abort()
                return result
                
            # Execute path
            self.action_state = ActionState.EXECUTING
            execution_result = self._execute_path_to_pose(goal, goal_handle, feedback, path)
            
            if execution_result:
                # Check if visual servoing should be used for final approach
                distance_to_goal = self._calculate_distance_to_pose(target_pose)
                
                if (goal.use_visual_servoing and 
                    distance_to_goal <= goal.approach_velocity and
                    self.config['visual_servoing_enable']):
                    
                    self.action_state = ActionState.VISUAL_SERVOING
                    visual_result = self._execute_visual_servoing(goal, goal_handle, feedback)
                    
                    if not visual_result:
                        result.success = False
                        result.result_code = ResultCode.TIMEOUT.value
                        result.error_message = "Visual servoing failed"
                        goal_handle.abort()
                        return result
                        
                # Final approach
                self.action_state = ActionState.APPROACHING
                approach_result = self._execute_final_approach(goal, goal_handle, feedback)
                
                if approach_result:
                    # Success
                    result.success = True
                    result.result_code = ResultCode.SUCCESS.value
                    result.error_message = "Goal reached successfully"
                    
                    # Fill result data
                    result.final_pose = self.current_pose
                    result.execution_time = time.time() - self.action_start_time
                    result.distance_traveled = self.distance_traveled
                    result.final_position_error = self._calculate_distance_to_pose(target_pose)
                    result.final_orientation_error = self._calculate_orientation_error(target_pose)
                    
                    self.action_state = ActionState.COMPLETE
                    goal_handle.succeed()
                    
                else:
                    result.success = False
                    result.result_code = ResultCode.TIMEOUT.value
                    result.error_message = "Failed to reach goal within tolerance"
                    goal_handle.abort()
                    
            else:
                result.success = False
                result.result_code = ResultCode.STUCK.value
                result.error_message = "Execution failed"
                goal_handle.abort()
                
        except Exception as e:
            self.get_logger().error(f"DriveToPose execution error: {str(e)}")
            result.success = False
            result.result_code = ResultCode.UNKNOWN_ERROR.value
            result.error_message = f"Execution error: {str(e)}"
            goal_handle.abort()
            
        finally:
            self._cleanup_action()
            
        return result
        
    def _execute_follow_path(self, goal_handle):
        """Execute FollowPath action"""
        self.get_logger().info("Executing FollowPath action")
        
        goal = goal_handle.request
        feedback = FollowPath.Feedback()
        result = FollowPath.Result()
        
        # Initialize action state
        self.action_in_progress = True
        self.current_action = "follow_path"
        self.action_state = ActionState.EXECUTING
        self.action_start_time = time.time()
        self.current_path = goal.path
        self.current_waypoint_index = 0
        self.waypoints_completed = 0
        self.distance_traveled = 0.0
        self.path_start_pose = self.current_pose
        
        # Start autonomy timer
        if goal.mission_mode != "manual":
            self._start_autonomy_timer()
            
        try:
            # Validate path
            if not goal.path.poses or len(goal.path.poses) == 0:
                result.success = False
                result.result_code = ResultCode.UNREACHABLE.value
                result.error_message = "Empty path provided"
                goal_handle.abort()
                return result
                
            # Set mission mode
            self.mission_mode = goal.mission_mode
            config = self.mission_configs.get(self.mission_mode, self.mission_configs['exploration'])
            
            total_waypoints = len(goal.path.poses)
            
            # Execute path waypoint by waypoint
            for i, waypoint in enumerate(goal.path.poses):
                if goal_handle.is_cancel_requested:
                    break
                    
                self.current_waypoint_index = i
                
                # Navigate to waypoint
                waypoint_result = self._navigate_to_waypoint(waypoint, goal, goal_handle, feedback, config)
                
                if waypoint_result:
                    self.waypoints_completed += 1
                    
                    # Update feedback
                    feedback.current_state = "APPROACHING_WAYPOINT"
                    feedback.progress_percentage = (self.waypoints_completed / total_waypoints) * 100.0
                    feedback.current_waypoint = i
                    feedback.target_waypoint = waypoint
                    feedback.current_pose = self.current_pose
                    goal_handle.publish_feedback(feedback)
                    
                else:
                    if goal.skip_unreachable:
                        self.get_logger().warn(f"Skipping unreachable waypoint {i}")
                        continue
                    else:
                        result.success = False
                        result.result_code = ResultCode.UNREACHABLE.value
                        result.error_message = f"Failed to reach waypoint {i}"
                        goal_handle.abort()
                        return result
                        
            # Check if action was cancelled
            if goal_handle.is_cancel_requested:
                result.success = False
                result.result_code = ResultCode.CANCELLED.value
                result.error_message = "Path following cancelled"
                goal_handle.canceled()
            else:
                # Success
                result.success = True
                result.result_code = ResultCode.SUCCESS.value
                result.error_message = "Path completed successfully"
                result.waypoints_completed = self.waypoints_completed
                result.total_waypoints = total_waypoints
                result.execution_time = time.time() - self.action_start_time
                result.total_distance = self.distance_traveled
                result.final_pose = self.current_pose
                
                self.action_state = ActionState.COMPLETE
                goal_handle.succeed()
                
        except Exception as e:
            self.get_logger().error(f"FollowPath execution error: {str(e)}")
            result.success = False
            result.result_code = ResultCode.UNKNOWN_ERROR.value
            result.error_message = f"Execution error: {str(e)}"
            goal_handle.abort()
            
        finally:
            self._cleanup_action()
            
        return result
        
    def _execute_set_velocity(self, goal_handle):
        """Execute SetVelocity action"""
        self.get_logger().info("Executing SetVelocity action")
        
        goal = goal_handle.request
        feedback = SetVelocity.Feedback()
        result = SetVelocity.Result()
        
        # Initialize action state
        self.action_in_progress = True
        self.current_action = "set_velocity"
        self.action_state = ActionState.EXECUTING
        self.action_start_time = time.time()
        self.velocity_start_time = time.time()
        self.velocity_duration = goal.duration
        self.target_velocity = goal.target_velocity
        
        try:
            # Set mission mode
            self.mission_mode = goal.mission_mode
            config = self.mission_configs.get(self.mission_mode, self.mission_configs['exploration'])
            
            # Apply safety limits if requested
            if goal.enforce_safety_limits:
                limited_velocity = self._apply_velocity_limits(goal.target_velocity, config)
            else:
                limited_velocity = goal.target_velocity
                
            # Ramp to velocity if requested
            if goal.ramp_to_velocity:
                self.action_state = ActionState.EXECUTING
                ramp_result = self._ramp_to_velocity(limited_velocity, goal, goal_handle, feedback)
                if not ramp_result:
                    result.success = False
                    result.result_code = ResultCode.TIMEOUT.value
                    result.error_message = "Failed to ramp to target velocity"
                    goal_handle.abort()
                    return result
                    
            # Maintain velocity for specified duration
            start_time = time.time()
            end_time = start_time + goal.duration if goal.duration > 0 else float('inf')
            
            while time.time() < end_time and not goal_handle.is_cancel_requested:
                # Check safety
                if not self.safety_ok:
                    break
                    
                # Publish velocity command
                self.cmd_vel_pub.publish(limited_velocity)
                
                # Update feedback
                elapsed_time = time.time() - start_time
                feedback.current_state = "MAINTAINING"
                feedback.current_velocity = self.current_velocity
                feedback.time_remaining = max(0, goal.duration - elapsed_time) if goal.duration > 0 else 0
                feedback.progress_percentage = min(100.0, (elapsed_time / goal.duration) * 100.0) if goal.duration > 0 else 100.0
                goal_handle.publish_feedback(feedback)
                
                time.sleep(0.1)  # 10 Hz update rate
                
            # Stop rover
            stop_velocity = Twist()
            self.cmd_vel_pub.publish(stop_velocity)
            
            # Check if action was cancelled
            if goal_handle.is_cancel_requested:
                result.success = False
                result.result_code = ResultCode.CANCELLED.value
                result.error_message = "Velocity control cancelled"
                goal_handle.canceled()
            else:
                # Success
                result.success = True
                result.result_code = ResultCode.SUCCESS.value
                result.error_message = "Velocity control completed"
                result.execution_time = time.time() - self.action_start_time
                result.final_velocity = self.current_velocity
                
                goal_handle.succeed()
                
        except Exception as e:
            self.get_logger().error(f"SetVelocity execution error: {str(e)}")
            result.success = False
            result.result_code = ResultCode.UNKNOWN_ERROR.value
            result.error_message = f"Execution error: {str(e)}"
            goal_handle.abort()
            
        finally:
            # Ensure rover is stopped
            stop_velocity = Twist()
            self.cmd_vel_pub.publish(stop_velocity)
            self._cleanup_action()
            
        return result
        
    def _execute_cancel_motion(self, goal_handle):
        """Execute CancelMotion action"""
        self.get_logger().info("Executing CancelMotion action")
        
        goal = goal_handle.request
        feedback = CancelMotion.Feedback()
        result = CancelMotion.Result()
        
        try:
            cancel_start_time = time.time()
            initial_velocity = self.current_velocity
            
            # Cancel all running actions
            actions_cancelled = 0
            if self.action_in_progress:
                actions_cancelled = 1
                self.action_in_progress = False
                
            if goal.emergency_stop:
                # Immediate stop
                feedback.current_state = "STOPPING"
                stop_velocity = Twist()
                self.cmd_vel_pub.publish(stop_velocity)
                
                # Publish emergency stop
                estop_msg = Bool()
                estop_msg.data = True
                # Would publish to emergency stop topic if available
                
                stop_time = 0.1  # Immediate stop
                
            else:
                # Gradual deceleration
                feedback.current_state = "STOPPING"
                decel_time = goal.deceleration_time
                
                # Calculate deceleration
                current_speed = math.sqrt(self.current_velocity.linear.x**2 + self.current_velocity.linear.y**2)
                if current_speed > 0 and decel_time > 0:
                    decel_rate = current_speed / decel_time
                    
                    # Gradual deceleration
                    start_time = time.time()
                    while time.time() - start_time < decel_time:
                        elapsed = time.time() - start_time
                        remaining_time = decel_time - elapsed
                        speed_factor = remaining_time / decel_time
                        
                        decel_velocity = Twist()
                        decel_velocity.linear.x = self.current_velocity.linear.x * speed_factor
                        decel_velocity.angular.z = self.current_velocity.angular.z * speed_factor
                        
                        self.cmd_vel_pub.publish(decel_velocity)
                        
                        # Update feedback
                        feedback.progress_percentage = (elapsed / decel_time) * 100.0
                        feedback.current_velocity = decel_velocity
                        goal_handle.publish_feedback(feedback)
                        
                        time.sleep(0.05)  # 20 Hz
                        
                stop_time = decel_time
                
            # Final stop
            stop_velocity = Twist()
            self.cmd_vel_pub.publish(stop_velocity)
            
            feedback.current_state = "STOPPED"
            feedback.progress_percentage = 100.0
            feedback.current_velocity = stop_velocity
            goal_handle.publish_feedback(feedback)
            
            # Success
            result.success = True
            result.result_code = ResultCode.SUCCESS.value
            result.error_message = f"Motion cancelled: {goal.reason}"
            result.stop_time = time.time() - cancel_start_time
            result.velocity_at_stop = initial_velocity
            result.final_pose = self.current_pose
            result.actions_cancelled = actions_cancelled
            
            goal_handle.succeed()
            
        except Exception as e:
            self.get_logger().error(f"CancelMotion execution error: {str(e)}")
            result.success = False
            result.result_code = ResultCode.UNKNOWN_ERROR.value
            result.error_message = f"Execution error: {str(e)}"
            goal_handle.abort()
            
        finally:
            self._cleanup_action()
            
        return result
        
    # Helper methods
    def _validate_drive_to_pose_goal(self, goal) -> bool:
        """Validate DriveToPose goal"""
        # Check if target is reachable (simplified check)
        target_x = goal.target_pose.pose.position.x
        target_y = goal.target_pose.pose.position.y
        
        # Simple distance check (in real implementation, use navigation capabilities)
        distance = math.sqrt(target_x**2 + target_y**2)
        max_distance = 100.0  # meters
        
        return distance <= max_distance
        
    def _plan_path_to_pose(self, target_pose: PoseStamped) -> Optional[Path]:
        """Plan path to target pose (simplified)"""
        # In real implementation, this would use the navigation stack
        path = Path()
        path.header = target_pose.header
        
        # Simple straight-line path for demonstration
        path.poses.append(self.current_pose)
        path.poses.append(target_pose)
        
        return path
        
    def _execute_path_to_pose(self, goal, goal_handle, feedback, path: Path) -> bool:
        """Execute path to pose"""
        # Simplified path execution using pure pursuit
        config = self.mission_configs.get(self.mission_mode, self.mission_configs['exploration'])
        
        # Execute using pure pursuit algorithm (simplified)
        return self._pure_pursuit_controller(path, goal, goal_handle, feedback, config)
        
    def _pure_pursuit_controller(self, path: Path, goal, goal_handle, feedback, config: MissionModeConfig) -> bool:
        """Pure pursuit path following controller"""
        lookahead_distance = config.lookahead_distance
        max_velocity = config.max_velocity
        
        start_time = time.time()
        timeout = goal.timeout if hasattr(goal, 'timeout') else 60.0
        
        while time.time() - start_time < timeout and not goal_handle.is_cancel_requested:
            # Check safety
            if not self.safety_ok or self.stuck_detected:
                return False
                
            # Find lookahead point
            lookahead_point = self._find_lookahead_point(path, lookahead_distance)
            if not lookahead_point:
                break  # Reached end of path
                
            # Calculate control commands
            velocity_cmd = self._calculate_pure_pursuit_velocity(lookahead_point, max_velocity)
            
            # Publish velocity command
            self.cmd_vel_pub.publish(velocity_cmd)
            
            # Update feedback
            progress = self._calculate_path_progress(path)
            feedback.progress_percentage = progress * 100.0
            feedback.current_pose = self.current_pose
            feedback.estimated_time_remaining = (timeout - (time.time() - start_time))
            goal_handle.publish_feedback(feedback)
            
            time.sleep(0.1)  # 10 Hz control loop
            
        # Stop rover
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        return not goal_handle.is_cancel_requested and self.safety_ok
        
    def _execute_visual_servoing(self, goal, goal_handle, feedback) -> bool:
        """Execute visual servoing approach (URC-specific)"""
        if not self.config['visual_servoing_enable']:
            return True
            
        self.visual_servoing_active = True
        # TODO: Implement visual servoing using tennis ball detection
        # This would integrate with the perception system
        time.sleep(1.0)  # Placeholder
        self.visual_servoing_active = False
        
        return True
        
    def _execute_final_approach(self, goal, goal_handle, feedback) -> bool:
        """Execute final approach to goal"""
        config = self.mission_configs.get(self.mission_mode, self.mission_configs['exploration'])
        approach_velocity = getattr(goal, 'approach_velocity', config.max_velocity * 0.5)
        
        start_time = time.time()
        timeout = 10.0  # 10 second timeout for final approach
        
        while time.time() - start_time < timeout:
            distance_error = self._calculate_distance_to_pose(goal.target_pose)
            orientation_error = self._calculate_orientation_error(goal.target_pose)
            
            # Check if goal is reached
            if (distance_error <= goal.position_tolerance and 
                orientation_error <= goal.orientation_tolerance):
                return True
                
            # Calculate velocity command for approach
            velocity_cmd = self._calculate_approach_velocity(goal.target_pose, approach_velocity)
            self.cmd_vel_pub.publish(velocity_cmd)
            
            time.sleep(0.05)  # 20 Hz control
            
        return False
        
    def _navigate_to_waypoint(self, waypoint: PoseStamped, goal, goal_handle, feedback, config: MissionModeConfig) -> bool:
        """Navigate to a single waypoint"""
        start_time = time.time()
        timeout = 30.0  # 30 second timeout per waypoint
        
        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested or not self.safety_ok:
                return False
                
            distance_error = self._calculate_distance_to_pose(waypoint)
            
            # Check if waypoint is reached
            if distance_error <= goal.waypoint_tolerance:
                return True
                
            # Calculate velocity command
            velocity_cmd = self._calculate_approach_velocity(waypoint, config.max_velocity)
            self.cmd_vel_pub.publish(velocity_cmd)
            
            time.sleep(0.1)  # 10 Hz control
            
        return False
        
    def _apply_velocity_limits(self, velocity: Twist, config: MissionModeConfig) -> Twist:
        """Apply mission-specific velocity limits"""
        limited_velocity = Twist()
        
        # Linear velocity limits
        linear_speed = math.sqrt(velocity.linear.x**2 + velocity.linear.y**2)
        if linear_speed > config.max_velocity:
            scale_factor = config.max_velocity / linear_speed
            limited_velocity.linear.x = velocity.linear.x * scale_factor
            limited_velocity.linear.y = velocity.linear.y * scale_factor
        else:
            limited_velocity.linear.x = velocity.linear.x
            limited_velocity.linear.y = velocity.linear.y
            
        # Angular velocity limits (simplified)
        max_angular_velocity = 1.0  # rad/s
        limited_velocity.angular.z = max(-max_angular_velocity, 
                                        min(max_angular_velocity, velocity.angular.z))
                                        
        return limited_velocity
        
    def _ramp_to_velocity(self, target_velocity: Twist, goal, goal_handle, feedback) -> bool:
        """Gradually ramp to target velocity"""
        ramp_time = 2.0  # 2 second ramp time
        steps = 20
        dt = ramp_time / steps
        
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                return False
                
            scale_factor = (i + 1) / steps
            
            ramped_velocity = Twist()
            ramped_velocity.linear.x = target_velocity.linear.x * scale_factor
            ramped_velocity.linear.y = target_velocity.linear.y * scale_factor
            ramped_velocity.angular.z = target_velocity.angular.z * scale_factor
            
            self.cmd_vel_pub.publish(ramped_velocity)
            
            feedback.current_state = "RAMPING"
            feedback.progress_percentage = scale_factor * 100.0
            feedback.current_velocity = ramped_velocity
            goal_handle.publish_feedback(feedback)
            
            time.sleep(dt)
            
        return True
        
    def _find_lookahead_point(self, path: Path, lookahead_distance: float) -> Optional[Point]:
        """Find lookahead point for pure pursuit"""
        # Simplified lookahead point calculation
        if not path.poses:
            return None
            
        # For simplicity, return the last pose
        last_pose = path.poses[-1]
        lookahead_point = Point()
        lookahead_point.x = last_pose.pose.position.x
        lookahead_point.y = last_pose.pose.position.y
        lookahead_point.z = last_pose.pose.position.z
        
        return lookahead_point
        
    def _calculate_pure_pursuit_velocity(self, lookahead_point: Point, max_velocity: float) -> Twist:
        """Calculate velocity command for pure pursuit"""
        # Simplified pure pursuit calculation
        dx = lookahead_point.x - self.current_pose.pose.position.x
        dy = lookahead_point.y - self.current_pose.pose.position.y
        
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # Extract current orientation (simplified)
        current_angle = 0.0  # Would extract from quaternion
        angle_error = target_angle - current_angle
        
        # Normalize angle
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
            
        velocity_cmd = Twist()
        velocity_cmd.linear.x = min(max_velocity, distance * 0.5)
        velocity_cmd.angular.z = angle_error * 1.0
        
        return velocity_cmd
        
    def _calculate_approach_velocity(self, target_pose: PoseStamped, max_velocity: float) -> Twist:
        """Calculate velocity command for approaching a target"""
        dx = target_pose.pose.position.x - self.current_pose.pose.position.x
        dy = target_pose.pose.position.y - self.current_pose.pose.position.y
        
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # Simple proportional controller
        velocity_cmd = Twist()
        velocity_cmd.linear.x = min(max_velocity, distance * 0.8)
        velocity_cmd.angular.z = target_angle * 0.5
        
        return velocity_cmd
        
    def _calculate_distance_to_pose(self, target_pose: PoseStamped) -> float:
        """Calculate distance to target pose"""
        dx = target_pose.pose.position.x - self.current_pose.pose.position.x
        dy = target_pose.pose.position.y - self.current_pose.pose.position.y
        return math.sqrt(dx**2 + dy**2)
        
    def _calculate_orientation_error(self, target_pose: PoseStamped) -> float:
        """Calculate orientation error to target pose"""
        # Simplified orientation error calculation
        # In real implementation, would properly handle quaternions
        return 0.1  # Placeholder
        
    def _calculate_path_progress(self, path: Path) -> float:
        """Calculate progress along path (0.0 to 1.0)"""
        # Simplified progress calculation
        return 0.5  # Placeholder
        
    def _start_autonomy_timer(self):
        """Start autonomy timer"""
        self.autonomy_timer_active = True
        # Would communicate with safety monitor to start timer
        
    def _cleanup_action(self):
        """Clean up after action completion"""
        self.action_in_progress = False
        self.current_action = "none"
        self.action_state = ActionState.IDLE
        self.autonomy_timer_active = False
        
        # Stop rover
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
    # Callback functions
    def _odom_callback(self, msg: Odometry):
        """Process odometry updates"""
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        self.current_velocity = msg.twist.twist
        
    def _safety_callback(self, msg: Bool):
        """Process safety status updates"""
        self.safety_ok = msg.data
        
    def _stuck_callback(self, msg: Bool):
        """Process stuck detection updates"""
        self.stuck_detected = msg.data
        
    def _autonomy_timer_callback(self, msg):
        """Process autonomy timer updates"""
        self.autonomy_time_remaining = msg.data
        
    def _publish_status_callback(self):
        """Publish rover status"""
        status_msg = RoverStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.current_action = self.current_action
        status_msg.mission_mode = self.mission_mode
        status_msg.is_autonomous = self.autonomy_timer_active
        status_msg.safety_ok = self.safety_ok
        status_msg.current_pose = self.current_pose
        status_msg.current_velocity = self.current_velocity
        status_msg.autonomy_time_remaining = self.autonomy_time_remaining
        status_msg.stuck_detected = self.stuck_detected
        status_msg.visual_servoing_active = self.visual_servoing_active
        
        self.status_pub.publish(status_msg)


def main(args=None):
    """Main entry point for the action servers node"""
    rclpy.init(args=args)
    
    try:
        node = RoverActionServers()
        
        # Use MultiThreadedExecutor for concurrent action handling
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in action servers: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 