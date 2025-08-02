#!/usr/bin/env python3
"""
=============================================================================
GRIPPER CONTROLLER
=============================================================================
Dedicated gripper controller that ramps finger torque, monitors force/slip,
and tags grasps as SECURE/SOFT/FAILED for mission logic feedback.
Provides force-based and position-based grasping strategies.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import time
from typing import Optional, Dict
from pathlib import Path
import yaml
from enum import Enum

from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import WrenchStamped

from arm_control.msg import ArmStatus, Fault


class GraspStatus(Enum):
    """Grasp status enumeration"""
    UNKNOWN = "UNKNOWN"
    SECURE = "SECURE"
    SOFT = "SOFT"
    FAILED = "FAILED"
    SLIPPING = "SLIPPING"


class GripperController(Node):
    """
    Gripper controller with force monitoring and grasp verification.
    
    Features:
    - Force-based and position-based grasping
    - Torque ramping for gentle grasping
    - Slip detection and grasp security assessment
    - Structured grasp outcome reporting
    """
    
    def __init__(self):
        super().__init__("gripper_controller")
        
        # Load configuration
        self._load_configuration()
        
        # Gripper state
        self.gripper_position = 0.0  # meters (0 = closed, max_opening = open)
        self.gripper_velocity = 0.0  # m/s
        self.gripper_force = 0.0     # N
        self.gripper_current = 0.0   # A
        
        # Command state
        self.target_position = 0.0
        self.target_force = 0.0
        self.command_active = False
        self.grasp_strategy = "position"  # "position", "force", "adaptive"
        
        # Grasp monitoring
        self.grasp_status = GraspStatus.UNKNOWN
        self.object_detected = False
        self.slip_detected = False
        self.force_stable = False
        self.force_history = []
        self.position_history = []
        
        # Control parameters
        self.max_opening = 0.12      # meters
        self.max_force = 100.0       # N
        self.force_ramp_rate = 20.0  # N/s
        self.position_tolerance = 0.001  # m
        self.force_tolerance = 2.0   # N
        self.slip_threshold = 0.5    # N/s force change rate
        
        # Callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()
        
        # Action server for gripper commands
        self.gripper_action_server = ActionServer(
            self,
            GripperCommand,
            "/arm/gripper_command",
            self._execute_gripper_command,
            callback_group=self.callback_group
        )
        
        # Publishers
        self.gripper_status_pub = self.create_publisher(
            JointState, "/gripper/joint_states", 10)
        self.grasp_status_pub = self.create_publisher(
            ArmStatus, "/gripper/grasp_status", 10)
        self.fault_pub = self.create_publisher(
            Fault, "/arm/faults", 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10)
        self.gripper_force_sub = self.create_subscription(
            Float32MultiArray, "/gripper/force_sensors", self._force_callback, 10)
        self.tcp_wrench_sub = self.create_subscription(
            WrenchStamped, "/arm/tcp_wrench", self._tcp_wrench_callback, 10)
        
        # Control timer
        self.control_timer = self.create_timer(
            0.01, self._control_loop_callback)  # 100 Hz control loop
        
        # Status timer
        self.status_timer = self.create_timer(
            0.1, self._publish_status_callback)  # 10 Hz status updates
        
        self.get_logger().info("Gripper Controller initialized")
    
    def _load_configuration(self):
        """Load gripper configuration from YAML"""
        try:
            # Load arm parameters for gripper config
            arm_params_path = Path(__file__).parent.parent / "config" / "arm_params.yaml"
            with open(arm_params_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Extract gripper parameters
            gripper_config = config.get('arm', {}).get('gripper', {})
            self.max_opening = gripper_config.get('max_opening', 0.12)
            self.max_force = gripper_config.get('max_force', 100.0)
            
            # Load controller config
            controller_config_path = Path(__file__).parent.parent / "config" / "controller_config.yaml"
            with open(controller_config_path, 'r') as f:
                controller_config = yaml.safe_load(f)
            
            # Extract gripper controller parameters
            gripper_controller_config = controller_config.get('gripper_controller', {}).get('ros__parameters', {})
            self.position_tolerance = gripper_controller_config.get('goal_tolerance', 0.001)
            
            self.get_logger().info(f"Gripper configuration loaded - Max opening: {self.max_opening}m, Max force: {self.max_force}N")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load gripper configuration: {e}")
            # Use default values
    
    def _joint_state_callback(self, msg: JointState):
        """Update gripper joint state"""
        try:
            # Find gripper joint in joint state message
            if "gripper_joint" in msg.name:
                idx = msg.name.index("gripper_joint")
                if idx < len(msg.position):
                    self.gripper_position = msg.position[idx]
                if idx < len(msg.velocity):
                    self.gripper_velocity = msg.velocity[idx]
                if idx < len(msg.effort):
                    # Convert effort to force (simplified)
                    self.gripper_force = abs(msg.effort[idx]) * 10.0  # Rough conversion
                    
        except Exception as e:
            self.get_logger().error(f"Failed to process gripper joint state: {e}")
    
    def _force_callback(self, msg: Float32MultiArray):
        """Update gripper force sensor data"""
        if len(msg.data) > 0:
            self.gripper_force = msg.data[0]
            
            # Update force history for slip detection
            self.force_history.append((time.time(), self.gripper_force))
            # Keep only recent history (last 1 second)
            cutoff_time = time.time() - 1.0
            self.force_history = [(t, f) for t, f in self.force_history if t > cutoff_time]
    
    def _tcp_wrench_callback(self, msg: WrenchStamped):
        """Update TCP wrench data for grasp analysis"""
        # TCP wrench can provide additional information about grasp quality
        # This could be used to detect object contact and grasp stability
        pass
    
    async def _execute_gripper_command(self, goal_handle):
        """Execute gripper command action"""
        self.get_logger().info("Received gripper command")
        
        try:
            goal = goal_handle.request
            command = goal.command
            
            # Validate command
            if not self._validate_gripper_command(command):
                goal_handle.abort()
                result = GripperCommand.Result()
                result.reached_goal = False
                result.stalled = False
                return result
            
            # Set command parameters
            self.target_position = np.clip(command.position, 0.0, self.max_opening)
            self.target_force = np.clip(command.max_effort, 0.0, self.max_force)
            self.command_active = True
            
            # Determine grasp strategy
            if self.target_force > 0.0 and command.position < self.max_opening * 0.5:
                self.grasp_strategy = "force"
            else:
                self.grasp_strategy = "position"
            
            self.get_logger().info(f"Executing {self.grasp_strategy} grasp - Position: {self.target_position:.3f}m, Force: {self.target_force:.1f}N")
            
            # Execute command with monitoring
            success = await self._execute_grasp_command(goal_handle)
            
            # Prepare result
            result = GripperCommand.Result()
            result.position = self.gripper_position
            result.effort = self.gripper_force
            result.stalled = (self.grasp_status == GraspStatus.FAILED)
            result.reached_goal = success
            
            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"Gripper command execution failed: {e}")
            goal_handle.abort()
            result = GripperCommand.Result()
            result.reached_goal = False
            result.stalled = True
            return result
        
        finally:
            self.command_active = False
    
    def _validate_gripper_command(self, command) -> bool:
        """Validate gripper command"""
        try:
            if command.position < 0.0 or command.position > self.max_opening:
                self.get_logger().error(f"Invalid gripper position: {command.position}")
                return False
            
            if command.max_effort < 0.0 or command.max_effort > self.max_force:
                self.get_logger().error(f"Invalid gripper force: {command.max_effort}")
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Gripper command validation failed: {e}")
            return False
    
    async def _execute_grasp_command(self, goal_handle) -> bool:
        """Execute grasp command with appropriate strategy"""
        try:
            if self.grasp_strategy == "force":
                return await self._execute_force_grasp(goal_handle)
            else:
                return await self._execute_position_grasp(goal_handle)
                
        except Exception as e:
            self.get_logger().error(f"Grasp execution failed: {e}")
            return False
    
    async def _execute_position_grasp(self, goal_handle) -> bool:
        """Execute position-based grasp"""
        timeout = 10.0  # seconds
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            
            # Check if position reached
            position_error = abs(self.target_position - self.gripper_position)
            if position_error < self.position_tolerance:
                self._assess_grasp_quality()
                return True
            
            # Publish feedback
            feedback = GripperCommand.Feedback()
            feedback.position = self.gripper_position
            feedback.effort = self.gripper_force
            feedback.stalled = False
            feedback.reached_goal = False
            goal_handle.publish_feedback(feedback)
            
            # Check for stall (object contact)
            if self._check_gripper_stall():
                self._assess_grasp_quality()
                return True
            
            await rclpy.task.sleep(0.01)  # 100 Hz update rate
        
        self.get_logger().warn("Position grasp timeout")
        self.grasp_status = GraspStatus.FAILED
        return False
    
    async def _execute_force_grasp(self, goal_handle) -> bool:
        """Execute force-based grasp with ramping"""
        timeout = 15.0  # seconds
        start_time = time.time()
        ramp_start_time = None
        
        # First, close to contact
        while time.time() - start_time < timeout * 0.5:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            
            # Check for object contact
            if self._check_object_contact():
                self.get_logger().info("Object contact detected, starting force ramp")
                ramp_start_time = time.time()
                break
            
            # Check if closed too far without contact
            if self.gripper_position <= 0.01:  # 1cm
                self.get_logger().warn("Gripper closed without object contact")
                self.grasp_status = GraspStatus.FAILED
                return False
            
            await rclpy.task.sleep(0.01)
        
        if ramp_start_time is None:
            self.get_logger().error("No object contact detected")
            self.grasp_status = GraspStatus.FAILED
            return False
        
        # Ramp force to target
        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            
            # Calculate desired force based on ramp
            ramp_time = time.time() - ramp_start_time
            desired_force = min(self.target_force, self.force_ramp_rate * ramp_time)
            
            # Check if target force reached
            force_error = abs(desired_force - self.gripper_force)
            if (desired_force >= self.target_force and 
                force_error < self.force_tolerance):
                
                # Wait for force to stabilize
                await rclpy.task.sleep(0.5)
                self._assess_grasp_quality()
                return True
            
            # Publish feedback
            feedback = GripperCommand.Feedback()
            feedback.position = self.gripper_position
            feedback.effort = self.gripper_force
            feedback.stalled = False
            feedback.reached_goal = False
            goal_handle.publish_feedback(feedback)
            
            # Check for slip
            if self._check_slip():
                self.get_logger().warn("Object slip detected during grasp")
                self.grasp_status = GraspStatus.SLIPPING
                # Continue trying to grasp
            
            await rclpy.task.sleep(0.01)
        
        self.get_logger().warn("Force grasp timeout")
        self.grasp_status = GraspStatus.FAILED
        return False
    
    def _check_object_contact(self) -> bool:
        """Check if gripper has made contact with object"""
        # Simple contact detection based on force threshold
        contact_force_threshold = 2.0  # N
        return self.gripper_force > contact_force_threshold
    
    def _check_gripper_stall(self) -> bool:
        """Check if gripper has stalled (hit obstacle)"""
        # Check if gripper stopped moving but command is still active
        velocity_threshold = 0.001  # m/s
        force_threshold = 5.0       # N
        
        return (abs(self.gripper_velocity) < velocity_threshold and 
                self.gripper_force > force_threshold)
    
    def _check_slip(self) -> bool:
        """Check for object slip based on force change rate"""
        if len(self.force_history) < 10:  # Need some history
            return False
        
        # Calculate force change rate over last 0.1 seconds
        recent_history = [(t, f) for t, f in self.force_history if t > time.time() - 0.1]
        
        if len(recent_history) < 5:
            return False
        
        # Simple linear regression to find force trend
        times = [t - recent_history[0][0] for t, f in recent_history]
        forces = [f for t, f in recent_history]
        
        if len(times) > 1:
            # Calculate slope (force change rate)
            n = len(times)
            sum_t = sum(times)
            sum_f = sum(forces)
            sum_tf = sum(t * f for t, f in zip(times, forces))
            sum_t2 = sum(t * t for t in times)
            
            if n * sum_t2 - sum_t * sum_t != 0:
                slope = (n * sum_tf - sum_t * sum_f) / (n * sum_t2 - sum_t * sum_t)
                
                # Negative slope indicates force loss (slip)
                return slope < -self.slip_threshold
        
        return False
    
    def _assess_grasp_quality(self):
        """Assess the quality of the current grasp"""
        try:
            # Force-based assessment
            if self.gripper_force < 1.0:
                self.grasp_status = GraspStatus.FAILED
                self.object_detected = False
            elif self.gripper_force > self.target_force * 0.8:
                if self._check_force_stability():
                    self.grasp_status = GraspStatus.SECURE
                    self.object_detected = True
                else:
                    self.grasp_status = GraspStatus.SOFT
                    self.object_detected = True
            else:
                self.grasp_status = GraspStatus.SOFT
                self.object_detected = True
            
            # Check for slip
            if self._check_slip():
                self.grasp_status = GraspStatus.SLIPPING
            
            self.get_logger().info(f"Grasp assessment: {self.grasp_status.value} - Force: {self.gripper_force:.1f}N")
            
        except Exception as e:
            self.get_logger().error(f"Grasp assessment failed: {e}")
            self.grasp_status = GraspStatus.UNKNOWN
    
    def _check_force_stability(self) -> bool:
        """Check if grasp force is stable"""
        if len(self.force_history) < 20:  # Need sufficient history
            return False
        
        # Check force variation over last 0.5 seconds
        recent_forces = [f for t, f in self.force_history if t > time.time() - 0.5]
        
        if len(recent_forces) < 10:
            return False
        
        # Calculate force stability (low variance indicates stable grasp)
        mean_force = np.mean(recent_forces)
        force_std = np.std(recent_forces)
        
        stability_threshold = 1.0  # N standard deviation
        return force_std < stability_threshold and mean_force > 2.0
    
    def _control_loop_callback(self):
        """Main gripper control loop"""
        try:
            if not self.command_active:
                return
            
            # Update position history
            self.position_history.append((time.time(), self.gripper_position))
            # Keep only recent history
            cutoff_time = time.time() - 1.0
            self.position_history = [(t, p) for t, p in self.position_history if t > cutoff_time]
            
            # TODO: Implement actual gripper control commands
            # This would send position/force commands to the gripper hardware
            
        except Exception as e:
            self.get_logger().error(f"Gripper control loop error: {e}")
    
    def _publish_status_callback(self):
        """Publish gripper status"""
        try:
            # Publish gripper joint state
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ["gripper_joint"]
            joint_state.position = [self.gripper_position]
            joint_state.velocity = [self.gripper_velocity]
            joint_state.effort = [self.gripper_force / 10.0]  # Convert force to effort
            
            self.gripper_status_pub.publish(joint_state)
            
            # Publish grasp status
            grasp_status = ArmStatus()
            grasp_status.header.stamp = self.get_clock().now().to_msg()
            grasp_status.header.frame_id = "gripper_base"
            
            # Set gripper-specific fields
            grasp_status.current_tool_name = "gripper"
            grasp_status.tool_attached = True
            grasp_status.gripper_opening = self.gripper_position
            grasp_status.gripper_force = self.gripper_force
            
            # Motion status
            grasp_status.in_motion = self.command_active
            
            # Additional status in error message field
            grasp_status.error_message = f"Grasp: {self.grasp_status.value}, Object: {self.object_detected}"
            
            self.grasp_status_pub.publish(grasp_status)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish gripper status: {e}")


def main():
    """Main entry point"""
    rclpy.init()
    
    try:
        executor = MultiThreadedExecutor()
        gripper_controller = GripperController()
        executor.add_node(gripper_controller)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Gripper controller failed: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main() 