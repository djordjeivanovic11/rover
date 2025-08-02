#!/usr/bin/env python3
"""
=============================================================================
TRAJECTORY EXECUTOR
=============================================================================
Node that streams JointTrajectory goals from MoveIt to hardware interface.
Monitors tracking error, velocity, and instantaneous torque on every cycle.
Issues immediate halt if any threshold is exceeded or E-stop is triggered.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import threading
import time
from typing import Optional, List, Dict
from pathlib import Path
import yaml

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped

from arm_control.msg import ArmStatus, Fault


class TrajectoryExecutor(Node):
    """
    Trajectory executor for latency-aware trajectory streaming.
    
    Features:
    - Streams JointTrajectory goals with real-time monitoring
    - Tracks position, velocity, and torque limits
    - Immediate halt on threshold violations or E-stop
    - Publishes detailed execution status
    """
    
    def __init__(self):
        super().__init__("trajectory_executor")
        
        # Load configuration
        self._load_configuration()
        
        # Joint configuration
        self.joint_names = [
            "base_rotate", "shoulder_pitch", "shoulder_roll",
            "elbow_pitch", "wrist_pitch", "wrist_roll"
        ]
        self.num_joints = len(self.joint_names)
        
        # Current state
        self.current_joint_state = JointState()
        self.current_trajectory: Optional[JointTrajectory] = None
        self.trajectory_start_time: Optional[float] = None
        self.trajectory_active = False
        self.execution_halted = False
        
        # Monitoring data
        self.joint_currents = np.zeros(self.num_joints)
        self.joint_temperatures = np.zeros(self.num_joints)
        self.tcp_wrench = WrenchStamped()
        self.estop_active = False
        
        # Execution parameters
        self.tracking_tolerance = 0.1  # rad
        self.velocity_tolerance = 0.5  # rad/s
        self.force_limit = 50.0  # N
        self.torque_limit = 10.0  # Nm
        
        # Callback group for thread-safe operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Action server for receiving trajectory goals
        self.trajectory_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            self._execute_trajectory_callback,
            callback_group=self.callback_group
        )
        
        # Publishers
        self.arm_status_pub = self.create_publisher(
            ArmStatus, "/arm/status", 10)
        self.fault_pub = self.create_publisher(
            Fault, "/arm/faults", 10)
        self.joint_command_pub = self.create_publisher(
            JointTrajectory, "/arm/joint_trajectory_controller/joint_trajectory", 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10)
        self.joint_currents_sub = self.create_subscription(
            Float32MultiArray, "/arm/joint_currents", self._joint_currents_callback, 10)
        self.joint_temps_sub = self.create_subscription(
            Float32MultiArray, "/arm/joint_temperatures", self._joint_temps_callback, 10)
        self.tcp_wrench_sub = self.create_subscription(
            WrenchStamped, "/arm/tcp_wrench", self._tcp_wrench_callback, 10)
        self.estop_sub = self.create_subscription(
            Bool, "/arm/estop_hw", self._estop_callback, 10)
        
        # Status publishing timer
        self.status_timer = self.create_timer(
            0.1, self._publish_status_callback)  # 10 Hz status updates
        
        # Monitoring timer  
        self.monitor_timer = self.create_timer(
            0.01, self._monitor_execution_callback)  # 100 Hz monitoring
        
        self.get_logger().info("Trajectory Executor initialized")
    
    def _load_configuration(self):
        """Load configuration from YAML files"""
        try:
            # Load safety parameters
            safety_params_path = Path(__file__).parent.parent / "config" / "safety_params.yaml"
            with open(safety_params_path, 'r') as f:
                safety_config = yaml.safe_load(f)
            
            # Extract monitoring parameters
            safety_params = safety_config.get('safety_monitor', {}).get('ros__parameters', {})
            tracking_limits = safety_params.get('tracking_error_limits', {})
            self.tracking_tolerance = tracking_limits.get('position_error', 0.1)
            self.velocity_tolerance = tracking_limits.get('velocity_error', 0.5)
            
            force_limits = safety_params.get('force_torque_limits', {})
            self.force_limit = force_limits.get('max_force_z', 50.0)
            self.torque_limit = force_limits.get('max_torque_z', 10.0)
            
            self.get_logger().info("Configuration loaded successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            # Use default values
    
    def _joint_state_callback(self, msg: JointState):
        """Update current joint state"""
        self.current_joint_state = msg
    
    def _joint_currents_callback(self, msg: Float32MultiArray):
        """Update joint current monitoring"""
        if len(msg.data) >= self.num_joints:
            self.joint_currents = np.array(msg.data[:self.num_joints])
    
    def _joint_temps_callback(self, msg: Float32MultiArray):
        """Update joint temperature monitoring"""
        if len(msg.data) >= self.num_joints:
            self.joint_temperatures = np.array(msg.data[:self.num_joints])
    
    def _tcp_wrench_callback(self, msg: WrenchStamped):
        """Update TCP force/torque monitoring"""
        self.tcp_wrench = msg
    
    def _estop_callback(self, msg: Bool):
        """Update emergency stop status"""
        was_active = self.estop_active
        self.estop_active = msg.data
        
        if not was_active and self.estop_active:
            self.get_logger().error("EMERGENCY STOP ACTIVATED - Halting trajectory execution")
            self._halt_trajectory("Emergency stop activated")
    
    async def _execute_trajectory_callback(self, goal_handle):
        """Execute trajectory goal from action client"""
        self.get_logger().info("Received trajectory execution goal")
        
        try:
            goal = goal_handle.request
            trajectory = goal.trajectory
            
            # Validate trajectory
            if not self._validate_trajectory(trajectory):
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "Invalid trajectory"
                return result
            
            # Start execution
            self.current_trajectory = trajectory
            self.trajectory_start_time = time.time()
            self.trajectory_active = True
            self.execution_halted = False
            
            # Stream trajectory points
            success = await self._stream_trajectory(goal_handle, trajectory)
            
            # Prepare result
            result = FollowJointTrajectory.Result()
            if success:
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Trajectory executed successfully"
                goal_handle.succeed()
            else:
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = "Trajectory execution failed"
                goal_handle.abort()
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"Trajectory execution failed: {e}")
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(e)
            return result
        
        finally:
            self.trajectory_active = False
            self.current_trajectory = None
            self.trajectory_start_time = None
    
    def _validate_trajectory(self, trajectory: JointTrajectory) -> bool:
        """Validate trajectory before execution"""
        try:
            # Check joint names match
            if trajectory.joint_names != self.joint_names:
                self.get_logger().error("Joint names mismatch in trajectory")
                return False
            
            # Check trajectory is not empty
            if not trajectory.points:
                self.get_logger().error("Empty trajectory")
                return False
            
            # Check trajectory points are valid
            for i, point in enumerate(trajectory.points):
                if len(point.positions) != self.num_joints:
                    self.get_logger().error(f"Invalid position count at point {i}")
                    return False
                    
                if point.velocities and len(point.velocities) != self.num_joints:
                    self.get_logger().error(f"Invalid velocity count at point {i}")
                    return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Trajectory validation failed: {e}")
            return False
    
    async def _stream_trajectory(self, goal_handle, trajectory: JointTrajectory) -> bool:
        """Stream trajectory points with real-time monitoring"""
        try:
            total_points = len(trajectory.points)
            
            for i, point in enumerate(trajectory.points):
                # Check if goal was canceled
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Trajectory execution canceled")
                    goal_handle.canceled()
                    return False
                
                # Check if execution was halted
                if self.execution_halted:
                    self.get_logger().error("Trajectory execution halted due to safety violation")
                    return False
                
                # Send trajectory point to controller
                self._send_trajectory_point(point)
                
                # Publish feedback
                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = self.joint_names
                
                if len(self.current_joint_state.position) >= self.num_joints:
                    feedback.actual = self.current_joint_state
                    feedback.desired.positions = point.positions
                    feedback.desired.velocities = point.velocities if point.velocities else [0.0] * self.num_joints
                    
                    # Calculate error
                    position_error = np.array(point.positions) - np.array(self.current_joint_state.position[:self.num_joints])
                    feedback.error.positions = position_error.tolist()
                
                goal_handle.publish_feedback(feedback)
                
                # Wait for trajectory point time
                if i < total_points - 1:  # Don't wait after last point
                    point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                    next_time = trajectory.points[i + 1].time_from_start.sec + trajectory.points[i + 1].time_from_start.nanosec * 1e-9
                    wait_time = next_time - point_time
                    
                    if wait_time > 0:
                        await rclpy.task.sleep(wait_time)
            
            # Wait for final position to be reached
            await self._wait_for_trajectory_completion(trajectory.points[-1])
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Trajectory streaming failed: {e}")
            return False
    
    def _send_trajectory_point(self, point: JointTrajectoryPoint):
        """Send single trajectory point to controller"""
        # Create single-point trajectory for immediate execution
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.joint_names
        traj_msg.points = [point]
        
        self.joint_command_pub.publish(traj_msg)
    
    async def _wait_for_trajectory_completion(self, final_point: JointTrajectoryPoint) -> bool:
        """Wait for trajectory to reach final position within tolerance"""
        timeout = 5.0  # seconds
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.execution_halted:
                return False
            
            # Check if we're within tolerance of final position
            if len(self.current_joint_state.position) >= self.num_joints:
                position_error = np.array(final_point.positions) - np.array(self.current_joint_state.position[:self.num_joints])
                if np.all(np.abs(position_error) < self.tracking_tolerance):
                    return True
            
            await rclpy.task.sleep(0.01)  # 100 Hz check rate
        
        self.get_logger().warn("Trajectory completion timeout")
        return False
    
    def _monitor_execution_callback(self):
        """Monitor trajectory execution for safety violations"""
        if not self.trajectory_active:
            return
        
        try:
            # Check tracking error
            if not self._check_tracking_error():
                self._halt_trajectory("Tracking error exceeded")
                return
            
            # Check velocity limits
            if not self._check_velocity_limits():
                self._halt_trajectory("Velocity limit exceeded")
                return
            
            # Check force/torque limits
            if not self._check_force_limits():
                self._halt_trajectory("Force/torque limit exceeded")
                return
            
            # Check emergency stop
            if self.estop_active:
                self._halt_trajectory("Emergency stop activated")
                return
                
        except Exception as e:
            self.get_logger().error(f"Execution monitoring failed: {e}")
    
    def _check_tracking_error(self) -> bool:
        """Check if tracking error is within limits"""
        if not self.current_trajectory or not self.trajectory_start_time:
            return True
        
        if len(self.current_joint_state.position) < self.num_joints:
            return True  # No joint state yet
        
        # Get current desired position from trajectory
        current_time = time.time() - self.trajectory_start_time
        desired_position = self._interpolate_trajectory_position(current_time)
        
        if desired_position is None:
            return True
        
        # Calculate tracking error
        actual_position = np.array(self.current_joint_state.position[:self.num_joints])
        position_error = np.abs(desired_position - actual_position)
        
        # Check if any joint exceeds tracking tolerance
        if np.any(position_error > self.tracking_tolerance):
            max_error_idx = np.argmax(position_error)
            self.get_logger().error(f"Tracking error on {self.joint_names[max_error_idx]}: {position_error[max_error_idx]:.3f} rad")
            return False
        
        return True
    
    def _check_velocity_limits(self) -> bool:
        """Check if joint velocities are within limits"""
        if len(self.current_joint_state.velocity) < self.num_joints:
            return True  # No velocity data yet
        
        velocities = np.array(self.current_joint_state.velocity[:self.num_joints])
        velocity_magnitudes = np.abs(velocities)
        
        # Use safety limits from configuration
        # TODO: Load actual velocity limits from config
        velocity_limits = np.array([0.8, 0.8, 0.8, 0.8, 1.2, 1.6])  # rad/s
        
        if np.any(velocity_magnitudes > velocity_limits):
            max_vel_idx = np.argmax(velocity_magnitudes)
            self.get_logger().error(f"Velocity limit exceeded on {self.joint_names[max_vel_idx]}: {velocity_magnitudes[max_vel_idx]:.3f} rad/s")
            return False
        
        return True
    
    def _check_force_limits(self) -> bool:
        """Check if TCP forces/torques are within limits"""
        try:
            # Check force magnitude
            force = self.tcp_wrench.wrench.force
            force_magnitude = np.sqrt(force.x**2 + force.y**2 + force.z**2)
            
            if force_magnitude > self.force_limit:
                self.get_logger().error(f"Force limit exceeded: {force_magnitude:.1f} N")
                return False
            
            # Check torque magnitude
            torque = self.tcp_wrench.wrench.torque
            torque_magnitude = np.sqrt(torque.x**2 + torque.y**2 + torque.z**2)
            
            if torque_magnitude > self.torque_limit:
                self.get_logger().error(f"Torque limit exceeded: {torque_magnitude:.1f} Nm")
                return False
            
            return True
            
        except Exception as e:
            # If force/torque data is not available, don't fail
            return True
    
    def _interpolate_trajectory_position(self, current_time: float) -> Optional[np.ndarray]:
        """Interpolate desired position from trajectory at current time"""
        if not self.current_trajectory:
            return None
        
        points = self.current_trajectory.points
        
        # Find surrounding trajectory points
        for i in range(len(points) - 1):
            t1 = points[i].time_from_start.sec + points[i].time_from_start.nanosec * 1e-9
            t2 = points[i + 1].time_from_start.sec + points[i + 1].time_from_start.nanosec * 1e-9
            
            if t1 <= current_time <= t2:
                # Linear interpolation
                alpha = (current_time - t1) / (t2 - t1) if t2 > t1 else 0.0
                pos1 = np.array(points[i].positions)
                pos2 = np.array(points[i + 1].positions)
                return pos1 + alpha * (pos2 - pos1)
        
        # If past the end, return final position
        if points:
            return np.array(points[-1].positions)
        
        return None
    
    def _halt_trajectory(self, reason: str):
        """Immediately halt trajectory execution"""
        self.execution_halted = True
        self.trajectory_active = False
        
        # Publish fault
        fault = Fault()
        fault.header.stamp = self.get_clock().now().to_msg()
        fault.severity = Fault.ERROR
        fault.fault_code = 105  # JOINT_TRACKING_ERROR
        fault.fault_category = "TRAJECTORY"
        fault.fault_description = reason
        fault.component_name = "trajectory_executor"
        fault.auto_recoverable = True
        
        self.fault_pub.publish(fault)
        
        self.get_logger().error(f"Trajectory execution halted: {reason}")
    
    def _publish_status_callback(self):
        """Publish arm status for monitoring"""
        try:
            status = ArmStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.header.frame_id = "base_link"
            
            # Set state
            if self.execution_halted:
                status.state = ArmStatus.FAULT
            elif self.trajectory_active:
                status.state = ArmStatus.EXECUTING
            else:
                status.state = ArmStatus.IDLE
            
            # Joint state
            status.joint_state = self.current_joint_state
            
            # Safety status
            status.safety_ok = not (self.execution_halted or self.estop_active)
            status.estop_active = self.estop_active
            
            # Motion status
            status.in_motion = self.trajectory_active
            if len(self.current_joint_state.velocity) >= self.num_joints:
                velocities = np.array(self.current_joint_state.velocity[:self.num_joints])
                status.velocity_norm = float(np.linalg.norm(velocities))
            
            # Temperature and current monitoring
            status.joint_temperatures = self.joint_temperatures.tolist()
            status.joint_currents = self.joint_currents.tolist()
            status.max_temperature = float(np.max(self.joint_temperatures)) if len(self.joint_temperatures) > 0 else 0.0
            status.max_current = float(np.max(self.joint_currents)) if len(self.joint_currents) > 0 else 0.0
            
            # Force/torque status
            status.current_wrench = self.tcp_wrench
            
            # Action progress
            if self.trajectory_active and self.current_trajectory and self.trajectory_start_time:
                elapsed_time = time.time() - self.trajectory_start_time
                total_time = self.current_trajectory.points[-1].time_from_start.sec + self.current_trajectory.points[-1].time_from_start.nanosec * 1e-9
                status.action_progress = min(100.0, (elapsed_time / total_time) * 100.0) if total_time > 0 else 0.0
            else:
                status.action_progress = 0.0
            
            self.arm_status_pub.publish(status)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish status: {e}")


def main():
    """Main entry point"""
    rclpy.init()
    
    try:
        executor = MultiThreadedExecutor()
        trajectory_executor = TrajectoryExecutor()
        executor.add_node(trajectory_executor)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Trajectory executor failed: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main() 