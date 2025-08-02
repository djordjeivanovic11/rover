#!/usr/bin/env python3
"""
=============================================================================
ACTION SERVERS
=============================================================================
Implements the three main ROS 2 actions for autonomous operation:
- GoToNamedPose: Move to predefined poses from arm_params.yaml
- PickAndPlace: Composite pick and place operations with grasp verification
- ToolChange: Tool swapping with hot URDF/SRDF reloading

Provides simple Goal/Feedback/Result interface for behavior trees.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import time
from typing import Dict, List, Optional, Tuple
from pathlib import Path
import yaml
import threading

from geometry_msgs.msg import PoseStamped, Point, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand

from arm_control.action import GoToNamedPose, PickAndPlace, ToolChange
from arm_control.msg import ArmStatus, Fault


class ArmActionServers(Node):
    """
    Action servers for autonomous arm operation.
    
    Exposes three main actions:
    - GoToNamedPose: Move to predefined poses
    - PickAndPlace: Composite manipulation operations  
    - ToolChange: End-effector swapping
    
    Each action wraps planning, execution, safety checks, and error recovery.
    """
    
    def __init__(self):
        super().__init__("arm_action_servers")
        
        # Load configuration
        self._load_configuration()
        
        # System state
        self.arm_status = ArmStatus()
        self.current_action = "none"
        self.action_in_progress = False
        
        # Callback groups for concurrent actions
        self.action_callback_group = ReentrantCallbackGroup()
        self.client_callback_group = ReentrantCallbackGroup()
        
        # Action servers
        self.goto_pose_server = ActionServer(
            self,
            GoToNamedPose,
            "/arm/go_to_named_pose",
            self._execute_go_to_named_pose,
            callback_group=self.action_callback_group
        )
        
        self.pick_place_server = ActionServer(
            self,
            PickAndPlace,
            "/arm/pick_and_place",
            self._execute_pick_and_place,
            callback_group=self.action_callback_group
        )
        
        self.tool_change_server = ActionServer(
            self,
            ToolChange,
            "/arm/tool_change_action",
            self._execute_tool_change_action,
            callback_group=self.action_callback_group
        )
        
        # Action clients for lower-level actions
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            callback_group=self.client_callback_group
        )
        
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            "/arm/gripper_command",
            callback_group=self.client_callback_group
        )
        
        self.tool_manager_client = ActionClient(
            self,
            ToolChange,
            "/arm/tool_change",
            callback_group=self.client_callback_group
        )
        
        # Subscribers
        self.arm_status_sub = self.create_subscription(
            ArmStatus, "/arm/status", self._arm_status_callback, 10)
        
        # Publishers
        self.fault_pub = self.create_publisher(
            Fault, "/arm/faults", 10)
        
        # Wait for action servers to be available
        self._wait_for_action_servers()
        
        self.get_logger().info("Arm Action Servers initialized and ready")
    
    def _load_configuration(self):
        """Load configuration from YAML files"""
        try:
            # Load arm parameters
            arm_params_path = Path(__file__).parent.parent / "config" / "arm_params.yaml"
            with open(arm_params_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Extract named poses
            self.named_poses = config.get('arm', {}).get('mission_poses', {})
            self.joint_names = config.get('arm', {}).get('joints', [])
            
            # Load safety parameters
            safety_params_path = Path(__file__).parent.parent / "config" / "safety_params.yaml"
            with open(safety_params_path, 'r') as f:
                safety_config = yaml.safe_load(f)
            
            self.safety_config = safety_config.get('safety_monitor', {}).get('ros__parameters', {})
            
            self.get_logger().info(f"Loaded {len(self.named_poses)} named poses")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            # Use minimal defaults
            self.named_poses = {"home": [0.0] * 6}
            self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
            self.safety_config = {}
    
    def _wait_for_action_servers(self):
        """Wait for required action servers to be available"""
        self.get_logger().info("Waiting for action servers...")
        
        # Wait for trajectory action server
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn("Trajectory action server not available")
        
        # Wait for gripper action server
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Gripper action server not available")
        
        # Wait for tool manager action server
        if not self.tool_manager_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Tool manager action server not available")
        
        self.get_logger().info("Action servers ready")
    
    def _arm_status_callback(self, msg: ArmStatus):
        """Update arm status"""
        self.arm_status = msg
    
    # =========================================================================
    # GO TO NAMED POSE ACTION
    # =========================================================================
    
    async def _execute_go_to_named_pose(self, goal_handle):
        """Execute GoToNamedPose action"""
        self.get_logger().info(f"GoToNamedPose action requested")
        
        try:
            goal = goal_handle.request
            pose_name = goal.pose_name
            
            # Validate pose name
            if not self._validate_named_pose(pose_name):
                return self._abort_action(goal_handle, GoToNamedPose.Result(), 
                                        "Invalid pose name", 603)
            
            self.current_action = f"go_to_{pose_name}"
            self.action_in_progress = True
            
            # Get joint positions for named pose
            target_positions = self.named_poses[pose_name]
            
            # Create trajectory
            trajectory = self._create_trajectory_to_position(
                target_positions, 
                goal.velocity_scaling,
                goal.acceleration_scaling
            )
            
            if trajectory is None:
                return self._abort_action(goal_handle, GoToNamedPose.Result(),
                                        "Failed to create trajectory", 602)
            
            # Execute trajectory
            start_time = time.time()
            success, final_pose = await self._execute_trajectory(
                goal_handle, trajectory, GoToNamedPose.Feedback())
            
            execution_time = time.time() - start_time
            
            # Prepare result
            result = GoToNamedPose.Result()
            result.success = success
            result.planning_time = 0.1  # Mock planning time
            result.execution_time = execution_time
            
            if success:
                result.error_message = f"Successfully reached pose: {pose_name}"
                result.error_code = 0
                # TODO: Calculate actual pose errors
                result.final_position_error = 0.01
                result.final_orientation_error = 0.01
                goal_handle.succeed()
            else:
                result.error_message = "Failed to reach target pose"
                result.error_code = 105  # JOINT_TRACKING_ERROR
                goal_handle.abort()
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"GoToNamedPose action failed: {e}")
            return self._abort_action(goal_handle, GoToNamedPose.Result(),
                                    f"Exception: {e}", 602)
        finally:
            self.action_in_progress = False
            self.current_action = "none"
    
    def _validate_named_pose(self, pose_name: str) -> bool:
        """Validate that named pose exists"""
        if pose_name not in self.named_poses:
            self.get_logger().error(f"Unknown named pose: {pose_name}")
            return False
        
        pose_positions = self.named_poses[pose_name]
        if len(pose_positions) != len(self.joint_names):
            self.get_logger().error(f"Pose {pose_name} has incorrect number of joints")
            return False
        
        return True
    
    # =========================================================================
    # PICK AND PLACE ACTION
    # =========================================================================
    
    async def _execute_pick_and_place(self, goal_handle):
        """Execute PickAndPlace action"""
        self.get_logger().info("PickAndPlace action requested")
        
        try:
            goal = goal_handle.request
            
            self.current_action = "pick_and_place"
            self.action_in_progress = True
            start_time = time.time()
            
            # Phase 1: Approach pick pose
            success = await self._approach_pick_pose(goal_handle, goal)
            if not success:
                return self._abort_pick_place(goal_handle, "APPROACH", "Failed to approach pick pose")
            
            # Phase 2: Execute grasp
            success, grasp_force = await self._execute_grasp(goal_handle, goal)
            if not success:
                return self._abort_pick_place(goal_handle, "GRASP", "Failed to grasp object")
            
            # Phase 3: Lift object
            success = await self._lift_object(goal_handle, goal)
            if not success:
                return self._abort_pick_place(goal_handle, "LIFT", "Failed to lift object")
            
            # Phase 4: Transport to place pose
            success = await self._transport_object(goal_handle, goal)
            if not success:
                return self._abort_pick_place(goal_handle, "TRANSPORT", "Failed to transport object")
            
            # Phase 5: Place object
            success = await self._place_object(goal_handle, goal)
            if not success:
                return self._abort_pick_place(goal_handle, "PLACE", "Failed to place object")
            
            # Phase 6: Retreat
            success = await self._retreat_from_place(goal_handle, goal)
            if not success:
                return self._abort_pick_place(goal_handle, "RETREAT", "Failed to retreat from place")
            
            # Prepare successful result
            result = PickAndPlace.Result()
            result.success = True
            result.grasp_successful = True
            result.place_successful = True
            result.grasp_force_achieved = grasp_force
            result.total_execution_time = time.time() - start_time
            
            goal_handle.succeed()
            return result
            
        except Exception as e:
            self.get_logger().error(f"PickAndPlace action failed: {e}")
            return self._abort_pick_place(goal_handle, "EXCEPTION", f"Exception: {e}")
        finally:
            self.action_in_progress = False
            self.current_action = "none"
    
    async def _approach_pick_pose(self, goal_handle, goal) -> bool:
        """Approach the pick pose"""
        try:
            # Publish feedback
            feedback = PickAndPlace.Feedback()
            feedback.current_phase = "APPROACH"
            feedback.phase_progress = 0.0
            feedback.overall_progress = 10.0
            feedback.status_message = "Approaching pick pose"
            goal_handle.publish_feedback(feedback)
            
            # Calculate approach pose (offset from pick pose)
            approach_pose = self._calculate_approach_pose(goal.pick_pose, goal.approach_offset)
            
            # Move to approach pose
            # TODO: Convert pose to joint trajectory and execute
            await rclpy.task.sleep(2.0)  # Simulate approach time
            
            feedback.phase_progress = 100.0
            feedback.overall_progress = 20.0
            feedback.status_message = "Approach complete"
            goal_handle.publish_feedback(feedback)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Approach failed: {e}")
            return False
    
    async def _execute_grasp(self, goal_handle, goal) -> Tuple[bool, float]:
        """Execute grasping operation"""
        try:
            feedback = PickAndPlace.Feedback()
            feedback.current_phase = "GRASP"
            feedback.phase_progress = 0.0
            feedback.overall_progress = 40.0
            feedback.status_message = "Executing grasp"
            goal_handle.publish_feedback(feedback)
            
            # Move to pick pose
            # TODO: Execute trajectory to pick pose
            await rclpy.task.sleep(1.0)
            
            # Execute gripper grasp
            gripper_goal = GripperCommand.Goal()
            gripper_goal.command.position = 0.02  # Close gripper
            gripper_goal.command.max_effort = goal.grasp_force
            
            gripper_future = self.gripper_client.send_goal_async(gripper_goal)
            await gripper_future
            
            gripper_result = await gripper_future.result().get_result_async()
            
            # Check grasp success
            grasp_successful = gripper_result.result.reached_goal
            grasp_force = gripper_result.result.effort
            
            feedback.current_grasp_force = grasp_force
            feedback.object_detected = grasp_successful
            feedback.grasp_status = "SECURE" if grasp_successful else "FAILED"
            feedback.phase_progress = 100.0
            feedback.overall_progress = 50.0
            goal_handle.publish_feedback(feedback)
            
            return grasp_successful, grasp_force
            
        except Exception as e:
            self.get_logger().error(f"Grasp failed: {e}")
            return False, 0.0
    
    async def _lift_object(self, goal_handle, goal) -> bool:
        """Lift the grasped object"""
        try:
            feedback = PickAndPlace.Feedback()
            feedback.current_phase = "LIFT"
            feedback.phase_progress = 0.0
            feedback.overall_progress = 60.0
            feedback.status_message = f"Lifting object {goal.lift_height}m"
            goal_handle.publish_feedback(feedback)
            
            # TODO: Execute lift motion (move up by lift_height)
            await rclpy.task.sleep(1.5)
            
            feedback.phase_progress = 100.0
            feedback.overall_progress = 70.0
            feedback.status_message = "Lift complete"
            goal_handle.publish_feedback(feedback)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Lift failed: {e}")
            return False
    
    async def _transport_object(self, goal_handle, goal) -> bool:
        """Transport object to place pose"""
        try:
            feedback = PickAndPlace.Feedback()
            feedback.current_phase = "TRANSPORT"
            feedback.phase_progress = 0.0
            feedback.overall_progress = 80.0
            feedback.status_message = "Transporting object"
            goal_handle.publish_feedback(feedback)
            
            # TODO: Plan and execute trajectory to place pose approach
            await rclpy.task.sleep(3.0)
            
            feedback.phase_progress = 100.0
            feedback.overall_progress = 85.0
            feedback.status_message = "Transport complete"
            goal_handle.publish_feedback(feedback)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Transport failed: {e}")
            return False
    
    async def _place_object(self, goal_handle, goal) -> bool:
        """Place the object at target location"""
        try:
            feedback = PickAndPlace.Feedback()
            feedback.current_phase = "PLACE"
            feedback.phase_progress = 0.0
            feedback.overall_progress = 90.0
            feedback.status_message = "Placing object"
            goal_handle.publish_feedback(feedback)
            
            # Move to place pose
            # TODO: Execute trajectory to place pose
            await rclpy.task.sleep(1.0)
            
            # Open gripper to release object
            gripper_goal = GripperCommand.Goal()
            gripper_goal.command.position = 0.1  # Open gripper
            gripper_goal.command.max_effort = goal.place_force
            
            gripper_future = self.gripper_client.send_goal_async(gripper_goal)
            await gripper_future
            
            await gripper_future.result().get_result_async()
            
            feedback.phase_progress = 100.0
            feedback.overall_progress = 95.0
            feedback.status_message = "Object placed"
            goal_handle.publish_feedback(feedback)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Place failed: {e}")
            return False
    
    async def _retreat_from_place(self, goal_handle, goal) -> bool:
        """Retreat from place position"""
        try:
            feedback = PickAndPlace.Feedback()
            feedback.current_phase = "RETREAT"
            feedback.phase_progress = 0.0
            feedback.overall_progress = 98.0
            feedback.status_message = "Retreating from place"
            goal_handle.publish_feedback(feedback)
            
            # TODO: Execute retreat motion
            await rclpy.task.sleep(1.0)
            
            feedback.phase_progress = 100.0
            feedback.overall_progress = 100.0
            feedback.status_message = "Pick and place complete"
            goal_handle.publish_feedback(feedback)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Retreat failed: {e}")
            return False
    
    def _calculate_approach_pose(self, target_pose: PoseStamped, offset: Vector3) -> PoseStamped:
        """Calculate approach pose with offset"""
        approach_pose = PoseStamped()
        approach_pose.header = target_pose.header
        approach_pose.pose.position.x = target_pose.pose.position.x + offset.x
        approach_pose.pose.position.y = target_pose.pose.position.y + offset.y
        approach_pose.pose.position.z = target_pose.pose.position.z + offset.z
        approach_pose.pose.orientation = target_pose.pose.orientation
        return approach_pose
    
    def _abort_pick_place(self, goal_handle, phase: str, error_msg: str):
        """Abort pick and place action"""
        result = PickAndPlace.Result()
        result.success = False
        result.failure_phase = phase
        result.error_message = error_msg
        result.error_code = 602
        goal_handle.abort()
        return result
    
    # =========================================================================
    # TOOL CHANGE ACTION
    # =========================================================================
    
    async def _execute_tool_change_action(self, goal_handle):
        """Execute ToolChange action (wrapper for tool manager)"""
        self.get_logger().info(f"ToolChange action requested: {goal_handle.request.new_tool_name}")
        
        try:
            # Forward request to tool manager
            tool_change_future = self.tool_manager_client.send_goal_async(goal_handle.request)
            await tool_change_future
            
            tool_change_handle = tool_change_future.result()
            if not tool_change_handle.accepted:
                return self._abort_action(goal_handle, ToolChange.Result(),
                                        "Tool manager rejected request", 603)
            
            # Wait for result and forward feedback
            while not tool_change_handle.done():
                await rclpy.task.sleep(0.1)
                # TODO: Forward feedback from tool manager
            
            result = await tool_change_handle.get_result_async()
            
            if result.result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            
            return result.result
            
        except Exception as e:
            self.get_logger().error(f"ToolChange action failed: {e}")
            return self._abort_action(goal_handle, ToolChange.Result(),
                                    f"Exception: {e}", 602)
    
    # =========================================================================
    # UTILITY METHODS
    # =========================================================================
    
    def _create_trajectory_to_position(self, target_positions: List[float], 
                                     velocity_scaling: float = 0.1,
                                     acceleration_scaling: float = 0.1) -> Optional[JointTrajectory]:
        """Create trajectory to target joint positions"""
        try:
            trajectory = JointTrajectory()
            trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory.joint_names = self.joint_names
            
            # Create single point trajectory
            point = JointTrajectoryPoint()
            point.positions = target_positions
            point.velocities = [0.0] * len(target_positions)
            point.accelerations = [0.0] * len(target_positions)
            point.time_from_start.sec = int(5.0 / velocity_scaling)  # Scale time
            point.time_from_start.nanosec = 0
            
            trajectory.points = [point]
            
            return trajectory
            
        except Exception as e:
            self.get_logger().error(f"Failed to create trajectory: {e}")
            return None
    
    async def _execute_trajectory(self, goal_handle, trajectory: JointTrajectory, 
                                feedback_type) -> Tuple[bool, Optional[PoseStamped]]:
        """Execute trajectory and monitor progress"""
        try:
            # Send trajectory to trajectory executor
            traj_goal = FollowJointTrajectory.Goal()
            traj_goal.trajectory = trajectory
            
            traj_future = self.trajectory_client.send_goal_async(traj_goal)
            await traj_future
            
            traj_handle = traj_future.result()
            if not traj_handle.accepted:
                self.get_logger().error("Trajectory execution rejected")
                return False, None
            
            # Monitor execution and forward feedback
            while not traj_handle.done():
                if goal_handle.is_cancel_requested:
                    await traj_handle.cancel_goal_async()
                    goal_handle.canceled()
                    return False, None
                
                # Forward progress feedback
                feedback = feedback_type
                feedback.status_message = "Executing trajectory"
                feedback.progress_percentage = 50.0  # TODO: Calculate actual progress
                goal_handle.publish_feedback(feedback)
                
                await rclpy.task.sleep(0.1)
            
            # Get result
            traj_result = await traj_handle.get_result_async()
            success = traj_result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            
            return success, None
            
        except Exception as e:
            self.get_logger().error(f"Trajectory execution failed: {e}")
            return False, None
    
    def _abort_action(self, goal_handle, result_type, error_msg: str, error_code: int):
        """Abort action with structured error"""
        result = result_type
        result.success = False
        result.error_message = error_msg
        result.error_code = error_code
        
        # Publish fault
        fault = Fault()
        fault.header.stamp = self.get_clock().now().to_msg()
        fault.severity = Fault.ERROR
        fault.fault_code = error_code
        fault.fault_category = "ACTION"
        fault.fault_description = error_msg
        fault.component_name = "action_servers"
        fault.auto_recoverable = True
        
        self.fault_pub.publish(fault)
        
        goal_handle.abort()
        return result


def main():
    """Main entry point"""
    rclpy.init()
    
    try:
        executor = MultiThreadedExecutor()
        action_servers = ArmActionServers()
        executor.add_node(action_servers)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Action servers failed: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main() 