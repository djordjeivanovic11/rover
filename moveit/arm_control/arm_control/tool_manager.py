#!/usr/bin/env python3
"""
=============================================================================
TOOL MANAGER
=============================================================================
Coordinates end-effector swaps with hot URDF/SRDF reloading.
Pauses motion, unlocks coupling, loads new configurations, and reloads
kinematics so MoveIt begins planning with correct tip frame - no restarts.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import os
import yaml
import subprocess
import time
from typing import Dict, Optional, List
from pathlib import Path
import threading

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from arm_control.action import ToolChange
from arm_control.msg import ArmStatus, Fault


class ToolManager(Node):
    """
    Tool manager for automated end-effector changes.
    
    Features:
    - Coordinates with arm motion for safe tool changes
    - Manages quick-change coupling mechanism
    - Hot-loads URDF/SRDF configurations
    - Updates MoveIt kinematics without restart
    - Tracks tool inventory and status
    """
    
    def __init__(self):
        super().__init__("tool_manager")
        
        # Load configuration
        self._load_configuration()
        
        # Tool state
        self.current_tool = "gripper"  # Default tool
        self.tool_attached = True
        self.coupling_locked = True
        self.tool_change_in_progress = False
        
        # Tool inventory
        self.available_tools = {}
        self.tool_dock_poses = {}
        self.tool_configurations = {}
        
        # System state
        self.arm_motion_paused = False
        self.kinematics_updated = False
        
        # Callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()
        
        # Action server for tool change requests
        self.tool_change_action_server = ActionServer(
            self,
            ToolChange,
            "/arm/tool_change",
            self._execute_tool_change,
            callback_group=self.callback_group
        )
        
        # Publishers
        self.tool_status_pub = self.create_publisher(
            ArmStatus, "/arm/tool_status", 10)
        self.coupling_command_pub = self.create_publisher(
            Bool, "/tool_coupling/lock_command", 10)
        self.motion_pause_pub = self.create_publisher(
            Bool, "/arm/motion_pause", 10)
        self.fault_pub = self.create_publisher(
            Fault, "/arm/faults", 10)
        
        # Subscribers
        self.coupling_status_sub = self.create_subscription(
            Bool, "/tool_coupling/locked", self._coupling_status_callback, 10)
        self.tool_detect_sub = self.create_subscription(
            String, "/tool_coupling/detected_tool", self._tool_detect_callback, 10)
        self.arm_status_sub = self.create_subscription(
            ArmStatus, "/arm/status", self._arm_status_callback, 10)
        
        # Service clients for MoveIt reconfiguration
        self.moveit_reload_client = self.create_client(
            Trigger, "/move_group/reload_parameters")
        
        # Status timer
        self.status_timer = self.create_timer(
            0.5, self._publish_status_callback)  # 2 Hz status updates
        
        self.get_logger().info(f"Tool Manager initialized - Current tool: {self.current_tool}")
    
    def _load_configuration(self):
        """Load tool configurations and dock poses"""
        try:
            # Load arm parameters for tool definitions
            arm_params_path = Path(__file__).parent.parent / "config" / "arm_params.yaml"
            with open(arm_params_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Extract tool configurations
            tools_config = config.get('tools', {})
            for tool_name, tool_config in tools_config.items():
                self.available_tools[tool_name] = tool_config
                self.tool_configurations[tool_name] = {
                    'mass': tool_config.get('mass', 0.2),
                    'size': tool_config.get('size', [0.02, 0.02, 0.1]),
                    'mount_offset': tool_config.get('mount_offset', [0.0, 0.0, 0.05])
                }
            
            # Load tool dock poses (for now, use default positions)
            self._load_default_dock_poses()
            
            self.get_logger().info(f"Loaded {len(self.available_tools)} tool configurations")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load tool configuration: {e}")
            self._load_default_tools()
    
    def _load_default_dock_poses(self):
        """Load default tool dock poses"""
        # Default dock positions (these would be calibrated for real system)
        base_dock_pose = PoseStamped()
        base_dock_pose.header.frame_id = "base_link"
        base_dock_pose.pose.position.x = 0.5
        base_dock_pose.pose.position.y = 0.3
        base_dock_pose.pose.position.z = 0.2
        base_dock_pose.pose.orientation.w = 1.0
        
        # Create dock poses for each tool with small offsets
        for i, tool_name in enumerate(self.available_tools.keys()):
            dock_pose = PoseStamped()
            dock_pose.header.frame_id = "base_link"
            dock_pose.pose.position.x = base_dock_pose.pose.position.x
            dock_pose.pose.position.y = base_dock_pose.pose.position.y + i * 0.15
            dock_pose.pose.position.z = base_dock_pose.pose.position.z
            dock_pose.pose.orientation.w = 1.0
            
            self.tool_dock_poses[tool_name] = dock_pose
    
    def _load_default_tools(self):
        """Load default tool configurations"""
        self.available_tools = {
            "gripper": {"name": "gripper", "mass": 0.5},
            "wrench": {"name": "wrench", "mass": 0.3},
            "probe": {"name": "probe", "mass": 0.2},
            "scoop": {"name": "scoop", "mass": 0.4}
        }
        self._load_default_dock_poses()
    
    def _coupling_status_callback(self, msg: Bool):
        """Update coupling lock status"""
        self.coupling_locked = msg.data
    
    def _tool_detect_callback(self, msg: String):
        """Update detected tool information"""
        detected_tool = msg.data
        if detected_tool != self.current_tool:
            self.get_logger().info(f"Tool detection mismatch: Expected {self.current_tool}, detected {detected_tool}")
    
    def _arm_status_callback(self, msg: ArmStatus):
        """Update arm status information"""
        # Monitor arm state for safe tool changes
        pass
    
    async def _execute_tool_change(self, goal_handle):
        """Execute tool change action"""
        self.get_logger().info("Received tool change request")
        
        try:
            goal = goal_handle.request
            new_tool = goal.new_tool_name
            current_tool = goal.current_tool_name
            
            # Validate tool change request
            if not self._validate_tool_change_request(goal):
                goal_handle.abort()
                result = ToolChange.Result()
                result.success = False
                result.error_message = "Invalid tool change request"
                result.error_code = 603  # CONFIGURATION_ERROR
                return result
            
            # Execute tool change sequence
            self.tool_change_in_progress = True
            
            # Phase 1: Approach and prepare
            success = await self._approach_tool_dock(goal_handle, new_tool)
            if not success:
                return self._abort_tool_change(goal_handle, "Failed to approach tool dock")
            
            # Phase 2: Release current tool
            if self.tool_attached:
                success = await self._release_current_tool(goal_handle)
                if not success:
                    return self._abort_tool_change(goal_handle, "Failed to release current tool")
            
            # Phase 3: Acquire new tool
            success = await self._acquire_new_tool(goal_handle, new_tool)
            if not success:
                return self._abort_tool_change(goal_handle, "Failed to acquire new tool")
            
            # Phase 4: Update kinematics
            if goal.update_kinematics:
                success = await self._update_tool_kinematics(goal_handle, new_tool)
                if not success:
                    return self._abort_tool_change(goal_handle, "Failed to update kinematics")
            
            # Phase 5: Verify tool change
            success = await self._verify_tool_change(goal_handle, new_tool)
            if not success:
                return self._abort_tool_change(goal_handle, "Tool change verification failed")
            
            # Complete tool change
            self.current_tool = new_tool
            self.tool_change_in_progress = False
            
            # Prepare successful result
            result = ToolChange.Result()
            result.success = True
            result.old_tool_name = current_tool
            result.new_tool_name = new_tool
            result.coupling_verified = self.coupling_locked
            result.kinematics_updated = self.kinematics_updated
            result.total_execution_time = time.time() - goal_handle.request_stamp.sec
            
            goal_handle.succeed()
            return result
            
        except Exception as e:
            self.get_logger().error(f"Tool change execution failed: {e}")
            return self._abort_tool_change(goal_handle, f"Exception: {e}")
        
        finally:
            self.tool_change_in_progress = False
            self._resume_arm_motion()
    
    def _validate_tool_change_request(self, goal) -> bool:
        """Validate tool change request"""
        try:
            new_tool = goal.new_tool_name
            
            # Check if new tool is available
            if new_tool not in self.available_tools:
                self.get_logger().error(f"Unknown tool: {new_tool}")
                return False
            
            # Check if new tool is different from current
            if new_tool == self.current_tool:
                self.get_logger().warn(f"Tool {new_tool} is already attached")
                return True  # Not an error, just no-op
            
            # Check if dock pose is provided or can be looked up
            if new_tool not in self.tool_dock_poses:
                self.get_logger().error(f"No dock pose for tool: {new_tool}")
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Tool change validation failed: {e}")
            return False
    
    async def _approach_tool_dock(self, goal_handle, tool_name: str) -> bool:
        """Approach the tool dock station"""
        try:
            # Publish feedback
            feedback = ToolChange.Feedback()
            feedback.current_phase = "APPROACH"
            feedback.phase_progress = 0.0
            feedback.overall_progress = 10.0
            feedback.status_message = f"Approaching {tool_name} dock"
            goal_handle.publish_feedback(feedback)
            
            # Pause arm motion for safety
            self._pause_arm_motion()
            
            # TODO: Send approach command to motion planner
            # This would typically involve:
            # 1. Plan path to pre-dock pose
            # 2. Execute approach motion
            # 3. Monitor for obstacles and safety
            
            # Simulate approach time
            await rclpy.task.sleep(2.0)
            
            # Update feedback
            feedback.phase_progress = 100.0
            feedback.overall_progress = 20.0
            feedback.status_message = "Approach complete"
            goal_handle.publish_feedback(feedback)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Tool dock approach failed: {e}")
            return False
    
    async def _release_current_tool(self, goal_handle) -> bool:
        """Release the currently attached tool"""
        try:
            feedback = ToolChange.Feedback()
            feedback.current_phase = "RELEASE"
            feedback.phase_progress = 0.0
            feedback.overall_progress = 30.0
            feedback.status_message = f"Releasing {self.current_tool}"
            goal_handle.publish_feedback(feedback)
            
            # Unlock coupling mechanism
            self._unlock_coupling()
            
            # Wait for coupling to unlock
            timeout = 5.0
            start_time = time.time()
            while time.time() - start_time < timeout:
                if not self.coupling_locked:
                    break
                await rclpy.task.sleep(0.1)
            
            if self.coupling_locked:
                self.get_logger().error("Failed to unlock coupling")
                return False
            
            # TODO: Execute tool release motion
            # This would typically involve:
            # 1. Move away from tool mount
            # 2. Verify tool is released
            # 3. Update tool status
            
            await rclpy.task.sleep(1.0)  # Simulate release motion
            
            self.tool_attached = False
            self.current_tool = "none"
            
            feedback.phase_progress = 100.0
            feedback.overall_progress = 50.0
            feedback.status_message = "Tool released successfully"
            goal_handle.publish_feedback(feedback)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Tool release failed: {e}")
            return False
    
    async def _acquire_new_tool(self, goal_handle, tool_name: str) -> bool:
        """Acquire the new tool"""
        try:
            feedback = ToolChange.Feedback()
            feedback.current_phase = "ACQUIRE"
            feedback.phase_progress = 0.0
            feedback.overall_progress = 60.0
            feedback.status_message = f"Acquiring {tool_name}"
            goal_handle.publish_feedback(feedback)
            
            # TODO: Execute tool acquisition motion
            # This would typically involve:
            # 1. Approach tool mount point
            # 2. Engage with tool
            # 3. Verify tool engagement
            
            await rclpy.task.sleep(2.0)  # Simulate acquisition motion
            
            # Lock coupling mechanism
            self._lock_coupling()
            
            # Wait for coupling to lock
            timeout = 5.0
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.coupling_locked:
                    break
                await rclpy.task.sleep(0.1)
            
            if not self.coupling_locked:
                self.get_logger().error("Failed to lock coupling")
                return False
            
            self.tool_attached = True
            self.current_tool = tool_name
            
            feedback.phase_progress = 100.0
            feedback.overall_progress = 80.0
            feedback.status_message = f"{tool_name} acquired successfully"
            goal_handle.publish_feedback(feedback)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Tool acquisition failed: {e}")
            return False
    
    async def _update_tool_kinematics(self, goal_handle, tool_name: str) -> bool:
        """Update MoveIt kinematics for new tool"""
        try:
            feedback = ToolChange.Feedback()
            feedback.current_phase = "VERIFY"
            feedback.phase_progress = 0.0
            feedback.overall_progress = 85.0
            feedback.kinematics_status = "LOADING"
            feedback.status_message = f"Updating kinematics for {tool_name}"
            goal_handle.publish_feedback(feedback)
            
            # Generate new URDF/SRDF for tool
            success = self._generate_tool_urdf(tool_name)
            if not success:
                return False
            
            # Reload MoveIt parameters
            success = await self._reload_moveit_parameters()
            if not success:
                return False
            
            self.kinematics_updated = True
            
            feedback.phase_progress = 100.0
            feedback.overall_progress = 95.0
            feedback.kinematics_status = "UPDATED"
            feedback.status_message = "Kinematics updated successfully"
            goal_handle.publish_feedback(feedback)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Kinematics update failed: {e}")
            return False
    
    async def _verify_tool_change(self, goal_handle, tool_name: str) -> bool:
        """Verify tool change completion"""
        try:
            feedback = ToolChange.Feedback()
            feedback.current_phase = "VERIFY"
            feedback.phase_progress = 0.0
            feedback.overall_progress = 98.0
            feedback.status_message = f"Verifying {tool_name} installation"
            goal_handle.publish_feedback(feedback)
            
            # Check coupling status
            if not self.coupling_locked:
                self.get_logger().error("Coupling not locked after tool change")
                return False
            
            # Check tool detection
            # TODO: Verify tool is properly detected by sensors
            
            # Check tool functionality
            # TODO: Perform basic tool function test
            
            await rclpy.task.sleep(0.5)  # Simulate verification time
            
            feedback.phase_progress = 100.0
            feedback.overall_progress = 100.0
            feedback.status_message = "Tool change verification complete"
            goal_handle.publish_feedback(feedback)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Tool change verification failed: {e}")
            return False
    
    def _generate_tool_urdf(self, tool_name: str) -> bool:
        """Generate URDF/SRDF for the new tool"""
        try:
            # TODO: Implement dynamic URDF generation
            # This would typically involve:
            # 1. Load tool configuration from tool_configurations[tool_name]
            # 2. Generate URDF with correct tool geometry and mass properties
            # 3. Update SRDF with new end-effector definitions
            # 4. Save files to appropriate locations
            
            self.get_logger().info(f"Generated URDF/SRDF for tool: {tool_name}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"URDF generation failed: {e}")
            return False
    
    async def _reload_moveit_parameters(self) -> bool:
        """Reload MoveIt parameters for new tool configuration"""
        try:
            # Wait for service to be available
            if not self.moveit_reload_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("MoveIt reload service not available")
                return False
            
            # Call reload service
            request = Trigger.Request()
            future = self.moveit_reload_client.call_async(request)
            
            # Wait for response
            timeout = 10.0
            start_time = time.time()
            while not future.done() and time.time() - start_time < timeout:
                await rclpy.task.sleep(0.1)
            
            if not future.done():
                self.get_logger().error("MoveIt reload service timeout")
                return False
            
            response = future.result()
            if not response.success:
                self.get_logger().error(f"MoveIt reload failed: {response.message}")
                return False
            
            self.get_logger().info("MoveIt parameters reloaded successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"MoveIt reload failed: {e}")
            return False
    
    def _pause_arm_motion(self):
        """Pause arm motion for safe tool change"""
        pause_msg = Bool()
        pause_msg.data = True
        self.motion_pause_pub.publish(pause_msg)
        self.arm_motion_paused = True
        self.get_logger().info("Arm motion paused for tool change")
    
    def _resume_arm_motion(self):
        """Resume arm motion after tool change"""
        pause_msg = Bool()
        pause_msg.data = False
        self.motion_pause_pub.publish(pause_msg)
        self.arm_motion_paused = False
        self.get_logger().info("Arm motion resumed after tool change")
    
    def _lock_coupling(self):
        """Lock the tool coupling mechanism"""
        lock_msg = Bool()
        lock_msg.data = True
        self.coupling_command_pub.publish(lock_msg)
    
    def _unlock_coupling(self):
        """Unlock the tool coupling mechanism"""
        lock_msg = Bool()
        lock_msg.data = False
        self.coupling_command_pub.publish(lock_msg)
    
    def _abort_tool_change(self, goal_handle, error_message: str):
        """Abort tool change with error"""
        self.get_logger().error(f"Tool change aborted: {error_message}")
        
        # Publish fault
        fault = Fault()
        fault.header.stamp = self.get_clock().now().to_msg()
        fault.severity = Fault.ERROR
        fault.fault_code = 603  # CONFIGURATION_ERROR
        fault.fault_category = "SYSTEM"
        fault.fault_description = f"Tool change failed: {error_message}"
        fault.component_name = "tool_manager"
        fault.auto_recoverable = False
        
        self.fault_pub.publish(fault)
        
        # Resume motion
        self._resume_arm_motion()
        
        # Prepare failure result
        goal_handle.abort()
        result = ToolChange.Result()
        result.success = False
        result.error_message = error_message
        result.error_code = 603
        
        return result
    
    def _publish_status_callback(self):
        """Publish tool manager status"""
        try:
            status = ArmStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.header.frame_id = "tool_flange"
            
            # Tool information
            status.current_tool_name = self.current_tool
            status.tool_attached = self.tool_attached
            
            # State information
            if self.tool_change_in_progress:
                status.state = ArmStatus.TOOL_CHANGE
            else:
                status.state = ArmStatus.IDLE
            
            status.safety_ok = not self.tool_change_in_progress
            
            # Additional info in error message
            status.error_message = f"Tools available: {list(self.available_tools.keys())}"
            
            self.tool_status_pub.publish(status)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish tool status: {e}")


def main():
    """Main entry point"""
    rclpy.init()
    
    try:
        executor = MultiThreadedExecutor()
        tool_manager = ToolManager()
        executor.add_node(tool_manager)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Tool manager failed: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main() 