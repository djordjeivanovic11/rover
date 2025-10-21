#!/usr/bin/env python3
"""
=============================================================================
SIMPLE GRIPPER CONTROLLER FOR URC ROVER
=============================================================================
Provides basic gripper control - open, close, and position control.
No complex force sensing - just reliable position control for URC missions.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer
import time


class GripperController(Node):
    """
    Simple gripper controller for URC missions.
    Provides open/close commands and position control.
    """
    
    def __init__(self):
        super().__init__('gripper_controller')
        
        # Gripper parameters (adjust for your gripper)
        self.fully_open_position = 0
        self.fully_closed_position = 1000
        self.current_position = 0
        
        # Publisher for gripper motor commands
        self.gripper_pub = self.create_publisher(
            Int32,
            '/gripper_command',
            10
        )
        
        # Subscriber for gripper feedback
        self.gripper_feedback_sub = self.create_subscription(
            Int32,
            '/gripper_position',
            self._gripper_feedback_callback,
            10
        )
        
        # Action server for GripperCommand action
        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            self._execute_gripper_command
        )
        
        # Simple service-like topics for quick commands
        self.open_sub = self.create_subscription(
            Float32,
            '/gripper/open',
            self._open_callback,
            10
        )
        
        self.close_sub = self.create_subscription(
            Float32,
            '/gripper/close',
            self._close_callback,
            10
        )
        
        self.position_sub = self.create_subscription(
            Int32,
            '/gripper/set_position',
            self._position_callback,
            10
        )
        
        self.get_logger().info('Gripper Controller initialized')
        self.get_logger().info('Use topics: /gripper/open, /gripper/close, /gripper/set_position')
    
    def _gripper_feedback_callback(self, msg):
        """Update current gripper position from feedback"""
        self.current_position = msg.data
    
    def _open_callback(self, msg):
        """Open gripper (0.0 = fully closed, 1.0 = fully open)"""
        percentage = max(0.0, min(1.0, msg.data))
        position = int(self.fully_open_position + 
                      (self.fully_closed_position - self.fully_open_position) * (1.0 - percentage))
        self._set_gripper_position(position)
        self.get_logger().info(f'Opening gripper to {percentage*100:.0f}%')
    
    def _close_callback(self, msg):
        """Close gripper (0.0 = no force, 1.0 = maximum grip)"""
        percentage = max(0.0, min(1.0, msg.data))
        position = int(self.fully_closed_position * percentage)
        self._set_gripper_position(position)
        self.get_logger().info(f'Closing gripper to {percentage*100:.0f}%')
    
    def _position_callback(self, msg):
        """Set gripper to absolute position"""
        position = max(self.fully_open_position, 
                      min(self.fully_closed_position, msg.data))
        self._set_gripper_position(position)
        self.get_logger().info(f'Setting gripper position to {position}')
    
    def _set_gripper_position(self, position):
        """Send position command to gripper"""
        msg = Int32()
        msg.data = int(position)
        self.gripper_pub.publish(msg)
    
    def _execute_gripper_command(self, goal_handle):
        """Execute GripperCommand action"""
        self.get_logger().info('Executing gripper command action...')
        
        request = goal_handle.request
        target_position = int((request.command.position / request.command.max_effort) * 
                             self.fully_closed_position)
        
        # Send command
        self._set_gripper_position(target_position)
        
        # Wait for gripper to reach position (simple timeout-based)
        start_time = time.time()
        timeout = 3.0  # seconds
        
        while time.time() - start_time < timeout:
            if abs(self.current_position - target_position) < 50:  # Within tolerance
                goal_handle.succeed()
                result = GripperCommand.Result()
                result.position = self.current_position
                result.reached_goal = True
                return result
            time.sleep(0.1)
        
        # Timeout - still return success (gripper may be stalled on object)
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = self.current_position
        result.reached_goal = False
        result.stalled = True
        return result
    
    def open_gripper(self):
        """Convenience method: fully open gripper"""
        self._set_gripper_position(self.fully_open_position)
    
    def close_gripper(self):
        """Convenience method: fully close gripper"""
        self._set_gripper_position(self.fully_closed_position)


def main(args=None):
    rclpy.init(args=args)
    controller = GripperController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
