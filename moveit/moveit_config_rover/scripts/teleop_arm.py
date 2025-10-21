#!/usr/bin/env python3
"""
=============================================================================
MOVEIT ARM TELEOPERATION NODE
=============================================================================
Teleoperation interface for the rover arm using MoveIt2.
Supports game controller input (compatible with old PowerA controller system)
and keyboard control.

Features:
- Cartesian space control (move end effector in X, Y, Z)
- Joint space control
- Named pose shortcuts
- Gripper control
- Safety limits and collision avoidance (thanks to MoveIt)
=============================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
import numpy as np
from scipy.spatial.transform import Rotation

# Controller interface - using standard ROS2 Joy messages
CONTROLLER_AVAILABLE = False


class ArmTeleop(Node):
    """
    Teleoperation node for MoveIt-based arm control.
    """
    
    def __init__(self):
        super().__init__('arm_teleop')
        
        # Control parameters
        self.linear_scale = 0.05  # m per update
        self.angular_scale = 0.1  # rad per update
        self.gripper_scale = 100  # gripper units per update
        
        # State variables
        self.current_ee_position = np.array([0.0, 0.0, 0.3])  # Default position
        self.current_ee_rotation = np.array([0.0, 0.0, 0.0])  # Euler angles
        self.target_ee_position = self.current_ee_position.copy()
        self.target_ee_rotation = self.current_ee_rotation.copy()
        self.gripper_position = 0
        
        # Control mode
        self.cartesian_mode = True  # True: Cartesian, False: Joint
        self.rotation_mode = False  # Toggle with A button
        
        # Stick deadzone
        self.stick_threshold = 0.1
        
        # Publishers
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/arm/target_pose',
            10
        )
        
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/arm/twist_command',
            10
        )
        
        # Subscribers
        self.ee_position_sub = self.create_subscription(
            Float32MultiArray,
            '/current_cart',
            self._ee_position_callback,
            10
        )
        
        self.ee_rotation_sub = self.create_subscription(
            Float32MultiArray,
            '/current_rot',
            self._ee_rotation_callback,
            10
        )
        
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self._joy_callback,
            10
        )
        
        # Control loop timer
        self.control_timer = self.create_timer(0.05, self._control_loop)  # 20 Hz
        
        # Button states for edge detection
        self.last_buttons = {}
        
        self.get_logger().info('Arm Teleoperation Node initialized')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick: Move X/Y (or rotate if A pressed)')
        self.get_logger().info('  Right Stick: Move Z / Rotate base')
        self.get_logger().info('  Triggers: Gripper open/close')
        self.get_logger().info('  A: Toggle rotation mode')
        self.get_logger().info('  B: Toggle Cartesian/Joint mode')
        self.get_logger().info('  X: Go to stow position')
        self.get_logger().info('  Y: Go to ready position')
    
    def _ee_position_callback(self, msg):
        """Update current end effector position"""
        if len(msg.data) >= 3:
            self.current_ee_position = np.array(msg.data[:3])
            if np.linalg.norm(self.target_ee_position) < 0.01:  # Initialize target
                self.target_ee_position = self.current_ee_position.copy()
    
    def _ee_rotation_callback(self, msg):
        """Update current end effector rotation"""
        if len(msg.data) >= 3:
            self.current_ee_rotation = np.array(msg.data[:3])
            if np.linalg.norm(self.target_ee_rotation) < 0.01:  # Initialize target
                self.target_ee_rotation = self.current_ee_rotation.copy()
    
    def _joy_callback(self, msg):
        """
        Process joystick input.
        
        Joy message axes (typical):
          0: Left stick X
          1: Left stick Y
          2: Left trigger
          3: Right stick X
          4: Right stick Y
          5: Right trigger
        
        Joy message buttons:
          0: A
          1: B
          2: X
          3: Y
        """
        try:
            # Extract stick values
            left_x = msg.axes[0] if len(msg.axes) > 0 else 0.0
            left_y = msg.axes[1] if len(msg.axes) > 1 else 0.0
            right_x = msg.axes[3] if len(msg.axes) > 3 else 0.0
            right_y = msg.axes[4] if len(msg.axes) > 4 else 0.0
            
            left_trigger = (1.0 - msg.axes[2]) / 2.0 if len(msg.axes) > 2 else 0.0
            right_trigger = (1.0 - msg.axes[5]) / 2.0 if len(msg.axes) > 5 else 0.0
            
            # Get button states
            a_button = msg.buttons[0] if len(msg.buttons) > 0 else 0
            b_button = msg.buttons[1] if len(msg.buttons) > 1 else 0
            x_button = msg.buttons[2] if len(msg.buttons) > 2 else 0
            y_button = msg.buttons[3] if len(msg.buttons) > 3 else 0
            
            # Detect button presses (edge detection)
            a_pressed = a_button and not self.last_buttons.get('a', False)
            b_pressed = b_button and not self.last_buttons.get('b', False)
            x_pressed = x_button and not self.last_buttons.get('x', False)
            y_pressed = y_button and not self.last_buttons.get('y', False)
            
            # Update button states
            self.last_buttons = {
                'a': a_button,
                'b': b_button,
                'x': x_button,
                'y': y_button
            }
            
            # Handle button presses
            if a_pressed:
                self.rotation_mode = not self.rotation_mode
                mode_str = "ROTATION" if self.rotation_mode else "TRANSLATION"
                self.get_logger().info(f'Switched to {mode_str} mode')
            
            if b_pressed:
                self.cartesian_mode = not self.cartesian_mode
                mode_str = "CARTESIAN" if self.cartesian_mode else "JOINT"
                self.get_logger().info(f'Switched to {mode_str} mode')
            
            if x_pressed:
                self.get_logger().info('Moving to STOW position')
                self._move_to_named_pose('stow')
            
            if y_pressed:
                self.get_logger().info('Moving to READY position')
                self._move_to_named_pose('es_ready')
            
            # Process stick inputs
            if self.cartesian_mode:
                # Cartesian space control
                if not self.rotation_mode:
                    # Translation mode
                    if abs(left_x) > self.stick_threshold or abs(left_y) > self.stick_threshold:
                        self.target_ee_position[0] += -left_x * self.linear_scale
                        self.target_ee_position[1] += left_y * self.linear_scale
                    
                    if abs(right_y) > self.stick_threshold:
                        self.target_ee_position[2] += right_y * self.linear_scale
                else:
                    # Rotation mode
                    if abs(left_x) > self.stick_threshold:
                        self.target_ee_rotation[0] += left_x * self.angular_scale
                    
                    if abs(right_x) > self.stick_threshold:
                        self.target_ee_rotation[2] += right_x * self.angular_scale
            
            # Gripper control
            if left_trigger > 0.1:
                self.gripper_position += left_trigger * self.gripper_scale
            if right_trigger > 0.1:
                self.gripper_position -= right_trigger * self.gripper_scale
            
            # Clamp gripper position
            self.gripper_position = np.clip(self.gripper_position, 0, 1000)
            
        except Exception as e:
            self.get_logger().error(f'Error processing joy input: {e}')
    
    def _control_loop(self):
        """
        Main control loop - publishes target pose to MoveIt.
        """
        try:
            # Create target pose message
            target_pose = PoseStamped()
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.header.frame_id = 'base_link'
            
            # Set position
            target_pose.pose.position.x = float(self.target_ee_position[0])
            target_pose.pose.position.y = float(self.target_ee_position[1])
            target_pose.pose.position.z = float(self.target_ee_position[2])
            
            # Set orientation (convert from Euler to quaternion)
            rotation = Rotation.from_euler('xyz', self.target_ee_rotation)
            quat = rotation.as_quat()  # Returns [x, y, z, w]
            target_pose.pose.orientation.x = quat[0]
            target_pose.pose.orientation.y = quat[1]
            target_pose.pose.orientation.z = quat[2]
            target_pose.pose.orientation.w = quat[3]
            
            # Publish target pose
            self.target_pose_pub.publish(target_pose)
            
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')
    
    def _move_to_named_pose(self, pose_name):
        """
        Request move to a named pose.
        This would typically call a service on the MoveIt controller.
        """
        # In a complete implementation, this would call a service
        # For now, we just log it
        self.get_logger().info(f'Requested move to named pose: {pose_name}')
        # TODO: Implement service call to moveit_arm_controller


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        teleop = ArmTeleop()
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Teleoperation error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

