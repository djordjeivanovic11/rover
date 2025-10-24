#!/usr/bin/env python3
"""
=============================================================================
ROVER ARM HARDWARE BRIDGE - ADAPTED FROM OLD SYSTEM
=============================================================================
Exactly matches the old ros_arm_interface.py behavior for hardware compatibility.

Topics (same as old system):
- Subscribes: /get_arm_position (Int32MultiArray) - encoder ticks from motors
- Publishes: /arm_target_motor_positions (Int32MultiArray) - motor commands in ticks

Bridge for ros2_control:
- Subscribes: /arm_joint_commands (JointState) - from ros2_control
- Publishes: /arm_joint_states (JointState) - to ros2_control
=============================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
import numpy as np

# Constants from old system (ros_arm_interface.py)
TICKS_PER_REV = 1600

# Gear ratios from old system - EXACTLY as they were
# Index: [0,      1,    2,    3,   4,         5]
# Joint: [azimuth, j1,  j2,   j3,  j4,        j5]
GEAR_RATIOS = np.array([50, 100, 100, 50, 50*22/9, 1])

# Joint mapping: MoveIt joint name → motor index
# This maps our 5 MoveIt joints to the 6 motor array
JOINT_TO_MOTOR_INDEX = {
    'AB_Rev': 0,   # Base/azimuth
    'AS1_Rev': 1,  # Shoulder joint 1
    'AS2_Rev': 2,  # Shoulder joint 2
    'AW_Rev': 3,   # Elbow
    'AM_Rev': 4,   # Wrist
}

JOINT_NAMES = ['AB_Rev', 'AS1_Rev', 'AS2_Rev', 'AW_Rev', 'AM_Rev']


class RoverArmHardwareBridge(Node):
    """Hardware bridge matching old system behavior"""
    
    def __init__(self):
        super().__init__('rover_arm_hardware_bridge')
        
        # Motor array (6 values as in old system, last one unused)
        self.motor_positions_ticks = np.zeros(6, dtype=int)
        self.motor_commands_ticks = np.zeros(6, dtype=int)
        
        # Joint positions in radians (5 joints for MoveIt)
        self.joint_positions = np.zeros(5)
        self.joint_velocities = np.zeros(5)
        
        # === OLD SYSTEM TOPICS (to motor firmware) ===
        
        # Subscribe to encoder feedback from motors
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/get_arm_position',
            self.encoder_callback,
            10
        )
        
        # Publish motor commands to firmware
        self.motor_cmd_pub = self.create_publisher(
            Int32MultiArray,
            '/arm_target_motor_positions',
            10
        )
        
        # === NEW SYSTEM TOPICS (to/from ros2_control) ===
        # Subscribe to commands from ros2_control
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            '/arm_joint_commands',
            self.joint_command_callback,
            10
        )
        
        # Publish joint states to ros2_control
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/arm_joint_states',
            10
        )
        
        # Publish joint states at 100Hz
        self.state_timer = self.create_timer(0.01, self.publish_joint_states)
        
        # For velocity estimation
        self.last_time = self.get_clock().now()
        self.last_positions = np.zeros(5)
        
        self.get_logger().info('Rover Arm Hardware Bridge - Old System Compatible')
        self.get_logger().info(f'Using gear ratios: {GEAR_RATIOS.tolist()}')
        self.get_logger().info('Waiting for motor encoders on /get_arm_position...')
    
    def encoder_callback(self, msg: Int32MultiArray):
        """
        Receive encoder feedback from motors (OLD SYSTEM FORMAT)
        Exactly matches old ros_arm_interface.py line 64-68
        """
        if len(msg.data) < 6:
            self.get_logger().warn(f'Expected 6 encoder values, got {len(msg.data)}')
            return
        
        # Store raw motor ticks (6 values)
        inp = np.array(list(msg.data[:6]))
        
        # Convert ticks to radians using OLD SYSTEM formula
        # From line 66: angles = list(inp*2*np.pi/TICKS_PER_REV/gear_ratios)
        angles = inp * 2 * np.pi / TICKS_PER_REV / GEAR_RATIOS
        
        # Apply inversion on motor 4 (OLD SYSTEM line 67)
        angles[4] *= -1
        
        # Map motor angles to our 5 joints
        for i, joint_name in enumerate(JOINT_NAMES):
            motor_idx = JOINT_TO_MOTOR_INDEX[joint_name]
            self.joint_positions[i] = angles[motor_idx]
        
        # Estimate velocities
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0:
            self.joint_velocities = (self.joint_positions - self.last_positions) / dt
        
        self.last_time = current_time
        self.last_positions = self.joint_positions.copy()
    
    def joint_command_callback(self, msg: JointState):
        """
        Receive joint commands from ros2_control (radians)
        Convert to motor ticks using OLD SYSTEM formula (inverse)
        """
        # Extract commanded positions for each joint
        commanded_angles = np.zeros(6)  # 6 motors
        
        for joint_name in msg.name:
            if joint_name in JOINT_TO_MOTOR_INDEX:
                joint_idx = msg.name.index(joint_name)
                motor_idx = JOINT_TO_MOTOR_INDEX[joint_name]
                commanded_angles[motor_idx] = msg.position[joint_idx]
        
        # Apply inversion on motor 4 BEFORE converting to ticks
        commanded_angles[4] *= -1
        
        # Convert radians to ticks using OLD SYSTEM formula (inverse)
        # Original: angles = inp * 2*pi / TICKS_PER_REV / gear_ratios
        # Inverse:  ticks = angles * TICKS_PER_REV * gear_ratios / (2*pi)
        self.motor_commands_ticks = (
            commanded_angles * TICKS_PER_REV * GEAR_RATIOS / (2 * np.pi)
        ).astype(int)
        
        # Publish to motor firmware (OLD SYSTEM FORMAT)
        motor_msg = Int32MultiArray()
        motor_msg.data = self.motor_commands_ticks.tolist()
        self.motor_cmd_pub.publish(motor_msg)
    
    def publish_joint_states(self):
        """Publish current joint states for ros2_control"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self.joint_positions.tolist()
        msg.velocity = self.joint_velocities.tolist()
        msg.effort = [0.0] * 5  # No effort feedback
        
        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    bridge = RoverArmHardwareBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
