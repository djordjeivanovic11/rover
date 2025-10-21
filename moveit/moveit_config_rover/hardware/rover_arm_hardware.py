#!/usr/bin/env python3
"""
=============================================================================
ROVER ARM HARDWARE INTERFACE
=============================================================================
ROS2 Control hardware interface that bridges MoveIt trajectories to the
actual rover arm motor controllers. This replaces the old custom IK system
with a MoveIt-compatible interface while maintaining hardware compatibility.

Based on the existing motor control system from old_arm/.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import numpy as np
import time


# Motor and encoder constants from old system
TICKS_PER_REV = 1600
GEAR_RATIOS = {
    'AB_Rev': 50,       # Base rotation (continuous)
    'AS1_Rev': 100,     # Shoulder pitch (Y-axis)
    'AS2_Rev': 100,     # Shoulder yaw (Z-axis) - NEW 5th DOF
    'AW_Rev': 50,       # Elbow pitch
    'AM_Rev': 50 * 22 / 9,  # Wrist roll
}


class RoverArmHardware(LifecycleNode):
    """
    Hardware interface for the rover arm.
    
    This class implements the ROS2 Control hardware interface for the rover
    arm. It communicates with the actual motor controllers and provides
    position/velocity feedback to MoveIt.
    """
    
    def __init__(self):
        super().__init__('rover_arm_hardware')
        
        # Joint configuration (matching URDF - NOW 5-DOF)
        self.joint_names = ['AB_Rev', 'AS1_Rev', 'AS2_Rev', 'AW_Rev', 'AM_Rev']
        self.num_joints = len(self.joint_names)
        
        # State vectors (in radians)
        self.joint_positions = np.zeros(self.num_joints)
        self.joint_velocities = np.zeros(self.num_joints)
        self.joint_efforts = np.zeros(self.num_joints)
        
        # Command vectors (in radians)
        self.position_commands = np.zeros(self.num_joints)
        self.velocity_commands = np.zeros(self.num_joints)
        
        # Motor encoder positions (in ticks)
        self.motor_encoder_ticks = np.zeros(self.num_joints)
        
        # Publishers for motor commands (in ticks)
        self.motor_command_pub = self.create_publisher(
            Int32MultiArray,
            '/arm_target_motor_positions',
            10
        )
        
        # Subscribers for motor feedback
        self.motor_feedback_sub = self.create_subscription(
            Int32MultiArray,
            '/get_arm_position',
            self._motor_feedback_callback,
            10
        )
        
        # Timing
        self.last_update_time = time.time()
        self.control_frequency = 50.0  # Hz
        
        # Create timer for periodic updates
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self._control_loop_callback
        )
        
        self.get_logger().info('Rover Arm Hardware Interface initialized')
    
    def _motor_feedback_callback(self, msg):
        """
        Callback for motor encoder feedback.
        Converts motor ticks to joint angles in radians.
        """
        if len(msg.data) < self.num_joints:
            self.get_logger().warn('Received incomplete motor feedback')
            return
        
        # Store raw encoder ticks
        self.motor_encoder_ticks = np.array(msg.data[:self.num_joints])
        
        # Convert ticks to radians
        for i, joint_name in enumerate(self.joint_names):
            gear_ratio = GEAR_RATIOS[joint_name]
            # Ticks to radians: ticks / (TICKS_PER_REV * gear_ratio) * 2π
            self.joint_positions[i] = (
                self.motor_encoder_ticks[i] / (TICKS_PER_REV * gear_ratio) * 2 * np.pi
            )
        
        # Estimate velocities using finite differences
        current_time = time.time()
        dt = current_time - self.last_update_time
        if dt > 0:
            # Simple velocity estimation
            # In a real system, you might get this from motor controllers
            pass
        
        self.last_update_time = current_time
    
    def _control_loop_callback(self):
        """
        Main control loop that sends position commands to motors.
        """
        # Convert commanded joint positions (radians) to motor ticks
        motor_commands_ticks = np.zeros(self.num_joints, dtype=int)
        
        for i, joint_name in enumerate(self.joint_names):
            gear_ratio = GEAR_RATIOS[joint_name]
            # Radians to ticks: radians / (2π) * (TICKS_PER_REV * gear_ratio)
            motor_commands_ticks[i] = int(
                self.position_commands[i] / (2 * np.pi) * (TICKS_PER_REV * gear_ratio)
            )
        
        # Publish motor commands
        msg = Int32MultiArray()
        msg.data = motor_commands_ticks.tolist()
        self.motor_command_pub.publish(msg)
    
    # ROS2 Control Hardware Interface Methods
    
    def read(self):
        """
        Read current state from hardware.
        Called by controller manager to get joint states.
        """
        # State is updated by _motor_feedback_callback
        return self.joint_positions, self.joint_velocities, self.joint_efforts
    
    def write(self, commands):
        """
        Write commands to hardware.
        Called by controller manager to send joint commands.
        
        Args:
            commands: Dictionary with joint names as keys and position commands as values
        """
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in commands:
                self.position_commands[i] = commands[joint_name]
    
    def get_joint_positions(self):
        """Get current joint positions in radians"""
        return self.joint_positions.copy()
    
    def get_joint_velocities(self):
        """Get current joint velocities in rad/s"""
        return self.joint_velocities.copy()
    
    def set_joint_positions(self, positions):
        """
        Set commanded joint positions in radians
        
        Args:
            positions: numpy array or list of joint positions [AB_Rev, AS1_Rev, AS2_Rev, AW_Rev, AM_Rev]
        """
        if len(positions) == self.num_joints:
            self.position_commands = np.array(positions)
        else:
            self.get_logger().error(f'Expected {self.num_joints} positions, got {len(positions)}')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        hardware = RoverArmHardware()
        rclpy.spin(hardware)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Hardware interface error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

