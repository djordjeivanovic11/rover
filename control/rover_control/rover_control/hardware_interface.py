#!/usr/bin/env python3
"""
=============================================================================
ROVER HARDWARE INTERFACE
=============================================================================
ROS 2 Control hardware interface plugin for URC rover differential drive.
Starts in mock mode for safe testing, then adds real hardware protocol.
Provides velocity control for 4-wheel differential drive system.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from hardware_interface import SystemInterface
from lifecycle_msgs.msg import State

import yaml
import numpy as np
import threading
import time
import math
from typing import Dict, List, Optional
from pathlib import Path

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class RoverHardwareInterface(SystemInterface, LifecycleNode):
    """
    Hardware interface for URC rover differential drive system.
    
    Provides 50 Hz control loop with:
    - Velocity command interface for 4 wheels
    - Joint state feedback (position, velocity)
    - Odometry computation and publishing
    - Mock mode for safe testing
    - Real hardware integration via CAN bus
    """
    
    def __init__(self):
        LifecycleNode.__init__(self, "rover_hardware_interface")
        
        # Load configuration
        self._load_configuration()
        
        # Wheel joint configuration
        self.joint_names = [
            "left_front_wheel", "left_rear_wheel",
            "right_front_wheel", "right_rear_wheel"
        ]
        self.num_joints = len(self.joint_names)
        
        # State vectors
        self.joint_positions = np.zeros(self.num_joints)      # rad
        self.joint_velocities = np.zeros(self.num_joints)     # rad/s
        self.joint_efforts = np.zeros(self.num_joints)        # N⋅m
        
        # Command vectors
        self.velocity_commands = np.zeros(self.num_joints)    # rad/s
        self.prev_velocity_commands = np.zeros(self.num_joints)
        
        # Physical parameters
        self.wheel_radius = self.config.get('wheel_radius', 0.125)      # meters
        self.wheel_separation = self.config.get('wheel_separation', 0.48)  # meters
        self.gear_ratio = self.config.get('gear_ratio', 20.0)
        self.encoder_resolution = self.config.get('encoder_resolution', 2048)
        
        # Control loop timing
        self.control_frequency = self.config.get('update_rate', 50.0)  # Hz
        self.control_period = 1.0 / self.control_frequency
        self.last_update_time = time.time()
        
        # Hardware mode
        self.mock_mode = self.config.get('mock_hardware', True)
        self.hardware_connected = False
        
        # Odometry
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        
        # Safety and diagnostics
        self.estop_active = False
        self.motor_currents = np.zeros(self.num_joints)       # Amps
        self.motor_temperatures = np.zeros(self.num_joints)   # Celsius
        self.battery_voltage = 24.0
        self.battery_current = 0.0
        
        # Publishers for monitoring
        self.joint_state_pub = self.create_publisher(
            JointState, "/joint_states", 10)
        self.odom_pub = self.create_publisher(
            Odometry, "/odom", 10)
        self.motor_current_pub = self.create_publisher(
            Float32MultiArray, "/rover/motor_currents", 10)
        self.motor_temp_pub = self.create_publisher(
            Float32MultiArray, "/rover/motor_temperatures", 10)
        self.estop_pub = self.create_publisher(
            Bool, "/rover/estop_hw", 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10)
            
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        self.estop_sub = self.create_subscription(
            Bool, "/rover/estop_sw", self._estop_callback, 10)
            
        # Control loop thread
        self.control_thread = None
        self.control_running = False
        
        # Mock hardware simulation
        self.mock_wheel_velocities = np.zeros(self.num_joints)
        
        # Real hardware (CAN bus) - placeholder for actual implementation
        self.can_interface = None
        self.motor_controllers = {}
        
        self.get_logger().info(f"Rover Hardware Interface initialized in {'MOCK' if self.mock_mode else 'REAL'} mode")
        
    def _load_configuration(self):
        """Load configuration from parameters"""
        # Get all parameters from the node
        self.config = {}
        
        # Declare parameters with defaults
        param_defaults = {
            'wheel_radius': 0.125,
            'wheel_separation': 0.48,
            'gear_ratio': 20.0,
            'encoder_resolution': 2048,
            'update_rate': 50.0,
            'mock_hardware': True,
            'can_interface': 'can0',
            'motor_ids': [1, 2, 3, 4],
            'max_velocity': 2.5,
            'max_acceleration': 2.0,
            'diagnostic_updater_rate': 10.0,
            'publish_joint_states': True,
            'publish_wheel_tf': False
        }
        
        for param_name, default_value in param_defaults.items():
            self.declare_parameter(param_name, default_value)
            self.config[param_name] = self.get_parameter(param_name).value
            
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the hardware interface"""
        self.get_logger().info("Configuring rover hardware interface...")
        
        if not self.mock_mode:
            # Initialize real hardware
            if not self._initialize_hardware():
                self.get_logger().error("Failed to initialize hardware")
                return TransitionCallbackReturn.FAILURE
                
        # Initialize diagnostic timer
        self.diagnostic_timer = self.create_timer(
            1.0 / self.config['diagnostic_updater_rate'], 
            self._publish_diagnostics_callback)
            
        self.get_logger().info("Hardware interface configured successfully")
        return TransitionCallbackReturn.SUCCESS
        
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the hardware interface"""
        self.get_logger().info("Activating rover hardware interface...")
        
        # Start control loop
        self.control_running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        
        self.get_logger().info("Hardware interface activated successfully")
        return TransitionCallbackReturn.SUCCESS
        
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the hardware interface"""
        self.get_logger().info("Deactivating rover hardware interface...")
        
        # Stop control loop
        self.control_running = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
            
        # Stop all motors
        self.velocity_commands.fill(0.0)
        self._send_motor_commands()
        
        self.get_logger().info("Hardware interface deactivated successfully")
        return TransitionCallbackReturn.SUCCESS
        
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup the hardware interface"""
        self.get_logger().info("Cleaning up rover hardware interface...")
        
        if not self.mock_mode and self.hardware_connected:
            self._shutdown_hardware()
            
        self.get_logger().info("Hardware interface cleaned up successfully")
        return TransitionCallbackReturn.SUCCESS
        
    def _initialize_hardware(self) -> bool:
        """Initialize real hardware (CAN bus motor controllers)"""
        try:
            # Initialize CAN interface
            can_interface = self.config['can_interface']
            motor_ids = self.config['motor_ids']
            
            self.get_logger().info(f"Initializing CAN interface: {can_interface}")
            
            # TODO: Initialize actual CAN bus communication
            # This would involve:
            # 1. Setting up python-can interface
            # 2. Configuring motor controllers
            # 3. Setting up encoder feedback
            # 4. Initializing safety monitoring
            
            # For now, simulate successful initialization
            self.hardware_connected = True
            self.get_logger().info("Hardware initialization successful (simulated)")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Hardware initialization failed: {str(e)}")
            return False
            
    def _shutdown_hardware(self):
        """Shutdown real hardware"""
        try:
            # Stop all motors
            self.velocity_commands.fill(0.0)
            self._send_motor_commands()
            
            # TODO: Properly close CAN interface
            self.hardware_connected = False
            self.get_logger().info("Hardware shutdown complete")
            
        except Exception as e:
            self.get_logger().error(f"Hardware shutdown error: {str(e)}")
            
    def _control_loop(self):
        """Main control loop running at specified frequency"""
        self.get_logger().info(f"Starting control loop at {self.control_frequency} Hz")
        
        while self.control_running:
            loop_start_time = time.time()
            
            try:
                # Read sensor data
                self._read_sensors()
                
                # Send motor commands
                self._send_motor_commands()
                
                # Update odometry
                self._update_odometry()
                
                # Publish joint states
                if self.config['publish_joint_states']:
                    self._publish_joint_states()
                    
                # Publish odometry
                self._publish_odometry()
                
                # Publish motor telemetry
                self._publish_motor_telemetry()
                
            except Exception as e:
                self.get_logger().error(f"Control loop error: {str(e)}")
                
            # Maintain loop frequency
            loop_duration = time.time() - loop_start_time
            sleep_time = max(0, self.control_period - loop_duration)
            
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                self.get_logger().warn(f"Control loop overrun: {loop_duration:.3f}s > {self.control_period:.3f}s")
                
    def _read_sensors(self):
        """Read sensor data from hardware or simulation"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        
        if self.mock_mode:
            # Simulate sensor readings
            self._simulate_sensors(dt)
        else:
            # Read from real hardware
            self._read_real_sensors()
            
        self.last_update_time = current_time
        
    def _simulate_sensors(self, dt: float):
        """Simulate sensor readings for mock mode"""
        # Simulate wheel motion based on commanded velocities
        for i in range(self.num_joints):
            # Simple integration with some realistic dynamics
            acceleration = (self.velocity_commands[i] - self.mock_wheel_velocities[i]) * 5.0  # rad/s²
            acceleration = np.clip(acceleration, -10.0, 10.0)  # Limit acceleration
            
            self.mock_wheel_velocities[i] += acceleration * dt
            self.joint_velocities[i] = self.mock_wheel_velocities[i]
            self.joint_positions[i] += self.joint_velocities[i] * dt
            
            # Simulate motor current based on velocity command
            self.motor_currents[i] = abs(self.velocity_commands[i]) * 2.0 + np.random.normal(0, 0.1)
            self.motor_currents[i] = max(0, self.motor_currents[i])
            
            # Simulate motor temperature
            self.motor_temperatures[i] = 25.0 + abs(self.motor_currents[i]) * 5.0 + np.random.normal(0, 1.0)
            
        # Simulate battery
        total_current = np.sum(self.motor_currents)
        self.battery_current = total_current
        self.battery_voltage = 24.0 - total_current * 0.1  # Simple voltage drop
        
    def _read_real_sensors(self):
        """Read sensor data from real hardware"""
        # TODO: Implement actual sensor reading
        # This would involve:
        # 1. Reading encoder positions from CAN bus
        # 2. Computing velocities from position differences
        # 3. Reading motor currents and temperatures
        # 4. Reading battery voltage and current
        
        pass
        
    def _send_motor_commands(self):
        """Send velocity commands to motors"""
        if self.estop_active:
            # Emergency stop - zero all commands
            commands = np.zeros(self.num_joints)
        else:
            # Apply safety limits
            commands = self._apply_safety_limits(self.velocity_commands.copy())
            
        if self.mock_mode:
            # In mock mode, commands are handled by simulation
            pass
        else:
            # Send commands to real hardware
            self._send_real_motor_commands(commands)
            
        self.prev_velocity_commands = commands.copy()
        
    def _apply_safety_limits(self, commands: np.ndarray) -> np.ndarray:
        """Apply safety limits to motor commands"""
        max_velocity = self.config['max_velocity'] / self.wheel_radius  # Convert m/s to rad/s
        max_acceleration = self.config['max_acceleration'] / self.wheel_radius  # Convert m/s² to rad/s²
        
        # Velocity limits
        commands = np.clip(commands, -max_velocity, max_velocity)
        
        # Acceleration limits
        dt = self.control_period
        max_velocity_change = max_acceleration * dt
        
        for i in range(self.num_joints):
            velocity_change = commands[i] - self.prev_velocity_commands[i]
            if abs(velocity_change) > max_velocity_change:
                commands[i] = self.prev_velocity_commands[i] + np.sign(velocity_change) * max_velocity_change
                
        return commands
        
    def _send_real_motor_commands(self, commands: np.ndarray):
        """Send commands to real motor controllers via CAN bus"""
        # TODO: Implement actual CAN bus communication
        # This would involve:
        # 1. Converting rad/s to motor controller units
        # 2. Sending CAN messages to each motor controller
        # 3. Handling communication errors and timeouts
        
        pass
        
    def _update_odometry(self):
        """Update odometry based on wheel positions"""
        # Get wheel positions for odometry calculation
        left_pos = (self.joint_positions[0] + self.joint_positions[1]) / 2.0  # Average of left wheels
        right_pos = (self.joint_positions[2] + self.joint_positions[3]) / 2.0  # Average of right wheels
        
        # Calculate distance traveled by each side
        left_dist = (left_pos - self.prev_left_pos) * self.wheel_radius
        right_dist = (right_pos - self.prev_right_pos) * self.wheel_radius
        
        # Calculate center distance and rotation
        center_dist = (left_dist + right_dist) / 2.0
        delta_theta = (right_dist - left_dist) / self.wheel_separation
        
        # Update pose
        self.odom_theta += delta_theta
        self.odom_x += center_dist * math.cos(self.odom_theta)
        self.odom_y += center_dist * math.sin(self.odom_theta)
        
        # Store previous positions
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos
        
    def _publish_joint_states(self):
        """Publish joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.name = self.joint_names
        msg.position = self.joint_positions.tolist()
        msg.velocity = self.joint_velocities.tolist()
        msg.effort = self.joint_efforts.tolist()
        
        self.joint_state_pub.publish(msg)
        
    def _publish_odometry(self):
        """Publish odometry message"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        # Position
        msg.pose.pose.position.x = self.odom_x
        msg.pose.pose.position.y = self.odom_y
        msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.odom_theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.odom_theta / 2.0)
        
        # Velocity
        left_vel = (self.joint_velocities[0] + self.joint_velocities[1]) / 2.0 * self.wheel_radius
        right_vel = (self.joint_velocities[2] + self.joint_velocities[3]) / 2.0 * self.wheel_radius
        
        linear_vel = (left_vel + right_vel) / 2.0
        angular_vel = (right_vel - left_vel) / self.wheel_separation
        
        msg.twist.twist.linear.x = linear_vel
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = angular_vel
        
        # Covariance (from config)
        pose_covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        
        twist_covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        
        msg.pose.covariance = pose_covariance
        msg.twist.covariance = twist_covariance
        
        self.odom_pub.publish(msg)
        
    def _publish_motor_telemetry(self):
        """Publish motor telemetry data"""
        # Motor currents
        current_msg = Float32MultiArray()
        current_msg.data = self.motor_currents.tolist()
        self.motor_current_pub.publish(current_msg)
        
        # Motor temperatures
        temp_msg = Float32MultiArray()
        temp_msg.data = self.motor_temperatures.tolist()
        self.motor_temp_pub.publish(temp_msg)
        
        # E-stop status
        estop_msg = Bool()
        estop_msg.data = self.estop_active
        self.estop_pub.publish(estop_msg)
        
    def _publish_diagnostics_callback(self):
        """Publish diagnostic information"""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Hardware interface status
        status = DiagnosticStatus()
        status.name = "rover_hardware_interface"
        status.hardware_id = "rover_control"
        
        if self.control_running and (self.mock_mode or self.hardware_connected):
            status.level = DiagnosticStatus.OK
            status.message = f"Operating in {'MOCK' if self.mock_mode else 'REAL'} mode"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = "Hardware interface not active"
            
        # Add key-value pairs
        status.values.append(KeyValue(key="mode", value="mock" if self.mock_mode else "real"))
        status.values.append(KeyValue(key="control_frequency", value=f"{self.control_frequency:.1f}"))
        status.values.append(KeyValue(key="hardware_connected", value=str(self.hardware_connected)))
        status.values.append(KeyValue(key="estop_active", value=str(self.estop_active)))
        status.values.append(KeyValue(key="battery_voltage", value=f"{self.battery_voltage:.2f}"))
        status.values.append(KeyValue(key="battery_current", value=f"{self.battery_current:.2f}"))
        
        msg.status.append(status)
        self.diagnostics_pub.publish(msg)
        
    def _cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to wheel velocities"""
        linear_vel = msg.linear.x  # m/s
        angular_vel = msg.angular.z  # rad/s
        
        # Convert to wheel velocities using differential drive kinematics
        left_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Set wheel velocity commands
        self.velocity_commands[0] = left_vel   # left_front_wheel
        self.velocity_commands[1] = left_vel   # left_rear_wheel
        self.velocity_commands[2] = right_vel  # right_front_wheel
        self.velocity_commands[3] = right_vel  # right_rear_wheel
        
    def _estop_callback(self, msg: Bool):
        """Handle software emergency stop"""
        self.estop_active = msg.data
        if self.estop_active:
            self.get_logger().warn("Software emergency stop activated!")
            self.velocity_commands.fill(0.0)


def main(args=None):
    """Main entry point for the hardware interface node"""
    rclpy.init(args=args)
    
    try:
        node = RoverHardwareInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in hardware interface: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 