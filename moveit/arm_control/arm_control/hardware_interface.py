#!/usr/bin/env python3
"""
=============================================================================
ARM HARDWARE INTERFACE
=============================================================================
ROS 2 Control hardware interface plugin for URC rover arm.
Starts in mock mode for safe testing, then adds real hardware protocol.
Enforces soft limits from arm_params.yaml at 250 Hz.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from hardware_interface import SystemInterface
from lifecycle_msgs.msg import State
from rclpy_lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

import yaml
import numpy as np
from typing import Dict, List, Optional
import threading
import time
from pathlib import Path

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import WrenchStamped


class ArmHardwareInterface(SystemInterface, LifecycleNode):
    """
    Hardware interface for URC rover arm.
    
    Provides 250 Hz control loop with:
    - Position/velocity command interface
    - Joint state feedback 
    - Current and temperature monitoring
    - Soft limit enforcement
    - Mock mode for safe testing
    """
    
    def __init__(self):
        LifecycleNode.__init__(self, "arm_hardware_interface")
        
        # Load configuration
        self._load_configuration()
        
        # Joint configuration
        self.joint_names = [
            "base_rotate", "shoulder_pitch", "shoulder_roll",
            "elbow_pitch", "wrist_pitch", "wrist_roll", "gripper_joint"
        ]
        self.num_joints = len(self.joint_names)
        
        # State vectors
        self.joint_positions = np.zeros(self.num_joints)
        self.joint_velocities = np.zeros(self.num_joints)
        self.joint_efforts = np.zeros(self.num_joints)
        self.joint_currents = np.zeros(self.num_joints)
        self.joint_temperatures = np.zeros(self.num_joints)
        
        # Command vectors
        self.position_commands = np.zeros(self.num_joints)
        self.velocity_commands = np.zeros(self.num_joints)
        self.effort_commands = np.zeros(self.num_joints)
        
        # Previous commands for rate limiting
        self.prev_position_commands = np.zeros(self.num_joints)
        self.prev_velocity_commands = np.zeros(self.num_joints)
        
        # Control loop timing
        self.control_frequency = 250.0  # Hz
        self.control_period = 1.0 / self.control_frequency
        self.last_update_time = time.time()
        
        # Hardware communication
        self.mock_mode = True  # Start in mock mode
        self.hardware_connected = False
        self.estop_active = False
        
        # Publishers for monitoring
        self.joint_current_pub = self.create_publisher(
            Float32MultiArray, "/arm/joint_currents", 10)
        self.joint_temp_pub = self.create_publisher(
            Float32MultiArray, "/arm/joint_temperatures", 10) 
        self.tcp_wrench_pub = self.create_publisher(
            WrenchStamped, "/arm/tcp_wrench", 10)
        self.estop_pub = self.create_publisher(
            Bool, "/arm/estop_hw", 10)
            
        # Control loop thread
        self.control_thread = None
        self.control_running = False
        
        self.get_logger().info("Arm Hardware Interface initialized in MOCK MODE")
        
    def _load_configuration(self):
        """Load configuration from YAML files"""
        try:
            # Load arm parameters
            arm_params_path = Path(__file__).parent.parent / "config" / "arm_params.yaml"
            with open(arm_params_path, 'r') as f:
                arm_config = yaml.safe_load(f)
            
            # Load controller configuration  
            controller_config_path = Path(__file__).parent.parent / "config" / "controller_config.yaml"
            with open(controller_config_path, 'r') as f:
                controller_config = yaml.safe_load(f)
            
            # Extract relevant parameters
            self.joint_limits = arm_config['arm']['joint_limits']
            self.hardware_config = controller_config.get('arm_hardware_interface', {}).get('ros__parameters', {})
            
            # Set mock mode from config
            self.mock_mode = self.hardware_config.get('mock_mode', True)
            
            self.get_logger().info(f"Configuration loaded - Mock mode: {self.mock_mode}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            # Use default values
            self.joint_limits = {}
            self.hardware_config = {}
    
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the hardware interface"""
        self.get_logger().info("Configuring hardware interface...")
        
        try:
            if self.mock_mode:
                self._configure_mock_hardware()
            else:
                self._configure_real_hardware()
                
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def _configure_mock_hardware(self):
        """Configure mock hardware for testing"""
        self.get_logger().info("Configuring MOCK hardware interface")
        
        # Initialize mock joint states to safe positions
        safe_positions = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0, 0.0]  # stow position + gripper
        self.joint_positions = np.array(safe_positions)
        
        # Mock temperature and current values
        self.joint_temperatures = np.full(self.num_joints, 25.0)  # Room temperature
        self.joint_currents = np.zeros(self.num_joints)
        
        self.hardware_connected = True
        
    def _configure_real_hardware(self):
        """Configure real hardware communication"""
        self.get_logger().info("Configuring REAL hardware interface")
        
        # TODO: Initialize CAN bus communication
        # TODO: Initialize motor controllers
        # TODO: Initialize sensor interfaces
        # TODO: Initialize GPIO for E-stop
        
        # For now, raise an error since real hardware is not implemented
        raise NotImplementedError("Real hardware interface not yet implemented")
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the hardware interface and start control loop"""
        self.get_logger().info("Activating hardware interface...")
        
        try:
            # Start control loop
            self._start_control_loop()
            
            self.get_logger().info("Hardware interface activated successfully")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Activation failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the hardware interface"""
        self.get_logger().info("Deactivating hardware interface...")
        
        try:
            # Stop control loop
            self._stop_control_loop()
            
            # Set all commands to current positions (hold position)
            self.position_commands = self.joint_positions.copy()
            self.velocity_commands = np.zeros(self.num_joints)
            self.effort_commands = np.zeros(self.num_joints)
            
            self.get_logger().info("Hardware interface deactivated")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Deactivation failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def _start_control_loop(self):
        """Start the 250 Hz control loop thread"""
        self.control_running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        self.get_logger().info("Control loop started at 250 Hz")
    
    def _stop_control_loop(self):
        """Stop the control loop thread"""
        self.control_running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        self.get_logger().info("Control loop stopped")
    
    def _control_loop(self):
        """Main 250 Hz control loop"""
        while self.control_running:
            start_time = time.time()
            
            try:
                # Read sensor data
                self._read_sensors()
                
                # Check safety limits
                safety_ok = self._check_safety_limits()
                
                if safety_ok and not self.estop_active:
                    # Write commands to hardware
                    self._write_commands()
                else:
                    # Emergency stop - hold current position
                    self._emergency_stop()
                
                # Publish monitoring data
                self._publish_monitoring_data()
                
            except Exception as e:
                self.get_logger().error(f"Control loop error: {e}")
            
            # Maintain 250 Hz timing
            elapsed = time.time() - start_time
            sleep_time = max(0, self.control_period - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                self.get_logger().warn(f"Control loop overrun: {elapsed:.4f}s > {self.control_period:.4f}s")
    
    def _read_sensors(self):
        """Read sensor data from hardware"""
        if self.mock_mode:
            self._read_mock_sensors()
        else:
            self._read_real_sensors()
    
    def _read_mock_sensors(self):
        """Read mock sensor data for testing"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Simple mock physics: move towards commanded positions
        position_error = self.position_commands - self.joint_positions
        max_velocity = 1.0  # rad/s
        
        for i in range(self.num_joints):
            # Limit velocity
            desired_velocity = position_error[i] / dt if dt > 0 else 0.0
            desired_velocity = np.clip(desired_velocity, -max_velocity, max_velocity)
            
            # Update position
            self.joint_positions[i] += desired_velocity * dt
            self.joint_velocities[i] = desired_velocity
            
            # Mock current based on effort
            self.joint_currents[i] = abs(self.joint_efforts[i]) * 0.1  # Rough approximation
            
        # Mock temperature (slowly increases with current)
        self.joint_temperatures += self.joint_currents * 0.001
        self.joint_temperatures = np.clip(self.joint_temperatures, 20.0, 100.0)
    
    def _read_real_sensors(self):
        """Read real sensor data from hardware"""
        # TODO: Implement CAN bus communication
        # TODO: Read encoder positions
        # TODO: Read motor currents  
        # TODO: Read temperature sensors
        # TODO: Read force/torque sensor
        # TODO: Read E-stop status
        pass
    
    def _check_safety_limits(self) -> bool:
        """Check all safety limits and return True if safe"""
        try:
            # Check position limits
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in self.joint_limits.get('position', {}):
                    pos_limits = self.joint_limits['position'][joint_name]
                    if self.joint_positions[i] < pos_limits[0] or self.joint_positions[i] > pos_limits[1]:
                        self.get_logger().error(f"Joint {joint_name} position limit exceeded: {self.joint_positions[i]}")
                        return False
            
            # Check velocity limits
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in self.joint_limits.get('velocity', {}):
                    vel_limit = self.joint_limits['velocity'][joint_name]
                    if abs(self.joint_velocities[i]) > vel_limit:
                        self.get_logger().error(f"Joint {joint_name} velocity limit exceeded: {self.joint_velocities[i]}")
                        return False
            
            # Check current limits
            current_limits = self.hardware_config.get('current_limit_amps', 10.0)
            if np.any(self.joint_currents > current_limits):
                self.get_logger().error("Current limit exceeded")
                return False
            
            # Check temperature limits  
            temp_limit = self.hardware_config.get('temp_limit_celsius', 80.0)
            if np.any(self.joint_temperatures > temp_limit):
                self.get_logger().error("Temperature limit exceeded")
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Safety check failed: {e}")
            return False
    
    def _write_commands(self):
        """Write commands to hardware"""
        if self.mock_mode:
            self._write_mock_commands()
        else:
            self._write_real_commands()
    
    def _write_mock_commands(self):
        """Write commands to mock hardware"""
        # In mock mode, commands are processed in _read_mock_sensors()
        pass
    
    def _write_real_commands(self):
        """Write commands to real hardware"""
        # TODO: Send position/velocity commands via CAN bus
        # TODO: Apply PID control
        # TODO: Send PWM signals to motors
        pass
    
    def _emergency_stop(self):
        """Execute emergency stop procedure"""
        # Hold current position
        self.position_commands = self.joint_positions.copy()
        self.velocity_commands = np.zeros(self.num_joints)
        self.effort_commands = np.zeros(self.num_joints)
        
        # In real hardware, this would:
        # TODO: Engage motor brakes
        # TODO: Cut power to motors
        # TODO: Activate safety relays
    
    def _publish_monitoring_data(self):
        """Publish monitoring data for safety monitor"""
        try:
            # Publish joint currents
            current_msg = Float32MultiArray()
            current_msg.data = self.joint_currents.tolist()
            self.joint_current_pub.publish(current_msg)
            
            # Publish joint temperatures
            temp_msg = Float32MultiArray()
            temp_msg.data = self.joint_temperatures.tolist()
            self.joint_temp_pub.publish(temp_msg)
            
            # Publish E-stop status
            estop_msg = Bool()
            estop_msg.data = self.estop_active
            self.estop_pub.publish(estop_msg)
            
            # TODO: Publish force/torque data when sensor is available
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish monitoring data: {e}")
    
    # Hardware interface methods (required by SystemInterface)
    def read(self):
        """Read hardware state (called by controller manager)"""
        # Hardware reading is done in control loop
        pass
    
    def write(self):
        """Write hardware commands (called by controller manager)"""
        # Hardware writing is done in control loop  
        pass


def main():
    """Main entry point"""
    rclpy.init()
    
    try:
        hardware_interface = ArmHardwareInterface()
        rclpy.spin(hardware_interface)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Hardware interface failed: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main() 