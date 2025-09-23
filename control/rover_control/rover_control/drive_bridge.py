#!/usr/bin/env python3
"""
=============================================================================
ROVER DRIVE BRIDGE - Jetson Side (Minimal)
=============================================================================
Ultra-simple bridge node for Arduino Uno drive system:
- Subscribes to /cmd_vel (Twist messages)
- Converts to left/right throttle values
- Sends ASCII commands to Arduino over serial
- No watchdog, no timeouts - just direct control
=============================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

import serial
import time
import threading
from typing import Optional


class DriveBridge(Node):
    """
    Minimal drive bridge for Arduino Uno PPM control.
    
    Converts ROS cmd_vel to left/right throttle commands
    and sends them to Arduino over serial.
    """
    
    def __init__(self):
        super().__init__('drive_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('wheel_separation', 0.48)  # meters
        self.declare_parameter('max_velocity', 1.0)       # m/s
        self.declare_parameter('send_rate', 20.0)         # Hz
        
        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = self.get_parameter('serial_baud').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.send_rate = self.get_parameter('send_rate').value
        
        # State
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Serial connection
        self.serial_conn = None
        self.serial_connected = False
        
        # Threading
        self.command_lock = threading.Lock()
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        
        # Publishers for status
        self.status_pub = self.create_publisher(String, '/drive_bridge_status', 10)
        self.connected_pub = self.create_publisher(Bool, '/drive_bridge_connected', 10)
        
        # Timers
        self.send_timer = self.create_timer(1.0 / self.send_rate, self._send_commands)
        self.status_timer = self.create_timer(1.0, self._publish_status)
        
        # Initialize serial connection
        self._connect_serial()
        
        self.get_logger().info(f"Drive Bridge initialized on {self.serial_port}")
    
    def _connect_serial(self):
        """Attempt to connect to Arduino"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baud,
                timeout=0.1,
                write_timeout=0.1
            )
            
            if self.serial_conn.is_open:
                self.serial_connected = True
                self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
                
                # Wait for Arduino to initialize
                time.sleep(2.0)
                
                # Send initial stop command
                self._send_throttle_command(0.0, 0.0)
            else:
                self.serial_connected = False
                self.get_logger().error(f"Failed to open {self.serial_port}")
                
        except Exception as e:
            self.serial_connected = False
            self.get_logger().error(f"Serial connection error: {e}")
    
    def _cmd_vel_callback(self, msg: Twist):
        """Handle incoming cmd_vel messages"""
        with self.command_lock:
            self.current_linear = msg.linear.x
            self.current_angular = msg.angular.z
    
    def _send_commands(self):
        """Send throttle commands to Arduino at fixed rate"""
        with self.command_lock:
            linear_vel = self.current_linear
            angular_vel = self.current_angular
        
        # Convert to left/right throttles
        left_throttle, right_throttle = self._compute_throttles(linear_vel, angular_vel)
        
        # Send to Arduino
        self._send_throttle_command(left_throttle, right_throttle)
    
    def _compute_throttles(self, linear_vel: float, angular_vel: float) -> tuple:
        """
        Convert linear and angular velocity to left/right throttles.
        
        Uses differential drive kinematics:
        v_left = v - (track_width/2) * omega
        v_right = v + (track_width/2) * omega
        """
        # Differential drive kinematics
        half_track = self.wheel_separation / 2.0
        
        v_left = linear_vel - half_track * angular_vel
        v_right = linear_vel + half_track * angular_vel
        
        # Normalize by max velocity to get throttle values (-1.0 to 1.0)
        left_throttle = v_left / self.max_velocity
        right_throttle = v_right / self.max_velocity
        
        # Clamp to valid range
        left_throttle = max(-1.0, min(1.0, left_throttle))
        right_throttle = max(-1.0, min(1.0, right_throttle))
        
        return left_throttle, right_throttle
    
    def _send_throttle_command(self, left: float, right: float):
        """Send throttle command to Arduino"""
        if not self.serial_connected or not self.serial_conn:
            return
        
        try:
            # Format: "L:<value> R:<value>\n"
            command = f"L:{left:.2f} R:{right:.2f}\n"
            
            # Send command (non-blocking)
            self.serial_conn.write(command.encode('utf-8'))
            self.serial_conn.flush()
            
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")
            self.serial_connected = False
            
            # Try to reconnect
            self._connect_serial()
    
    def _publish_status(self):
        """Publish status information"""
        # Connection status
        connected_msg = Bool()
        connected_msg.data = self.serial_connected
        self.connected_pub.publish(connected_msg)
        
        # Status message
        status_msg = String()
        if self.serial_connected:
            with self.command_lock:
                status_msg.data = f"CONNECTED - ACTIVE (v={self.current_linear:.2f}, ω={self.current_angular:.2f})"
        else:
            status_msg.data = f"DISCONNECTED - {self.serial_port}"
        
        self.status_pub.publish(status_msg)
    
    def destroy_node(self):
        """Clean shutdown"""
        # Send stop command before closing
        if self.serial_connected:
            self._send_throttle_command(0.0, 0.0)
            time.sleep(0.1)  # Give time for command to send
        
        # Close serial connection
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        
        super().destroy_node()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    drive_bridge = DriveBridge()
    
    try:
        rclpy.spin(drive_bridge)
    except KeyboardInterrupt:
        drive_bridge.get_logger().info("Drive bridge shutting down")
    finally:
        drive_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 