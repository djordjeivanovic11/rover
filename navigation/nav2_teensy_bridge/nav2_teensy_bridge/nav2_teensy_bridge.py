#!/usr/bin/env python3
"""
Nav2 cmd_vel to Teensy serial bridge with safety features.
Subscribes to /cmd_vel and sends commands to Teensy motor controller.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


def clamp(v, lo, hi): 
    return lo if v < lo else hi if v > hi else v


class CmdVelSerialBridge(Node):
    """Bridge between Nav2 cmd_vel and Teensy motor controller."""
    
    def __init__(self):
        super().__init__('nav2_teensy_bridge')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM1')  # NOT same as GPS!
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_linear_vel', 0.6)   # m/s
        self.declare_parameter('max_angular_vel', 1.8)  # rad/s
        self.declare_parameter('cmd_timeout', 1.0)      # seconds
        self.declare_parameter('retry_connection', True)
        
        # Get parameters
        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baudrate').value
        self.v_max = self.get_parameter('max_linear_vel').value
        self.w_max = self.get_parameter('max_angular_vel').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.retry = self.get_parameter('retry_connection').value
        
        # State
        self.last_cmd_time = time.time()
        self.ser = None
        
        # Try to connect to serial
        self.connect_serial()
        
        # Subscribe to cmd_vel
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Safety timer to stop motors if no commands received
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info(f'Teensy bridge started')
        self.get_logger().info(f'  Port: {self.port}')
        self.get_logger().info(f'  Max linear: {self.v_max} m/s')
        self.get_logger().info(f'  Max angular: {self.w_max} rad/s')
        self.get_logger().info(f'  Timeout: {self.cmd_timeout}s')

    def connect_serial(self):
        """Attempt to connect to serial port."""
        if self.ser is not None and self.ser.is_open:
            return
            
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f'✓ Connected to {self.port}')
            # Send stop command on connection
            self.send_stop()
        except serial.SerialException as e:
            self.get_logger().error(f'✗ Failed to open {self.port}: {e}')
            if not self.retry:
                sys.exit(1)
            self.ser = None
        except Exception as e:
            self.get_logger().error(f'✗ Unexpected error: {e}')
            if not self.retry:
                sys.exit(1)
            self.ser = None

    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming cmd_vel messages."""
        self.last_cmd_time = time.time()
        
        # Try to reconnect if disconnected
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
            if self.ser is None:
                return
        
        # Normalize velocities to [-1, 1]
        lin_norm = clamp(msg.linear.x / self.v_max, -1.0, 1.0)
        ang_norm = clamp(msg.angular.z / self.w_max, -1.0, 1.0)
        
        # Convert to int16 range [-32768, 32767]
        to_i16 = lambda v: int(clamp(round(v * 32767), -32768, 32767))
        y_val = to_i16(lin_norm)  # Forward/backward
        x_val = to_i16(ang_norm)  # Left/right
        
        # Send to Teensy
        try:
            self.ser.write(f"x{x_val}\n".encode('ascii'))
            self.ser.write(f"y{y_val}\n".encode('ascii'))
            
            # Log at debug level (not flooding console)
            if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
                self.get_logger().debug(
                    f'cmd_vel: lin={msg.linear.x:.2f} ang={msg.angular.z:.2f} → x={x_val} y={y_val}')
        except serial.SerialException as e:
            self.get_logger().warning(f'Serial write failed: {e}')
            self.ser = None  # Will try to reconnect
        except Exception as e:
            self.get_logger().error(f'Unexpected write error: {e}')

    def send_stop(self):
        """Send stop command to motors."""
        if self.ser is None or not self.ser.is_open:
            return
        try:
            self.ser.write(b"x0\n")
            self.ser.write(b"y0\n")
            self.get_logger().debug('Sent STOP command')
        except Exception as e:
            self.get_logger().warning(f'Failed to send stop: {e}')

    def safety_check(self):
        """Safety timer: stop motors if no commands received for timeout period."""
        if self.ser is None or not self.ser.is_open:
            return
            
        elapsed = time.time() - self.last_cmd_time
        if elapsed > self.cmd_timeout:
            self.send_stop()
            if elapsed < self.cmd_timeout + 0.2:  # Log once
                self.get_logger().warn(f'⚠ cmd_vel timeout ({elapsed:.1f}s), motors stopped')

    def destroy_node(self):
        """Clean shutdown: stop motors and close serial."""
        self.get_logger().info('Shutting down...')
        self.send_stop()
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerialBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

