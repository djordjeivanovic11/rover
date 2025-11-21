#!/usr/bin/env python3
"""
Wheel Speed Bridge to Teensy

Subscribes to /cmd_wheels (TankDriveTarget with wheel speeds in m/s)
Converts to RPM and sends to Teensy via Serial or UDP

Features:
- Converts m/s to RPM using wheel radius
- Deadman timeout (stops if no commands)
- Slew-rate limiting (smooth acceleration)
- Per-wheel inversion (motor wiring fixes)
- RPM clamping for safety
- Atomic L/R commands (single message)
"""
import socket
import math
import sys
import rclpy
from rclpy.node import Node
from urc_msgs.msg import TankDriveTarget

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

from std_msgs.msg import Int32


class WheelBridge(Node):
    def __init__(self):
        super().__init__('wheel_bridge')
        
        # Transport parameters
        self.declare_parameter('transport', 'serial')  # 'serial', 'udp', or 'ros'
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('udp_host', '192.168.0.10')
        self.declare_parameter('udp_port', 3000)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('timeout_ms', 250)
        
        # Kinematics parameters
        self.declare_parameter('wheel_radius_m', 0.105)  # Measure your wheel!
        self.declare_parameter('max_rpm', 15000)         # VESC limit or safe cap
        
        # Motor wiring fixes
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        
        # Slew limiting (max RPM change per update cycle)
        self.declare_parameter('max_rpm_step', 500)
        
        # Get parameters
        self.transport = self.get_parameter('transport').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.udp_host = self.get_parameter('udp_host').value
        self.udp_port = self.get_parameter('udp_port').value
        self.rate = self.get_parameter('rate_hz').value
        self.timeout_ms = self.get_parameter('timeout_ms').value
        self.wheel_radius = self.get_parameter('wheel_radius_m').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.invert_left = self.get_parameter('invert_left').value
        self.invert_right = self.get_parameter('invert_right').value
        self.max_step = self.get_parameter('max_rpm_step').value
        
        # Connection (None for ROS mode)
        self.conn = None
        
        # ROS publishers (only used in 'ros' transport mode)
        self.pub_left = None
        self.pub_right = None
        
        if self.transport == 'ros':
            self.pub_left = self.create_publisher(Int32, '/drive/left_rpm', 10)
            self.pub_right = self.create_publisher(Int32, '/drive/right_rpm', 10)
        else:
            self._connect()
        
        # State
        self.last_msg = None
        self.last_msg_time = self.get_clock().now()
        self.current_rpm_left = 0
        self.current_rpm_right = 0
        self.sent_stop = False
        
        # Manual throttling for logs
        self.last_warn_time = 0.0
        self.last_debug_time = 0.0
        
        # Conversion factor: m/s to RPM
        # rpm = (v_mps / (2 * pi * R)) * 60
        self.mps_to_rpm = 60.0 / (2.0 * math.pi * self.wheel_radius)
        
        # Subscriber
        self.sub = self.create_subscription(
            TankDriveTarget,
            '/cmd_wheels',
            self.wheel_callback,
            10
        )
        
        # Timer for periodic updates
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.update_timer)
        
        self.get_logger().info('═' * 60)
        self.get_logger().info(f'Wheel Bridge: {self.transport.upper()}')
        if self.transport == 'serial':
            self.get_logger().info(f'  Port: {self.serial_port} @ {self.baudrate} baud')
        elif self.transport == 'udp':
            self.get_logger().info(f'  UDP: {self.udp_host}:{self.udp_port}')
        elif self.transport == 'ros':
            self.get_logger().info(f'  Publishing to: /drive/left_rpm, /drive/right_rpm')
        self.get_logger().info(f'  Wheel radius: {self.wheel_radius:.3f} m')
        self.get_logger().info(f'  Max RPM: {self.max_rpm}')
        self.get_logger().info(f'  Update rate: {self.rate:.0f} Hz')
        self.get_logger().info('═' * 60)
    
    def _connect(self):
        """Establish connection to Teensy"""
        try:
            if self.transport == 'serial':
                if not HAS_SERIAL:
                    self.get_logger().error('pyserial not installed! Run: pip install pyserial')
                    sys.exit(1)
                self.conn = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
                self.get_logger().info(f'✓ Connected to {self.serial_port}')
            elif self.transport == 'udp':
                self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.get_logger().info(f'✓ UDP socket ready')
            else:
                self.get_logger().error(f'Unknown transport: {self.transport}')
                sys.exit(1)
        except Exception as e:
            self.get_logger().error(f'✗ Connection failed: {e}')
            sys.exit(1)
    
    def wheel_callback(self, msg: TankDriveTarget):
        """Store latest wheel speed command"""
        self.last_msg = msg
        self.last_msg_time = self.get_clock().now()
        self.sent_stop = False
    
    def mps_to_rpm_convert(self, speed_mps):
        """Convert linear wheel speed (m/s) to motor RPM"""
        return int(speed_mps * self.mps_to_rpm)
    
    def slew_limit(self, target, current):
        """Apply slew-rate limiting for smooth acceleration"""
        delta = target - current
        if delta > self.max_step:
            delta = self.max_step
        elif delta < -self.max_step:
            delta = -self.max_step
        return current + delta
    
    def send_to_teensy(self, rpm_left, rpm_right):
        """Send L/R RPM commands to Teensy or publish to ROS topics"""
        
        if self.transport == 'ros':
            # Publish to ROS topics for micro-ROS firmware to subscribe
            msg_left = Int32()
            msg_left.data = rpm_left
            self.pub_left.publish(msg_left)
            
            msg_right = Int32()
            msg_right.data = rpm_right
            self.pub_right.publish(msg_right)
            
        else:
            # Send via Serial or UDP
            if self.conn is None:
                return
            
            try:
                # Send both wheels in single atomic message
                cmd = f"L{rpm_left} R{rpm_right}\r\n".encode('ascii')
                
                if self.transport == 'serial':
                    self.conn.write(cmd)
                else:  # UDP
                    self.conn.sendto(cmd, (self.udp_host, self.udp_port))
            except Exception as e:
                now_sec = self.get_clock().now().nanoseconds / 1e9
                if now_sec - self.last_warn_time > 1.0:
                    self.get_logger().warn(f'Send failed: {e}')
                    self.last_warn_time = now_sec
    
    def update_timer(self):
        """Periodic update: check timeout and send wheel commands"""
        now = self.get_clock().now()
        
        # Check for message timeout (deadman)
        if self.last_msg is None:
            age_ms = float('inf')
        else:
            age_ms = (now - self.last_msg_time).nanoseconds / 1e6
        
        if age_ms > self.timeout_ms:
            # Timeout - slew to stop
            if not self.sent_stop:
                now_sec = self.get_clock().now().nanoseconds / 1e9
                if now_sec - self.last_warn_time > 2.0:
                    self.get_logger().warn(f'⚠ No commands for {age_ms:.0f}ms - stopping')
                    self.last_warn_time = now_sec
                self.sent_stop = True
            
            # Slew to zero
            self.current_rpm_left = self.slew_limit(0, self.current_rpm_left)
            self.current_rpm_right = self.slew_limit(0, self.current_rpm_right)
            self.send_to_teensy(self.current_rpm_left, self.current_rpm_right)
            return
        
        # Convert m/s to RPM
        target_rpm_left  = self.mps_to_rpm_convert(self.last_msg.left_speed)
        target_rpm_right = self.mps_to_rpm_convert(self.last_msg.right_speed)
        
        # Apply motor inversions if needed
        if self.invert_left:
            target_rpm_left = -target_rpm_left
        if self.invert_right:
            target_rpm_right = -target_rpm_right
        
        # Clamp to max RPM
        target_rpm_left  = max(-self.max_rpm, min(self.max_rpm, target_rpm_left))
        target_rpm_right = max(-self.max_rpm, min(self.max_rpm, target_rpm_right))
        
        # Apply slew-rate limiting
        self.current_rpm_left  = self.slew_limit(target_rpm_left,  self.current_rpm_left)
        self.current_rpm_right = self.slew_limit(target_rpm_right, self.current_rpm_right)
        
        # Send to Teensy
        self.send_to_teensy(self.current_rpm_left, self.current_rpm_right)
    
    def shutdown(self):
        """Clean shutdown - send stop"""
        self.get_logger().info('Shutting down - sending STOP')
        self.send_to_teensy(0, 0)
        if self.conn and self.transport == 'serial' and self.conn.is_open:
            self.conn.close()


def main(args=None):
    rclpy.init(args=args)
    node = WheelBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

