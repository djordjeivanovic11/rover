#!/usr/bin/env python3
"""
Twist to Wheel Speeds Converter

Converts /cmd_vel (Twist) to /cmd_wheels (TankDriveTarget)
Uses differential drive kinematics to compute left/right wheel speeds

For a differential drive robot:
  v_left  = v - ω * (track_width / 2)
  v_right = v + ω * (track_width / 2)

Where:
  v = linear velocity (m/s)
  ω = angular velocity (rad/s)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from urc_msgs.msg import TankDriveTarget


class TwistToWheels(Node):
    def __init__(self):
        super().__init__('twist_to_wheels')
        
        # Declare parameters
        self.declare_parameter('track_width_m', 0.42)         # Distance between wheel centers
        self.declare_parameter('max_wheel_speed_mps', 2.0)    # Safety limit for rim speed
        
        # Get parameters
        self.track_width = self.get_parameter('track_width_m').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_wheel_speed_mps').get_parameter_value().double_value
        
        # Half track width (used in calculations)
        self.half_track = self.track_width / 2.0
        
        # Publisher and subscriber
        self.pub = self.create_publisher(TankDriveTarget, '/cmd_wheels', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        
        self.get_logger().info(
            f'Twist→Wheels converter started: track={self.track_width:.3f}m, '
            f'max_speed={self.max_speed:.2f}m/s'
        )
    
    def clamp(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min_val, min(max_val, value))
    
    def twist_callback(self, msg: Twist):
        """Convert Twist to wheel speeds using differential drive kinematics"""
        
        # Extract velocities
        v = msg.linear.x      # Forward velocity (m/s)
        omega = msg.angular.z # Angular velocity (rad/s), positive = CCW
        
        # Differential drive kinematics
        # Left wheel needs to go slower when turning left (negative omega effect)
        # Right wheel needs to go faster when turning left (positive omega effect)
        v_left  = v - omega * self.half_track
        v_right = v + omega * self.half_track
        
        # Clamp to safe limits
        v_left  = self.clamp(v_left,  -self.max_speed, self.max_speed)
        v_right = self.clamp(v_right, -self.max_speed, self.max_speed)
        
        # Publish wheel speeds
        wheel_msg = TankDriveTarget()
        wheel_msg.left_speed = v_left
        wheel_msg.right_speed = v_right
        self.pub.publish(wheel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToWheels()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

