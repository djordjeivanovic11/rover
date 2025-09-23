#!/usr/bin/env python3
"""
Simple Keyboard Teleop for Rover Testing
Use WASD keys to drive the rover
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Speed settings
        self.linear_speed = 0.5   # m/s
        self.angular_speed = 1.0  # rad/s
        
        self.get_logger().info("Keyboard Teleop Ready!")
        self.print_instructions()
    
    def print_instructions(self):
        print("\n" + "="*50)
        print("ROVER KEYBOARD TELEOP")
        print("="*50)
        print("W/S : Forward/Backward")
        print("A/D : Turn Left/Right") 
        print("Q/E : Increase/Decrease Linear Speed")
        print("Z/C : Increase/Decrease Angular Speed")
        print("SPACE : Stop")
        print("X : Quit")
        print(f"\nCurrent speeds: Linear={self.linear_speed:.1f} m/s, Angular={self.angular_speed:.1f} rad/s")
        print("="*50)
    
    def get_key(self):
        """Get a single keypress"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None
    
    def send_twist(self, linear=0.0, angular=0.0):
        """Send a Twist message"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)
        
        # Show what we're sending
        if linear != 0.0 or angular != 0.0:
            print(f"Sending: Linear={linear:.2f}, Angular={angular:.2f}")
        else:
            print("STOP")


def main():
    rclpy.init()
    
    # Set up terminal for single key input
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    
    teleop = KeyboardTeleop()
    
    try:
        while rclpy.ok():
            key = teleop.get_key()
            
            if key:
                key = key.lower()
                
                if key == 'w':
                    teleop.send_twist(teleop.linear_speed, 0.0)
                elif key == 's':
                    teleop.send_twist(-teleop.linear_speed, 0.0)
                elif key == 'a':
                    teleop.send_twist(0.0, teleop.angular_speed)
                elif key == 'd':
                    teleop.send_twist(0.0, -teleop.angular_speed)
                elif key == ' ':
                    teleop.send_twist(0.0, 0.0)
                elif key == 'q':
                    teleop.linear_speed += 0.1
                    print(f"Linear speed: {teleop.linear_speed:.1f} m/s")
                elif key == 'e':
                    teleop.linear_speed = max(0.1, teleop.linear_speed - 0.1)
                    print(f"Linear speed: {teleop.linear_speed:.1f} m/s")
                elif key == 'z':
                    teleop.angular_speed += 0.1
                    print(f"Angular speed: {teleop.angular_speed:.1f} rad/s")
                elif key == 'c':
                    teleop.angular_speed = max(0.1, teleop.angular_speed - 0.1)
                    print(f"Angular speed: {teleop.angular_speed:.1f} rad/s")
                elif key == 'x':
                    teleop.send_twist(0.0, 0.0)  # Stop before quitting
                    break
            
            # Spin once to process callbacks
            rclpy.spin_once(teleop, timeout_sec=0.01)
    
    except KeyboardInterrupt:
        pass
    
    finally:
        # Send stop command
        teleop.send_twist(0.0, 0.0)
        
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        teleop.destroy_node()
        rclpy.shutdown()
        print("\nKeyboard teleop stopped.")


if __name__ == '__main__':
    main() 