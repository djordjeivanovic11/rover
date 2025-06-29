#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from urc_msgs.msg import GapCmd

SAFE_STOP_DIST = 0.6  # meters

class NavSupervisor(Node):
    """
    Muxes between Nav2's /cmd_vel_nav2 and gap-guidance /gap_cmd,
    publishes the final /cmd_vel to drive the rover safely.
    """
    def __init__(self):
        super().__init__('nav_supervisor')

        self.nav_vel = Twist()
        self.gap_cmd = GapCmd()
        self.gap_valid = False

        # Subscribe to Nav2 output and gap-guidance
        self.create_subscription(
            Twist, '/cmd_vel_nav2', self.cb_nav, 10)
        self.create_subscription(
            GapCmd, '/gap_cmd', self.cb_gap, 10)

        # Publisher for actual drive commands
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 20 Hz arbitration
        self.create_timer(0.05, self.tick)

    def cb_nav(self, msg: Twist):
        self.nav_vel = msg

    def cb_gap(self, msg: GapCmd):
        self.gap_cmd = msg
        self.gap_valid = True

    def tick(self):
        out = Twist()

        # If gap guidance sees obstacle too close, override steering/brake
        if (self.gap_valid and
            self.gap_cmd.clear_dist < SAFE_STOP_DIST):
            out.linear.x  = min(self.nav_vel.linear.x, 0.2)
            out.angular.z = self.gap_cmd.steer_angle
        else:
            out = self.nav_vel

        self.pub.publish(out)
        self.gap_valid = False  # require fresh gap every cycle

def main(args=None):
    rclpy.init(args=args)
    node = NavSupervisor()
    rclpy.spin(node)
    rclpy.shutdown()
