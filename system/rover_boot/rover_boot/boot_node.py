#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rover_boot.failsafes import Failsafes

class BootNode(Node):
    def __init__(self):
        super().__init__('boot_node')
        self.get_logger().info('✅ BootNode online — initiating failsafes and diagnostics')
        self.failsafes = Failsafes(self)

def main(args=None):
    rclpy.init(args=args)
    node = BootNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
