#!/usr/bin/env python3
"""
=============================================================================
SIMPLE SAFETY MONITOR FOR URC ROVER ARM
=============================================================================
Basic safety monitoring:
- Emergency stop handling
- Joint limit violations
- Simple fault reporting

No complex monitoring - just essential safety for URC.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from arm_control.msg import Fault
import time


class SafetyMonitor(Node):
    """
    Simple safety monitoring for the rover arm.
    Monitors E-stop and basic faults.
    """
    
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Joint limits (from URDF/config)
        self.joint_limits = {
            'AB_Rev': {'min': -3.1416, 'max': 3.1416, 'velocity': 1.0},
            'AS1_Rev': {'min': -2.618, 'max': 2.618, 'velocity': 1.0},
            'AW_Rev': {'min': -3.1416, 'max': 3.1416, 'velocity': 1.5},
            'AM_Rev': {'min': -3.1416, 'max': 3.1416, 'velocity': 2.0},
        }
        
        # Safety state
        self.estop_active = False
        self.fault_active = False
        self.current_joint_states = {}
        
        # Subscribers
        self.estop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self._estop_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        # Publishers
        self.fault_pub = self.create_publisher(Fault, '/arm/faults', 10)
        self.safe_pub = self.create_publisher(Bool, '/arm/is_safe', 10)
        
        # Monitor timer
        self.monitor_timer = self.create_timer(0.1, self._monitor_safety)  # 10 Hz
        
        self.get_logger().info('Safety Monitor active')
        self.get_logger().info('Monitoring: E-stop, joint limits')
    
    def _estop_callback(self, msg):
        """Handle emergency stop signal"""
        if msg.data and not self.estop_active:
            self.get_logger().error('🚨 EMERGENCY STOP ACTIVATED 🚨')
            self.estop_active = True
            self._publish_fault('ESTOP', 'Emergency stop activated', critical=True)
        elif not msg.data and self.estop_active:
            self.get_logger().info('Emergency stop released')
            self.estop_active = False
    
    def _joint_state_callback(self, msg):
        """Update current joint states"""
        for i, name in enumerate(msg.name):
            if name in self.joint_limits:
                self.current_joint_states[name] = {
                    'position': msg.position[i] if i < len(msg.position) else 0.0,
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                }
    
    def _monitor_safety(self):
        """Main safety monitoring loop"""
        is_safe = True
        
        # Check E-stop
        if self.estop_active:
            is_safe = False
        
        # Check joint limits
        for joint_name, state in self.current_joint_states.items():
            if joint_name in self.joint_limits:
                limits = self.joint_limits[joint_name]
                
                # Position limits
                if state['position'] < limits['min'] or state['position'] > limits['max']:
                    self.get_logger().error(
                        f'Joint {joint_name} position limit violated: {state["position"]:.3f} '
                        f'(limits: [{limits["min"]:.3f}, {limits["max"]:.3f}])'
                    )
                    self._publish_fault(
                        'JOINT_LIMIT',
                        f'{joint_name} position out of range',
                        critical=False
                    )
                    is_safe = False
                
                # Velocity limits
                if abs(state['velocity']) > limits['velocity']:
                    self.get_logger().warn(
                        f'Joint {joint_name} velocity high: {state["velocity"]:.3f} rad/s'
                    )
        
        # Publish safety status
        safety_msg = Bool()
        safety_msg.data = is_safe
        self.safe_pub.publish(safety_msg)
    
    def _publish_fault(self, fault_code, message, critical=False):
        """Publish a fault message"""
        try:
            from arm_control.msg import Fault
            fault_msg = Fault()
            fault_msg.timestamp = self.get_clock().now().to_msg()
            fault_msg.fault_code = fault_code
            fault_msg.message = message
            fault_msg.critical = critical
            
            self.fault_pub.publish(fault_msg)
        except ImportError:
            self.get_logger().error('Fault message not generated yet. Build the package first.')
        
        if critical:
            self.get_logger().error(f'🚨 CRITICAL FAULT: {fault_code} - {message}')
        else:
            self.get_logger().warn(f'⚠️  FAULT: {fault_code} - {message}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = SafetyMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
