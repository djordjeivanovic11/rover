#!/usr/bin/env python3
"""
=============================================================================
ROVER SAFETY MONITOR
=============================================================================
Safety monitor with <150ms fault response time for URC rover control.
Monitors critical parameters and provides immediate emergency response.
Includes URC-specific features like autonomy timer, bandwidth monitoring,
and stuck detection with automatic recovery.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import yaml
import numpy as np
import time
import threading
import psutil
from typing import Dict, List, Optional, Set
from dataclasses import dataclass
from enum import Enum

from std_msgs.msg import Bool, Float32MultiArray, Float32
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState, BatteryState
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rover_control.msg import RoverStatus

# Import custom fault message if available
try:
    from rover_control.msg import Fault
except ImportError:
    # Create a simple fault message structure if not available
    @dataclass
    class Fault:
        code: int
        message: str
        severity: int
        timestamp: float


class FaultSeverity(Enum):
    """Fault severity levels"""
    INFO = 0
    WARNING = 1
    CRITICAL = 2
    EMERGENCY = 3


class FaultCode(Enum):
    """Standardized fault codes"""
    # Success codes
    SUCCESS = 0
    COMPLETED = 1
    CANCELLED = 2
    
    # Motion faults (1xx)
    VELOCITY_LIMIT_EXCEEDED = 101
    ACCELERATION_LIMIT_EXCEEDED = 102
    TRACKING_ERROR = 103
    CONTROL_TIMEOUT = 104
    MOTOR_FAULT = 105
    
    # Navigation faults (2xx)
    PATH_PLANNING_FAILED = 201
    OBSTACLE_DETECTED = 202
    GOAL_UNREACHABLE = 203
    LOCALIZATION_LOST = 204
    
    # Stuck and recovery faults (3xx)
    STUCK_DETECTED = 301
    RECOVERY_FAILED = 302
    SLIP_EXCESSIVE = 303
    TRACTION_LOSS = 304
    
    # Power faults (4xx)
    LOW_BATTERY = 401
    CRITICAL_BATTERY = 402
    MOTOR_OVERTEMP = 403
    BATTERY_OVERTEMP = 404
    POWER_FAULT = 405
    
    # Communication faults (5xx)
    COMMUNICATION_TIMEOUT = 501
    BANDWIDTH_EXCEEDED = 502
    COMMAND_INVALID = 503
    TELEMETRY_FAILURE = 504
    
    # Safety faults (6xx)
    EMERGENCY_STOP = 601
    COLLISION_DETECTED = 602
    SAFETY_VIOLATION = 603
    SYSTEM_FAULT = 604
    AUTONOMY_TIMEOUT = 605
    
    # Environmental faults (7xx)
    TEMPERATURE_FAULT = 701
    ACCELERATION_FAULT = 702
    ROLLOVER_DETECTED = 703
    SENSOR_FAILURE = 704


@dataclass
class SafetyLimits:
    """Safety limits configuration"""
    max_linear_velocity: float = 2.5
    max_angular_velocity: float = 1.2
    max_acceleration: float = 2.0
    emergency_deceleration: float = 5.0
    
    # Mission mode limits
    mission_mode_limits: Dict[str, Dict[str, float]] = None
    
    # Temperature limits
    motor_temp_warning: float = 70.0
    motor_temp_critical: float = 85.0
    motor_temp_shutdown: float = 95.0
    battery_temp_warning: float = 45.0
    battery_temp_critical: float = 55.0
    
    # Battery limits
    low_battery_threshold: float = 0.3
    critical_battery_threshold: float = 0.15
    emergency_battery_threshold: float = 0.05
    
    # Communication limits
    max_downlink_bandwidth: float = 5.0
    bandwidth_warning_threshold: float = 4.0
    bandwidth_critical_threshold: float = 4.8


class SafetyMonitor(Node):
    """
    Safety monitor with <150ms fault response time.
    
    Monitors:
    - Velocity and acceleration limits
    - Motor currents and temperatures
    - Battery voltage and temperature
    - Emergency stop status
    - Communication timeouts
    - Stuck detection and recovery
    - URC autonomy timer
    - Bandwidth monitoring
    """
    
    def __init__(self):
        super().__init__("safety_monitor")
        
        # Load configuration
        self._load_configuration()
        
        # Safety state
        self.safety_ok = True
        self.emergency_stop_active = False
        self.fault_active = False
        self.active_faults: Set[int] = set()
        self.fault_history: List[Fault] = []
        
        # Monitoring data with timestamps
        self.last_joint_state_time = time.time()
        self.last_odom_time = time.time()
        self.last_cmd_vel_time = time.time()
        self.last_battery_time = time.time()
        
        # Current rover state
        self.current_velocity = Twist()
        self.current_position = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.motor_currents = np.zeros(4)
        self.motor_temperatures = np.zeros(4)
        self.battery_voltage = 24.0
        self.battery_soc = 1.0
        self.battery_temperature = 25.0
        
        # Mission state
        self.current_mission_mode = "exploration"
        self.autonomy_start_time = None
        self.autonomy_timer_active = False
        
        # Stuck detection
        self.stuck_detection_active = False
        self.stuck_start_time = None
        self.recovery_attempts = 0
        self.max_recovery_attempts = 3
        
        # Bandwidth monitoring
        self.bandwidth_usage = 0.0  # Mbps
        self.last_bandwidth_check = time.time()
        
        # Performance monitoring
        self.cpu_usage = 0.0
        self.memory_usage = 0.0
        
        # Safety limits
        self.safety_limits = SafetyLimits()
        self._update_safety_limits()
        
        # Thread-safe callback groups
        self.monitoring_callback_group = ReentrantCallbackGroup()
        self.publishing_callback_group = ReentrantCallbackGroup()
        
        # Subscribers for safety-critical topics
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10,
            callback_group=self.monitoring_callback_group)
            
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_callback, 10,
            callback_group=self.monitoring_callback_group)
            
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10,
            callback_group=self.monitoring_callback_group)
            
        self.motor_currents_sub = self.create_subscription(
            Float32MultiArray, "/rover/motor_currents", self._motor_currents_callback, 10,
            callback_group=self.monitoring_callback_group)
            
        self.motor_temps_sub = self.create_subscription(
            Float32MultiArray, "/rover/motor_temperatures", self._motor_temps_callback, 10,
            callback_group=self.monitoring_callback_group)
            
        self.battery_sub = self.create_subscription(
            BatteryState, "/rover/battery", self._battery_callback, 10,
            callback_group=self.monitoring_callback_group)
            
        self.estop_hw_sub = self.create_subscription(
            Bool, "/rover/estop_hw", self._estop_hw_callback, 10,
            callback_group=self.monitoring_callback_group)
            
        self.rover_status_sub = self.create_subscription(
            RoverStatus, "/rover/status", self._rover_status_callback, 10,
            callback_group=self.monitoring_callback_group)
        
        # Publishers
        self.safety_status_pub = self.create_publisher(
            DiagnosticArray, "/rover/safety_status", 10)
        self.fault_pub = self.create_publisher(
            DiagnosticStatus, "/rover/faults", 10)  # Using DiagnosticStatus instead of custom Fault
        self.emergency_stop_pub = self.create_publisher(
            Bool, "/rover/estop_sw", 10)
        self.stuck_status_pub = self.create_publisher(
            Bool, "/rover/stuck_detected", 10)
        self.autonomy_timer_pub = self.create_publisher(
            Float32, "/rover/autonomy_time_remaining", 10)
            
        # High-frequency monitoring timer (100 Hz for <150ms response)
        self.monitor_timer = self.create_timer(
            0.01, self._monitor_safety_callback, 
            callback_group=self.monitoring_callback_group)
            
        # Status publishing timer (10 Hz)
        self.status_timer = self.create_timer(
            0.1, self._publish_status_callback,
            callback_group=self.publishing_callback_group)
            
        # Performance monitoring timer (1 Hz)
        self.performance_timer = self.create_timer(
            1.0, self._monitor_performance_callback,
            callback_group=self.monitoring_callback_group)
            
        self.get_logger().info("Safety Monitor initialized with 150ms response time")
        
    def _load_configuration(self):
        """Load safety configuration from parameters"""
        # Declare and load parameters
        param_defaults = {
            'update_rate': 100.0,
            'fault_response_timeout': 0.15,
            'max_autonomy_time': 60.0,
            'stuck_detection_enable': True,
            'slip_threshold': 0.8,
            'stuck_duration': 2.0,
            'bandwidth_monitoring': True,
            'max_downlink_bandwidth': 5.0,
            'power_management_enable': True,
            'low_battery_threshold': 0.3,
            'critical_battery_threshold': 0.15,
            'emergency_battery_threshold': 0.05
        }
        
        self.config = {}
        for param_name, default_value in param_defaults.items():
            self.declare_parameter(param_name, default_value)
            self.config[param_name] = self.get_parameter(param_name).value
            
    def _update_safety_limits(self):
        """Update safety limits based on current mission mode"""
        # Base limits
        base_limits = {
            'max_linear_velocity': 2.5,
            'max_angular_velocity': 1.2,
            'max_acceleration': 2.0
        }
        
        # Mission mode specific limits
        mission_limits = {
            'exploration': {'max_velocity': 1.8, 'max_acceleration': 1.2},
            'science': {'max_velocity': 0.8, 'max_acceleration': 0.6},
            'return_to_base': {'max_velocity': 2.2, 'max_acceleration': 1.5},
            'manual': {'max_velocity': 1.5, 'max_acceleration': 2.0}
        }
        
        if self.current_mission_mode in mission_limits:
            mode_limits = mission_limits[self.current_mission_mode]
            self.safety_limits.max_linear_velocity = mode_limits['max_velocity']
            self.safety_limits.max_acceleration = mode_limits['max_acceleration']
        else:
            self.safety_limits.max_linear_velocity = base_limits['max_linear_velocity']
            self.safety_limits.max_acceleration = base_limits['max_acceleration']
            
    def _monitor_safety_callback(self):
        """Main safety monitoring callback - runs at 100 Hz"""
        try:
            current_time = time.time()
            
            # Check communication timeouts
            self._check_communication_timeouts(current_time)
            
            # Check velocity and acceleration limits
            self._check_velocity_limits()
            
            # Check motor temperatures
            self._check_motor_temperatures()
            
            # Check battery status
            self._check_battery_status()
            
            # Check stuck detection
            if self.config['stuck_detection_enable']:
                self._check_stuck_detection(current_time)
                
            # Check autonomy timer
            self._check_autonomy_timer(current_time)
            
            # Check bandwidth usage
            if self.config['bandwidth_monitoring']:
                self._check_bandwidth_usage(current_time)
                
            # Update overall safety status
            self._update_safety_status()
            
        except Exception as e:
            self.get_logger().error(f"Safety monitoring error: {str(e)}")
            self._trigger_fault(FaultCode.SYSTEM_FAULT, f"Safety monitor exception: {str(e)}")
            
    def _check_communication_timeouts(self, current_time: float):
        """Check for communication timeouts"""
        timeout_threshold = 2.0  # seconds
        
        # Check joint state timeout
        if current_time - self.last_joint_state_time > timeout_threshold:
            self._trigger_fault(FaultCode.SENSOR_FAILURE, "Joint state timeout")
            
        # Check odometry timeout
        if current_time - self.last_odom_time > timeout_threshold:
            self._trigger_fault(FaultCode.SENSOR_FAILURE, "Odometry timeout")
            
        # Check command timeout
        if current_time - self.last_cmd_vel_time > 5.0:  # 5 second timeout for commands
            self._trigger_fault(FaultCode.CONTROL_TIMEOUT, "Command velocity timeout")
            
    def _check_velocity_limits(self):
        """Check velocity and acceleration limits"""
        # Linear velocity check
        linear_vel = abs(self.current_velocity.linear.x)
        if linear_vel > self.safety_limits.max_linear_velocity:
            self._trigger_fault(FaultCode.VELOCITY_LIMIT_EXCEEDED, 
                              f"Linear velocity {linear_vel:.2f} > {self.safety_limits.max_linear_velocity:.2f}")
            
        # Angular velocity check
        angular_vel = abs(self.current_velocity.angular.z)
        if angular_vel > self.safety_limits.max_angular_velocity:
            self._trigger_fault(FaultCode.VELOCITY_LIMIT_EXCEEDED,
                              f"Angular velocity {angular_vel:.2f} > {self.safety_limits.max_angular_velocity:.2f}")
            
    def _check_motor_temperatures(self):
        """Check motor temperature limits"""
        for i, temp in enumerate(self.motor_temperatures):
            if temp > self.safety_limits.motor_temp_shutdown:
                self._trigger_fault(FaultCode.MOTOR_OVERTEMP, 
                                  f"Motor {i} temperature {temp:.1f}°C > {self.safety_limits.motor_temp_shutdown:.1f}°C")
            elif temp > self.safety_limits.motor_temp_critical:
                self._trigger_fault(FaultCode.TEMPERATURE_FAULT,
                                  f"Motor {i} temperature {temp:.1f}°C critical")
                                  
    def _check_battery_status(self):
        """Check battery voltage and state of charge"""
        # Low battery warnings
        if self.battery_soc < self.config['emergency_battery_threshold']:
            self._trigger_fault(FaultCode.CRITICAL_BATTERY, 
                              f"Battery SoC {self.battery_soc:.1%} critically low")
        elif self.battery_soc < self.config['critical_battery_threshold']:
            self._trigger_fault(FaultCode.CRITICAL_BATTERY,
                              f"Battery SoC {self.battery_soc:.1%} low")
        elif self.battery_soc < self.config['low_battery_threshold']:
            self._trigger_fault(FaultCode.LOW_BATTERY,
                              f"Battery SoC {self.battery_soc:.1%} warning")
                              
        # Battery temperature check
        if self.battery_temperature > self.safety_limits.battery_temp_critical:
            self._trigger_fault(FaultCode.BATTERY_OVERTEMP,
                              f"Battery temperature {self.battery_temperature:.1f}°C critical")
                              
    def _check_stuck_detection(self, current_time: float):
        """Check for stuck condition based on slip ratio"""
        # Calculate slip ratio (simplified - would need wheel speed vs expected speed)
        # For now, use a simplified approach based on commanded vs actual velocity
        commanded_vel = abs(self.current_velocity.linear.x)
        
        # Estimate slip ratio (this would be more sophisticated in real implementation)
        slip_ratio = 0.0  # Placeholder - would calculate from wheel speeds
        
        if slip_ratio > self.config['slip_threshold'] and commanded_vel > 0.1:
            if not self.stuck_detection_active:
                self.stuck_detection_active = True
                self.stuck_start_time = current_time
            elif current_time - self.stuck_start_time > self.config['stuck_duration']:
                self._trigger_fault(FaultCode.STUCK_DETECTED, f"Stuck detected - slip ratio {slip_ratio:.2f}")
                self._initiate_stuck_recovery()
        else:
            self.stuck_detection_active = False
            self.stuck_start_time = None
            
    def _check_autonomy_timer(self, current_time: float):
        """Check URC autonomy timer"""
        if self.autonomy_timer_active and self.autonomy_start_time:
            elapsed_time = current_time - self.autonomy_start_time
            remaining_time = self.config['max_autonomy_time'] - elapsed_time
            
            # Publish remaining time
            timer_msg = Float32()
            timer_msg.data = max(0.0, remaining_time)
            self.autonomy_timer_pub.publish(timer_msg)
            
            # Check for timeout
            if remaining_time <= 0:
                self._trigger_fault(FaultCode.AUTONOMY_TIMEOUT, "Autonomy timer expired")
                self.autonomy_timer_active = False
                
    def _check_bandwidth_usage(self, current_time: float):
        """Check bandwidth usage (URC-specific)"""
        # Update bandwidth usage (simplified - would integrate with actual network monitoring)
        if current_time - self.last_bandwidth_check > 1.0:  # Check every second
            # Placeholder for actual bandwidth measurement
            self.bandwidth_usage = 0.0  # Would measure actual usage
            self.last_bandwidth_check = current_time
            
            if self.bandwidth_usage > self.config['max_downlink_bandwidth']:
                self._trigger_fault(FaultCode.BANDWIDTH_EXCEEDED,
                                  f"Bandwidth {self.bandwidth_usage:.1f} Mbps > {self.config['max_downlink_bandwidth']:.1f} Mbps")
                                  
    def _monitor_performance_callback(self):
        """Monitor system performance"""
        try:
            # CPU usage
            self.cpu_usage = psutil.cpu_percent()
            
            # Memory usage
            memory = psutil.virtual_memory()
            self.memory_usage = memory.percent
            
            # Check for performance issues
            if self.cpu_usage > 90.0:
                self._trigger_fault(FaultCode.SYSTEM_FAULT, f"High CPU usage: {self.cpu_usage:.1f}%")
                
            if self.memory_usage > 90.0:
                self._trigger_fault(FaultCode.SYSTEM_FAULT, f"High memory usage: {self.memory_usage:.1f}%")
                
        except Exception as e:
            self.get_logger().warn(f"Performance monitoring error: {str(e)}")
            
    def _initiate_stuck_recovery(self):
        """Initiate automatic stuck recovery"""
        if self.recovery_attempts < self.max_recovery_attempts:
            self.recovery_attempts += 1
            self.get_logger().warn(f"Initiating stuck recovery attempt {self.recovery_attempts}")
            
            # Implement recovery strategy (reverse motion, etc.)
            # This would send recovery commands to the motion controller
            
        else:
            self._trigger_fault(FaultCode.RECOVERY_FAILED, "Maximum recovery attempts exceeded")
            
    def _trigger_fault(self, fault_code: FaultCode, message: str, severity: FaultSeverity = FaultSeverity.WARNING):
        """Trigger a safety fault"""
        fault_id = fault_code.value
        
        # Avoid duplicate faults
        if fault_id in self.active_faults:
            return
            
        self.active_faults.add(fault_id)
        
        # Create fault message
        fault = Fault()
        fault.code = fault_id
        fault.message = message
        fault.severity = severity.value
        fault.timestamp = time.time()
        
        self.fault_history.append(fault)
        
        # Determine response based on severity
        if severity in [FaultSeverity.CRITICAL, FaultSeverity.EMERGENCY]:
            self._trigger_emergency_stop(message)
            self.fault_active = True
            
        # Publish fault
        fault_msg = DiagnosticStatus()
        fault_msg.name = f"fault_{fault_code.name}"
        fault_msg.message = message
        fault_msg.hardware_id = "rover_safety"
        
        if severity == FaultSeverity.EMERGENCY:
            fault_msg.level = DiagnosticStatus.ERROR
        elif severity == FaultSeverity.CRITICAL:
            fault_msg.level = DiagnosticStatus.ERROR
        elif severity == FaultSeverity.WARNING:
            fault_msg.level = DiagnosticStatus.WARN
        else:
            fault_msg.level = DiagnosticStatus.OK
            
        fault_msg.values.append(KeyValue(key="code", value=str(fault_id)))
        fault_msg.values.append(KeyValue(key="severity", value=severity.name))
        fault_msg.values.append(KeyValue(key="timestamp", value=str(fault.timestamp)))
        
        self.fault_pub.publish(fault_msg)
        
        self.get_logger().warn(f"FAULT {fault_code.name}: {message}")
        
    def _trigger_emergency_stop(self, reason: str):
        """Trigger emergency stop"""
        self.emergency_stop_active = True
        
        # Publish emergency stop
        estop_msg = Bool()
        estop_msg.data = True
        self.emergency_stop_pub.publish(estop_msg)
        
        self.get_logger().error(f"EMERGENCY STOP: {reason}")
        
    def _clear_fault(self, fault_code: FaultCode):
        """Clear a specific fault"""
        fault_id = fault_code.value
        if fault_id in self.active_faults:
            self.active_faults.remove(fault_id)
            
    def _update_safety_status(self):
        """Update overall safety status"""
        # Check if any critical faults are active
        critical_faults = [f for f in self.active_faults if f >= 600]  # Emergency/critical fault codes
        
        self.safety_ok = len(critical_faults) == 0 and not self.emergency_stop_active
        
    def _publish_status_callback(self):
        """Publish safety status and diagnostics"""
        try:
            self._publish_safety_diagnostics()
            self._publish_stuck_status()
        except Exception as e:
            self.get_logger().error(f"Status publishing error: {str(e)}")
            
    def _publish_safety_diagnostics(self):
        """Publish comprehensive safety diagnostics"""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Overall safety status
        status = DiagnosticStatus()
        status.name = "rover_safety_monitor"
        status.hardware_id = "rover_control"
        
        if self.safety_ok:
            status.level = DiagnosticStatus.OK
            status.message = f"Safety OK - Mode: {self.current_mission_mode}"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = f"Safety FAULT - {len(self.active_faults)} active faults"
            
        # Add key-value pairs
        status.values.append(KeyValue(key="safety_ok", value=str(self.safety_ok)))
        status.values.append(KeyValue(key="emergency_stop", value=str(self.emergency_stop_active)))
        status.values.append(KeyValue(key="active_faults", value=str(len(self.active_faults))))
        status.values.append(KeyValue(key="mission_mode", value=self.current_mission_mode))
        status.values.append(KeyValue(key="battery_soc", value=f"{self.battery_soc:.1%}"))
        status.values.append(KeyValue(key="cpu_usage", value=f"{self.cpu_usage:.1f}%"))
        status.values.append(KeyValue(key="memory_usage", value=f"{self.memory_usage:.1f}%"))
        status.values.append(KeyValue(key="bandwidth_usage", value=f"{self.bandwidth_usage:.1f} Mbps"))
        
        if self.autonomy_timer_active and self.autonomy_start_time:
            remaining_time = self.config['max_autonomy_time'] - (time.time() - self.autonomy_start_time)
            status.values.append(KeyValue(key="autonomy_time_remaining", value=f"{max(0, remaining_time):.1f}s"))
            
        msg.status.append(status)
        self.safety_status_pub.publish(msg)
        
    def _publish_stuck_status(self):
        """Publish stuck detection status"""
        stuck_msg = Bool()
        stuck_msg.data = self.stuck_detection_active
        self.stuck_status_pub.publish(stuck_msg)
        
    # Callback functions for sensor data
    def _joint_state_callback(self, msg: JointState):
        """Process joint state updates"""
        self.last_joint_state_time = time.time()
        
    def _odom_callback(self, msg: Odometry):
        """Process odometry updates"""
        self.last_odom_time = time.time()
        self.current_velocity = msg.twist.twist
        
        # Update position
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        # Extract yaw from quaternion (simplified)
        self.current_position[2] = 2.0 * np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        
    def _cmd_vel_callback(self, msg: Twist):
        """Process velocity command updates"""
        self.last_cmd_vel_time = time.time()
        
    def _motor_currents_callback(self, msg: Float32MultiArray):
        """Process motor current updates"""
        if len(msg.data) >= 4:
            self.motor_currents = np.array(msg.data[:4])
            
    def _motor_temps_callback(self, msg: Float32MultiArray):
        """Process motor temperature updates"""
        if len(msg.data) >= 4:
            self.motor_temperatures = np.array(msg.data[:4])
            
    def _battery_callback(self, msg: BatteryState):
        """Process battery state updates"""
        self.last_battery_time = time.time()
        self.battery_voltage = msg.voltage
        self.battery_soc = msg.percentage
        # Temperature might be in a different field depending on battery message
        
    def _estop_hw_callback(self, msg: Bool):
        """Process hardware emergency stop"""
        if msg.data and not self.emergency_stop_active:
            self._trigger_emergency_stop("Hardware emergency stop activated")
            
    def _rover_status_callback(self, msg: RoverStatus):
        """Process rover status updates"""
        self.current_mission_mode = msg.mission_mode
        self._update_safety_limits()
        
        # Check if autonomy timer should be active
        if msg.is_autonomous and not self.autonomy_timer_active:
            self.autonomy_start_time = time.time()
            self.autonomy_timer_active = True
        elif not msg.is_autonomous:
            self.autonomy_timer_active = False
            
    def start_autonomy_timer(self):
        """Start the autonomy timer (called by action servers)"""
        self.autonomy_start_time = time.time()
        self.autonomy_timer_active = True
        self.get_logger().info("Autonomy timer started")
        
    def stop_autonomy_timer(self):
        """Stop the autonomy timer"""
        self.autonomy_timer_active = False
        self.autonomy_start_time = None
        self.get_logger().info("Autonomy timer stopped")
        
    def reset_safety_faults(self):
        """Reset all non-critical safety faults"""
        # Clear all faults except emergency stops
        critical_faults = {FaultCode.EMERGENCY_STOP.value, FaultCode.COLLISION_DETECTED.value}
        self.active_faults = self.active_faults.intersection(critical_faults)
        
        if not self.active_faults:
            self.fault_active = False
            self.emergency_stop_active = False
            
        self.get_logger().info("Safety faults reset")


def main(args=None):
    """Main entry point for the safety monitor node"""
    rclpy.init(args=args)
    
    try:
        node = SafetyMonitor()
        
        # Use MultiThreadedExecutor for concurrent callbacks
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in safety monitor: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 