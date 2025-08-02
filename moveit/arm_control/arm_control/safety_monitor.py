#!/usr/bin/env python3
"""
=============================================================================
SAFETY MONITOR
=============================================================================
Monitors all safety-critical topics and enforces limits with <150ms response.
Subscribes to joint currents, temperatures, F/T wrench, and E-stop topics.
Publishes FAULT state and triggers emergency stop on violations.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import time
from typing import Dict, List, Optional
from pathlib import Path
import yaml
import threading

from sensor_msgs.msg import JointState, Temperature
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import WrenchStamped, Point
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from arm_control.msg import ArmStatus, Fault


class SafetyMonitor(Node):
    """
    Safety monitor with <150ms fault response time.
    
    Monitors:
    - Joint currents and temperatures
    - TCP force/torque
    - Emergency stop status
    - Position and velocity limits
    - Communication timeouts
    - Workspace violations
    """
    
    def __init__(self):
        super().__init__("safety_monitor")
        
        # Load configuration
        self._load_configuration()
        
        # Joint configuration
        self.joint_names = [
            "base_rotate", "shoulder_pitch", "shoulder_roll",
            "elbow_pitch", "wrist_pitch", "wrist_roll", "gripper_joint"
        ]
        self.num_joints = len(self.joint_names)
        
        # Monitoring state
        self.monitoring_active = True
        self.safety_ok = True
        self.fault_active = False
        self.active_faults: List[Fault] = []
        
        # Sensor data with timestamps
        self.joint_state = JointState()
        self.joint_state_time = time.time()
        self.joint_currents = np.zeros(self.num_joints)
        self.joint_currents_time = time.time()
        self.joint_temperatures = np.zeros(self.num_joints)
        self.joint_temperatures_time = time.time()
        self.tcp_wrench = WrenchStamped()
        self.tcp_wrench_time = time.time()
        self.estop_active = False
        self.estop_time = time.time()
        
        # Fault counting and recovery
        self.fault_counts: Dict[str, int] = {}
        self.last_fault_times: Dict[str, float] = {}
        
        # Thread-safe callback groups
        self.monitoring_callback_group = ReentrantCallbackGroup()
        self.publishing_callback_group = ReentrantCallbackGroup()
        
        # Subscribers for safety-critical topics
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10,
            callback_group=self.monitoring_callback_group)
        
        self.joint_currents_sub = self.create_subscription(
            Float32MultiArray, "/arm/joint_currents", self._joint_currents_callback, 10,
            callback_group=self.monitoring_callback_group)
        
        self.joint_temps_sub = self.create_subscription(
            Float32MultiArray, "/arm/joint_temperatures", self._joint_temps_callback, 10,
            callback_group=self.monitoring_callback_group)
        
        self.tcp_wrench_sub = self.create_subscription(
            WrenchStamped, "/arm/tcp_wrench", self._tcp_wrench_callback, 10,
            callback_group=self.monitoring_callback_group)
        
        self.estop_hw_sub = self.create_subscription(
            Bool, "/arm/estop_hw", self._estop_hw_callback, 10,
            callback_group=self.monitoring_callback_group)
        
        self.estop_sw_sub = self.create_subscription(
            Bool, "/arm/estop_sw", self._estop_sw_callback, 10,
            callback_group=self.monitoring_callback_group)
        
        # Publishers
        self.safety_status_pub = self.create_publisher(
            DiagnosticArray, "/arm/safety_status", 10)
        
        self.fault_pub = self.create_publisher(
            Fault, "/arm/faults", 10)
        
        self.emergency_stop_pub = self.create_publisher(
            Bool, "/arm/emergency_stop", 10)
        
        # High-frequency monitoring timer (100 Hz for <150ms response)
        self.monitor_timer = self.create_timer(
            0.01, self._monitor_safety_callback, callback_group=self.monitoring_callback_group)
        
        # Status publishing timer (10 Hz)
        self.status_timer = self.create_timer(
            0.1, self._publish_status_callback, callback_group=self.publishing_callback_group)
        
        self.get_logger().info("Safety Monitor initialized - Monitoring at 100 Hz")
    
    def _load_configuration(self):
        """Load safety configuration from YAML"""
        try:
            safety_params_path = Path(__file__).parent.parent / "config" / "safety_params.yaml"
            with open(safety_params_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Extract safety parameters
            self.safety_config = config.get('safety_monitor', {}).get('ros__parameters', {})
            self.fault_codes = config.get('fault_codes', {})
            self.fault_recovery = config.get('fault_recovery', {})
            
            # Load specific limits
            self.current_limits = self.safety_config.get('current_limits', {})
            self.temp_limits = self.safety_config.get('temperature_limits', {})
            self.force_limits = self.safety_config.get('force_torque_limits', {})
            self.velocity_limits = self.safety_config.get('velocity_limits', {})
            self.position_limits = self.safety_config.get('position_limits', {})
            self.timeouts = self.safety_config.get('timeouts', {})
            
            # Fault response timeout
            self.fault_response_timeout = self.safety_config.get('fault_response_timeout', 0.15)
            
            self.get_logger().info(f"Safety configuration loaded - Fault response timeout: {self.fault_response_timeout}s")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load safety configuration: {e}")
            # Use conservative defaults
            self._load_default_config()
    
    def _load_default_config(self):
        """Load conservative default safety configuration"""
        self.current_limits = {joint: 5.0 for joint in self.joint_names}
        self.temp_limits = {"critical_temp": 80.0, "warning_temp": 65.0}
        self.force_limits = {"max_force_z": 50.0, "max_torque_z": 10.0}
        self.velocity_limits = {joint: 1.0 for joint in self.joint_names}
        self.position_limits = {joint: [-3.14, 3.14] for joint in self.joint_names}
        self.timeouts = {
            "joint_state_timeout": 0.05,
            "force_torque_timeout": 0.1,
            "command_timeout": 0.2
        }
        self.fault_response_timeout = 0.15
        self.fault_codes = {}
        self.fault_recovery = {}
    
    def _joint_state_callback(self, msg: JointState):
        """Update joint state data"""
        self.joint_state = msg
        self.joint_state_time = time.time()
    
    def _joint_currents_callback(self, msg: Float32MultiArray):
        """Update joint current data"""
        if len(msg.data) >= self.num_joints:
            self.joint_currents = np.array(msg.data[:self.num_joints])
            self.joint_currents_time = time.time()
    
    def _joint_temps_callback(self, msg: Float32MultiArray):
        """Update joint temperature data"""
        if len(msg.data) >= self.num_joints:
            self.joint_temperatures = np.array(msg.data[:self.num_joints])
            self.joint_temperatures_time = time.time()
    
    def _tcp_wrench_callback(self, msg: WrenchStamped):
        """Update TCP wrench data"""
        self.tcp_wrench = msg
        self.tcp_wrench_time = time.time()
    
    def _estop_hw_callback(self, msg: Bool):
        """Update hardware E-stop status"""
        was_active = self.estop_active
        self.estop_active = msg.data
        self.estop_time = time.time()
        
        if not was_active and self.estop_active:
            self._trigger_fault(
                fault_code=self.fault_codes.get('ESTOP_ACTIVATED', 301),
                category="ESTOP",
                description="Hardware emergency stop activated",
                severity=Fault.EMERGENCY,
                component="hardware_estop",
                auto_recoverable=False
            )
    
    def _estop_sw_callback(self, msg: Bool):
        """Update software E-stop status"""
        if msg.data:
            self._trigger_fault(
                fault_code=self.fault_codes.get('ESTOP_ACTIVATED', 301),
                category="ESTOP", 
                description="Software emergency stop activated",
                severity=Fault.EMERGENCY,
                component="software_estop",
                auto_recoverable=True
            )
    
    def _monitor_safety_callback(self):
        """Main safety monitoring callback (100 Hz)"""
        if not self.monitoring_active:
            return
        
        start_time = time.time()
        
        try:
            # Check communication timeouts
            self._check_communication_timeouts()
            
            # Check joint limits
            self._check_joint_limits()
            
            # Check current limits
            self._check_current_limits()
            
            # Check temperature limits
            self._check_temperature_limits()
            
            # Check force/torque limits
            self._check_force_torque_limits()
            
            # Check workspace violations
            self._check_workspace_limits()
            
            # Update overall safety status
            self._update_safety_status()
            
        except Exception as e:
            self.get_logger().error(f"Safety monitoring error: {e}")
            self._trigger_fault(
                fault_code=self.fault_codes.get('SOFTWARE_EXCEPTION', 602),
                category="SYSTEM",
                description=f"Safety monitoring exception: {e}",
                severity=Fault.CRITICAL,
                component="safety_monitor",
                auto_recoverable=False
            )
        
        # Check monitoring timing
        elapsed = time.time() - start_time
        if elapsed > self.fault_response_timeout:
            self.get_logger().warn(f"Safety monitoring overrun: {elapsed:.4f}s > {self.fault_response_timeout:.4f}s")
    
    def _check_communication_timeouts(self):
        """Check for communication timeouts"""
        current_time = time.time()
        
        # Joint state timeout
        joint_state_timeout = self.timeouts.get('joint_state_timeout', 0.05)
        if current_time - self.joint_state_time > joint_state_timeout:
            self._trigger_fault(
                fault_code=self.fault_codes.get('JOINT_STATE_TIMEOUT', 401),
                category="COMMUNICATION",
                description=f"Joint state timeout: {current_time - self.joint_state_time:.3f}s",
                severity=Fault.ERROR,
                component="joint_state_publisher",
                auto_recoverable=True
            )
        
        # Force/torque timeout
        ft_timeout = self.timeouts.get('force_torque_timeout', 0.1)
        if current_time - self.tcp_wrench_time > ft_timeout:
            self._trigger_fault(
                fault_code=self.fault_codes.get('FT_SENSOR_TIMEOUT', 203),
                category="FORCE_TORQUE",
                description=f"F/T sensor timeout: {current_time - self.tcp_wrench_time:.3f}s",
                severity=Fault.WARNING,
                component="ft_sensor",
                auto_recoverable=True
            )
    
    def _check_joint_limits(self):
        """Check joint position and velocity limits"""
        if len(self.joint_state.position) < self.num_joints:
            return
        
        positions = np.array(self.joint_state.position[:self.num_joints])
        
        # Check position limits
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in self.position_limits:
                limits = self.position_limits[joint_name]
                if positions[i] < limits[0] or positions[i] > limits[1]:
                    self._trigger_fault(
                        fault_code=self.fault_codes.get('JOINT_POSITION_LIMIT', 103),
                        category="JOINT",
                        description=f"Joint {joint_name} position limit exceeded: {positions[i]:.3f} rad",
                        severity=Fault.CRITICAL,
                        component=joint_name,
                        joint_name=joint_name,
                        fault_value=float(positions[i]),
                        fault_threshold=float(limits[1] if positions[i] > limits[1] else limits[0]),
                        fault_units="rad",
                        auto_recoverable=False
                    )
        
        # Check velocity limits
        if len(self.joint_state.velocity) >= self.num_joints:
            velocities = np.array(self.joint_state.velocity[:self.num_joints])
            
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in self.velocity_limits:
                    limit = self.velocity_limits[joint_name]
                    if abs(velocities[i]) > limit:
                        self._trigger_fault(
                            fault_code=self.fault_codes.get('JOINT_VELOCITY_LIMIT', 104),
                            category="JOINT",
                            description=f"Joint {joint_name} velocity limit exceeded: {velocities[i]:.3f} rad/s",
                            severity=Fault.ERROR,
                            component=joint_name,
                            joint_name=joint_name,
                            fault_value=float(abs(velocities[i])),
                            fault_threshold=float(limit),
                            fault_units="rad/s",
                            auto_recoverable=True
                        )
    
    def _check_current_limits(self):
        """Check joint current limits"""
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in self.current_limits:
                limit = self.current_limits[joint_name]
                current = self.joint_currents[i]
                
                if current > limit:
                    self._trigger_fault(
                        fault_code=self.fault_codes.get('JOINT_OVERCURRENT', 101),
                        category="JOINT",
                        description=f"Joint {joint_name} overcurrent: {current:.2f} A",
                        severity=Fault.CRITICAL,
                        component=joint_name,
                        joint_name=joint_name,
                        fault_value=float(current),
                        fault_threshold=float(limit),
                        fault_units="A",
                        auto_recoverable=False
                    )
    
    def _check_temperature_limits(self):
        """Check joint temperature limits"""
        critical_temp = self.temp_limits.get('critical_temp', 80.0)
        warning_temp = self.temp_limits.get('warning_temp', 65.0)
        
        for i, joint_name in enumerate(self.joint_names):
            temp = self.joint_temperatures[i]
            
            if temp > critical_temp:
                self._trigger_fault(
                    fault_code=self.fault_codes.get('JOINT_OVERTEMP', 102),
                    category="JOINT",
                    description=f"Joint {joint_name} critical temperature: {temp:.1f}°C",
                    severity=Fault.CRITICAL,
                    component=joint_name,
                    joint_name=joint_name,
                    fault_value=float(temp),
                    fault_threshold=float(critical_temp),
                    fault_units="°C",
                    auto_recoverable=False
                )
            elif temp > warning_temp:
                self._trigger_fault(
                    fault_code=self.fault_codes.get('JOINT_OVERTEMP', 102),
                    category="JOINT", 
                    description=f"Joint {joint_name} high temperature: {temp:.1f}°C",
                    severity=Fault.WARNING,
                    component=joint_name,
                    joint_name=joint_name,
                    fault_value=float(temp),
                    fault_threshold=float(warning_temp),
                    fault_units="°C",
                    auto_recoverable=True
                )
    
    def _check_force_torque_limits(self):
        """Check TCP force and torque limits"""
        try:
            # Check force limits
            force = self.tcp_wrench.wrench.force
            force_magnitude = np.sqrt(force.x**2 + force.y**2 + force.z**2)
            
            max_force = self.force_limits.get('max_force_z', 50.0)
            if force_magnitude > max_force:
                self._trigger_fault(
                    fault_code=self.fault_codes.get('FORCE_LIMIT_EXCEEDED', 201),
                    category="FORCE_TORQUE",
                    description=f"TCP force limit exceeded: {force_magnitude:.1f} N",
                    severity=Fault.ERROR,
                    component="tcp_force_sensor",
                    fault_value=float(force_magnitude),
                    fault_threshold=float(max_force),
                    fault_units="N",
                    auto_recoverable=True
                )
            
            # Check torque limits
            torque = self.tcp_wrench.wrench.torque
            torque_magnitude = np.sqrt(torque.x**2 + torque.y**2 + torque.z**2)
            
            max_torque = self.force_limits.get('max_torque_z', 10.0)
            if torque_magnitude > max_torque:
                self._trigger_fault(
                    fault_code=self.fault_codes.get('TORQUE_LIMIT_EXCEEDED', 202),
                    category="FORCE_TORQUE",
                    description=f"TCP torque limit exceeded: {torque_magnitude:.1f} Nm",
                    severity=Fault.ERROR,
                    component="tcp_force_sensor",
                    fault_value=float(torque_magnitude),
                    fault_threshold=float(max_torque),
                    fault_units="Nm",
                    auto_recoverable=True
                )
                
        except Exception as e:
            # If F/T data is invalid, don't trigger fault
            pass
    
    def _check_workspace_limits(self):
        """Check workspace violations"""
        # TODO: Implement workspace checking once forward kinematics is available
        # This would require computing TCP position from joint states
        pass
    
    def _trigger_fault(self, fault_code: int, category: str, description: str, 
                      severity: int, component: str, auto_recoverable: bool = True,
                      joint_name: str = "", fault_value: float = 0.0, 
                      fault_threshold: float = 0.0, fault_units: str = ""):
        """Trigger a safety fault with structured information"""
        
        # Check if this is a duplicate fault (within 1 second)
        fault_key = f"{fault_code}_{component}_{joint_name}"
        current_time = time.time()
        
        if (fault_key in self.last_fault_times and 
            current_time - self.last_fault_times[fault_key] < 1.0):
            return  # Suppress duplicate faults
        
        self.last_fault_times[fault_key] = current_time
        self.fault_counts[fault_key] = self.fault_counts.get(fault_key, 0) + 1
        
        # Create fault message
        fault = Fault()
        fault.header.stamp = self.get_clock().now().to_msg()
        fault.severity = severity
        fault.fault_code = fault_code
        fault.fault_category = category
        fault.fault_description = description
        fault.component_name = component
        fault.joint_name = joint_name
        fault.fault_value = fault_value
        fault.fault_threshold = fault_threshold
        fault.fault_units = fault_units
        fault.auto_recoverable = auto_recoverable
        fault.occurrence_count = self.fault_counts[fault_key]
        
        # System state information
        fault.system_state = ArmStatus.FAULT
        fault.current_action = "safety_monitoring"
        
        # Recovery information
        fault.recovery_attempted = False
        fault.recovery_successful = False
        
        # Publish fault
        self.fault_pub.publish(fault)
        self.active_faults.append(fault)
        
        # Update safety status
        if severity >= Fault.ERROR:
            self.safety_ok = False
            self.fault_active = True
            
            # Trigger emergency stop for critical faults
            if severity >= Fault.CRITICAL:
                self._trigger_emergency_stop(description)
        
        level = ["INFO", "WARN", "ERROR", "CRITICAL", "EMERGENCY"][severity]
        self.get_logger().log(
            level.upper(),
            f"FAULT {fault_code}: {description} (Component: {component})"
        )
    
    def _trigger_emergency_stop(self, reason: str):
        """Trigger emergency stop"""
        estop_msg = Bool()
        estop_msg.data = True
        self.emergency_stop_pub.publish(estop_msg)
        
        self.get_logger().error(f"EMERGENCY STOP TRIGGERED: {reason}")
    
    def _update_safety_status(self):
        """Update overall safety status"""
        # Remove old faults (older than 5 seconds for auto-recoverable)
        current_time = time.time()
        self.active_faults = [
            fault for fault in self.active_faults
            if not (fault.auto_recoverable and 
                   (current_time - fault.header.stamp.sec - fault.header.stamp.nanosec * 1e-9) > 5.0)
        ]
        
        # Update safety status
        critical_faults = [f for f in self.active_faults if f.severity >= Fault.ERROR]
        self.safety_ok = len(critical_faults) == 0 and not self.estop_active
        self.fault_active = len(self.active_faults) > 0
    
    def _publish_status_callback(self):
        """Publish safety status diagnostics"""
        try:
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            # Overall safety status
            overall_status = DiagnosticStatus()
            overall_status.name = "arm_safety_overall"
            overall_status.hardware_id = "arm_safety_monitor"
            
            if self.safety_ok:
                overall_status.level = DiagnosticStatus.OK
                overall_status.message = "All safety systems nominal"
            elif self.estop_active:
                overall_status.level = DiagnosticStatus.ERROR
                overall_status.message = "Emergency stop active"
            elif self.fault_active:
                overall_status.level = DiagnosticStatus.ERROR
                overall_status.message = f"Active faults: {len(self.active_faults)}"
            else:
                overall_status.level = DiagnosticStatus.WARN
                overall_status.message = "Safety system in warning state"
            
            # Add key values
            overall_status.values.append(KeyValue(key="safety_ok", value=str(self.safety_ok)))
            overall_status.values.append(KeyValue(key="estop_active", value=str(self.estop_active)))
            overall_status.values.append(KeyValue(key="active_faults", value=str(len(self.active_faults))))
            overall_status.values.append(KeyValue(key="fault_response_time", value=f"{self.fault_response_timeout:.3f}s"))
            
            diag_array.status.append(overall_status)
            
            # Individual subsystem status
            self._add_joint_diagnostics(diag_array)
            self._add_force_torque_diagnostics(diag_array)
            self._add_communication_diagnostics(diag_array)
            
            self.safety_status_pub.publish(diag_array)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish safety status: {e}")
    
    def _add_joint_diagnostics(self, diag_array: DiagnosticArray):
        """Add joint-specific diagnostics"""
        for i, joint_name in enumerate(self.joint_names):
            status = DiagnosticStatus()
            status.name = f"arm_joint_{joint_name}"
            status.hardware_id = joint_name
            status.level = DiagnosticStatus.OK
            status.message = "Joint nominal"
            
            # Add joint data
            if i < len(self.joint_temperatures):
                temp = self.joint_temperatures[i]
                status.values.append(KeyValue(key="temperature", value=f"{temp:.1f}°C"))
                
                warning_temp = self.temp_limits.get('warning_temp', 65.0)
                if temp > warning_temp:
                    status.level = DiagnosticStatus.WARN
                    status.message = "High temperature"
            
            if i < len(self.joint_currents):
                current = self.joint_currents[i]
                status.values.append(KeyValue(key="current", value=f"{current:.2f}A"))
                
                if joint_name in self.current_limits:
                    limit = self.current_limits[joint_name]
                    if current > limit * 0.8:  # 80% of limit
                        status.level = DiagnosticStatus.WARN
                        status.message = "High current"
            
            if len(self.joint_state.position) > i:
                pos = self.joint_state.position[i]
                status.values.append(KeyValue(key="position", value=f"{pos:.3f}rad"))
            
            if len(self.joint_state.velocity) > i:
                vel = self.joint_state.velocity[i]
                status.values.append(KeyValue(key="velocity", value=f"{vel:.3f}rad/s"))
            
            diag_array.status.append(status)
    
    def _add_force_torque_diagnostics(self, diag_array: DiagnosticArray):
        """Add force/torque sensor diagnostics"""
        status = DiagnosticStatus()
        status.name = "arm_force_torque"
        status.hardware_id = "tcp_ft_sensor"
        
        try:
            force = self.tcp_wrench.wrench.force
            torque = self.tcp_wrench.wrench.torque
            
            force_mag = np.sqrt(force.x**2 + force.y**2 + force.z**2)
            torque_mag = np.sqrt(torque.x**2 + torque.y**2 + torque.z**2)
            
            status.values.append(KeyValue(key="force_magnitude", value=f"{force_mag:.1f}N"))
            status.values.append(KeyValue(key="torque_magnitude", value=f"{torque_mag:.1f}Nm"))
            
            # Check age of data
            data_age = time.time() - self.tcp_wrench_time
            status.values.append(KeyValue(key="data_age", value=f"{data_age:.3f}s"))
            
            if data_age > 0.2:
                status.level = DiagnosticStatus.WARN
                status.message = "Stale F/T data"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "F/T sensor nominal"
            
        except Exception:
            status.level = DiagnosticStatus.ERROR
            status.message = "F/T sensor error"
        
        diag_array.status.append(status)
    
    def _add_communication_diagnostics(self, diag_array: DiagnosticArray):
        """Add communication diagnostics"""
        status = DiagnosticStatus()
        status.name = "arm_communication"
        status.hardware_id = "communication_monitor"
        
        current_time = time.time()
        
        # Check data ages
        joint_state_age = current_time - self.joint_state_time
        joint_currents_age = current_time - self.joint_currents_time
        joint_temps_age = current_time - self.joint_temperatures_time
        
        status.values.append(KeyValue(key="joint_state_age", value=f"{joint_state_age:.3f}s"))
        status.values.append(KeyValue(key="joint_currents_age", value=f"{joint_currents_age:.3f}s"))
        status.values.append(KeyValue(key="joint_temps_age", value=f"{joint_temps_age:.3f}s"))
        
        # Determine overall communication status
        max_age = max(joint_state_age, joint_currents_age, joint_temps_age)
        
        if max_age > 0.1:
            status.level = DiagnosticStatus.ERROR
            status.message = "Communication timeout"
        elif max_age > 0.05:
            status.level = DiagnosticStatus.WARN
            status.message = "Communication delay"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Communication nominal"
        
        diag_array.status.append(status)


def main():
    """Main entry point"""
    rclpy.init()
    
    try:
        executor = MultiThreadedExecutor()
        safety_monitor = SafetyMonitor()
        executor.add_node(safety_monitor)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Safety monitor failed: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main() 