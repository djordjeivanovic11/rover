"""
ARM CONTROL PACKAGE

Runtime control layer for URC rover arm manipulation system.

This package provides:
- Hardware interface for 250 Hz control loop
- Safety monitor with <150ms fault response  
- Trajectory executor with real-time monitoring
- Gripper controller with force feedback
- Tool manager with hot URDF/SRDF reloading
- Action servers for autonomous operation

All components are designed for mission-critical operation with
comprehensive safety monitoring and structured error reporting.
"""

__version__ = "1.0.0"
__author__ = "URC Rover Team"
__license__ = "MIT"
