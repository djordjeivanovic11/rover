#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import subprocess
import serial
import time
import os
from typing import List, Dict, Optional


class GNSSSetup(Node):
    """GNSS hardware detection and setup utility"""
    
    def __init__(self):
        super().__init__('gnss_setup')
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/gnss/setup_status', 10)
        
        # Services
        self.create_service(Trigger, '/gnss/detect_hardware', self.detect_hardware)
        self.create_service(Trigger, '/gnss/test_connection', self.test_connection)
        
        self.get_logger().info("🔧 GNSS Setup Utility ready")
    
    def detect_hardware(self, request, response):
        """Detect connected GNSS hardware"""
        try:
            devices = self.scan_serial_devices()
            gnss_devices = self.identify_gnss_devices(devices)
            
            if gnss_devices:
                device_list = ', '.join([f"{dev['port']} ({dev['type']})" for dev in gnss_devices])
                response.success = True
                response.message = f"Found GNSS devices: {device_list}"
                
                # Publish status
                status_msg = String()
                status_msg.data = f"detected:{len(gnss_devices)},{device_list}"
                self.status_pub.publish(status_msg)
                
                self.get_logger().info(f"📡 {response.message}")
            else:
                response.success = False
                response.message = "No GNSS devices detected"
                self.get_logger().warn("⚠️ No GNSS devices found")
            
        except Exception as e:
            response.success = False
            response.message = f"Hardware detection failed: {str(e)}"
            self.get_logger().error(f"GNSS detection error: {e}")
        
        return response
    
    def test_connection(self, request, response):
        """Test GNSS connection and data flow"""
        try:
            # Test default device
            test_device = '/dev/ttyACM0'
            
            if os.path.exists(test_device):
                # Test serial connection
                connection_ok = self.test_serial_connection(test_device)
                
                if connection_ok:
                    response.success = True
                    response.message = f"GNSS connection test passed: {test_device}"
                    self.get_logger().info(f"✅ {response.message}")
                else:
                    response.success = False
                    response.message = f"GNSS connection test failed: {test_device}"
                    self.get_logger().error(f"❌ {response.message}")
            else:
                response.success = False
                response.message = f"GNSS device not found: {test_device}"
                self.get_logger().error(f"❌ {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Connection test failed: {str(e)}"
            self.get_logger().error(f"Connection test error: {e}")
        
        return response
    
    def scan_serial_devices(self) -> List[str]:
        """Scan for available serial devices"""
        devices = []
        
        # Common serial device paths
        device_paths = [
            '/dev/ttyACM*',
            '/dev/ttyUSB*', 
            '/dev/serial/by-id/*'
        ]
        
        for path_pattern in device_paths:
            try:
                # Use shell expansion to find devices
                result = subprocess.run(['ls', path_pattern], 
                                      capture_output=True, text=True, shell=True)
                if result.returncode == 0:
                    devices.extend(result.stdout.strip().split('\n'))
            except:
                continue
        
        return [dev for dev in devices if dev and os.path.exists(dev)]
    
    def identify_gnss_devices(self, devices: List[str]) -> List[Dict]:
        """Identify which devices are GNSS receivers"""
        gnss_devices = []
        
        for device in devices:
            try:
                device_info = self.probe_device(device)
                if device_info:
                    gnss_devices.append(device_info)
            except:
                continue
        
        return gnss_devices
    
    def probe_device(self, device: str) -> Optional[Dict]:
        """Probe a device to determine if it's a GNSS receiver"""
        try:
            # Test common GNSS baudrates
            baudrates = [9600, 38400, 115200, 230400]
            
            for baudrate in baudrates:
                try:
                    with serial.Serial(device, baudrate, timeout=2) as ser:
                        # Read a few lines to identify device
                        for _ in range(10):
                            line = ser.readline().decode('ascii', errors='ignore').strip()
                            
                            # Check for NMEA sentences (common GNSS format)
                            if line.startswith('$GP') or line.startswith('$GN') or line.startswith('$GL'):
                                return {
                                    'port': device,
                                    'baudrate': baudrate,
                                    'type': 'NMEA',
                                    'protocol': 'NMEA-0183'
                                }
                            
                            # Check for u-blox UBX protocol
                            if b'\xb5\x62' in line.encode():
                                return {
                                    'port': device,
                                    'baudrate': baudrate,
                                    'type': 'u-blox',
                                    'protocol': 'UBX'
                                }
                        
                except serial.SerialException:
                    continue
            
            return None
            
        except Exception:
            return None
    
    def test_serial_connection(self, device: str, baudrate: int = 115200) -> bool:
        """Test serial connection to GNSS device"""
        try:
            with serial.Serial(device, baudrate, timeout=5) as ser:
                # Try to read some data
                start_time = time.time()
                while time.time() - start_time < 3.0:
                    if ser.in_waiting > 0:
                        data = ser.readline()
                        if data:
                            return True
                
                return False
                
        except Exception:
            return False


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        setup_node = GNSSSetup()
        rclpy.spin(setup_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
