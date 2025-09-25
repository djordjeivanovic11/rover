# GNSS Launch Package

**Plug-and-play GNSS driver setup with health monitoring and ZED integration**

## 🚀 Quick Start

### **1. Connect GNSS Hardware**
```bash
# Most common: USB connection shows as /dev/ttyACM0
# Check connected devices
ls /dev/ttyACM* /dev/ttyUSB*

# Test hardware detection
ros2 service call /gnss/detect_hardware std_srvs/srv/Trigger
```

### **2. Launch GNSS Driver**
```bash
# Auto-detect and launch (recommended)
ros2 launch gnss_launch gnss_complete.launch.py

# Specify device manually
ros2 launch gnss_launch gnss_complete.launch.py gnss_device:=/dev/ttyACM0

# Use different driver (ublox, nmea, septentrio)
ros2 launch gnss_launch gnss_complete.launch.py gnss_driver_type:=nmea
```

### **3. Verify GNSS Operation**
```bash
# Check fix data
ros2 topic echo /gps/fix

# Monitor health status
ros2 topic echo /gnss/health_status
ros2 topic echo /gnss/ready_for_fusion

# Check diagnostics
ros2 topic echo /gnss/diagnostics
```

## 📡 Supported Hardware

| Driver | Hardware | Protocol | Use Case |
|--------|----------|----------|----------|
| **ublox** | u-blox M8/M9/F9 | UBX + NMEA | **Recommended** - High accuracy, RTK support |
| **nmea** | Generic GNSS | NMEA-0183 | Basic GPS receivers |
| **septentrio** | Septentrio | SBF + NMEA | High-end RTK systems |

## 🔧 Configuration

### **Hardware Settings** (`params/gnss_config.yaml`):
```yaml
gnss_driver:
  device: /dev/ttyACM0        # Serial device
  baudrate: 115200           # Communication rate
  measurement_rate: 10       # GNSS update rate (Hz)
  min_satellites: 4          # Minimum satellites for valid fix
  max_hdop: 5.0             # Maximum dilution of precision
```

### **ZED Integration** (Automatic):
- Publishes to `/gps/fix` (ZED fusion compatible)
- Monitors signal quality for fusion readiness
- Provides health status for system monitoring

## 🎯 Usage Examples

### **Basic GNSS Operation:**
```bash
# Launch with u-blox receiver
ros2 launch gnss_launch gnss_complete.launch.py gnss_driver_type:=ublox

# Launch with generic NMEA receiver  
ros2 launch gnss_launch gnss_complete.launch.py gnss_driver_type:=nmea
```

### **With ZED Global Localization:**
```bash
# Complete perception with GNSS fusion
ros2 launch slam_launch perception_complete.launch.py \
    enable_global_localization:=true

# Monitor fusion status
ros2 topic echo /localization/global_ready
ros2 topic echo /localization/mode
```

### **Hardware Troubleshooting:**
```bash
# Detect GNSS hardware
ros2 service call /gnss/detect_hardware std_srvs/srv/Trigger

# Test connection
ros2 service call /gnss/test_connection std_srvs/srv/Trigger

# Check signal quality
ros2 topic echo /gnss/signal_quality
ros2 topic echo /gnss/satellite_count
```

## 📊 Health Monitoring

The system provides comprehensive GNSS health monitoring:

### **Health States:**
- **🟢 excellent** - RTK fix, high accuracy
- **🟡 good** - 3D fix, good accuracy  
- **🟠 fair** - 3D fix, moderate accuracy
- **🔴 poor** - 2D fix or high uncertainty
- **❌ no_fix** - No valid position
- **📵 no_signal** - No data received

### **Topics:**
```bash
/gnss/health_status        # Current health state
/gnss/signal_quality       # Quality score (0-1)
/gnss/satellite_count      # Number of satellites
/gnss/ready_for_fusion     # Ready for ZED fusion
/gnss/diagnostics          # Detailed diagnostics
```

## ⚡ Performance

- **Update Rate:** 5-10 Hz (configurable)
- **Latency:** <100ms from hardware to ROS topic
- **Accuracy:** Depends on receiver (1-5m typical, <10cm with RTK)
- **Startup Time:** 30-60 seconds for first fix (cold start)

**Ready to plug in your GNSS receiver and get global navigation!** 🌍
