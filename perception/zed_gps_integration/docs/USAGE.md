# ZED GPS Integration - Complete Usage Guide

## Table of Contents

1. [System Overview](#system-overview)
2. [Installation](#installation)
3. [Basic Usage](#basic-usage)
4. [Advanced Configuration](#advanced-configuration)
5. [Monitoring & Debugging](#monitoring--debugging)
6. [Integration Examples](#integration-examples)
7. [Troubleshooting](#troubleshooting)

---

## System Overview

### What It Does

- **Fuses** ZED camera visual-inertial odometry with GNSS GPS data
- **Calibrates** automatically to align VIO and GNSS reference frames
- **Publishes** fused poses in both local (meters) and global (lat/lon) coordinates
- **Visualizes** live position on real-world map with dual trajectories
- **Operates** seamlessly during GPS outages (VIO continues)

### Architecture

```
GPS Receiver → /gps/fix → ZED Fusion Node → Fused Outputs
                               ↓
                          Map Server → Web Files → Browser
```

---

## Installation

### 1. Build Package

```bash
cd ~/workspaces/rover
colcon build --packages-select zed_gps_integration
source install/setup.bash
```

### 2. Verify Installation

```bash
# Check package is installed
ros2 pkg list | grep zed_gps_integration

# Check executables
ros2 pkg executables zed_gps_integration
# Should show:
#   zed_gps_integration zed_gnss_fusion
#   zed_gps_integration map_server
```

---

## Basic Usage

### Method 1: Simple Launch (Recommended)

**Single command to launch everything:**

```bash
# If GPS already running:
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py

# Or launch GPS too:
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py launch_gnss:=true
```

### Method 2: Manual Launch (For Debugging)

**Terminal 1 - GPS:**
```bash
ros2 launch gnss_launch gnss.launch.py
```

**Terminal 2 - Fusion:**
```bash
ros2 run zed_gps_integration zed_gnss_fusion --ros-args \
    --params-file ~/workspaces/rover/install/zed_gps_integration/share/zed_gps_integration/config/fusion_params.yaml
```

**Terminal 3 - Map Server:**
```bash
ros2 run zed_gps_integration map_server --ros-args \
    --params-file ~/workspaces/rover/install/zed_gps_integration/share/zed_gps_integration/config/map_server_params.yaml
```

**Terminal 4 - Web Server:**
```bash
cd ~/zed_map_data
python3 -m http.server 8000
# Open: http://localhost:8000/
```

### Calibration Procedure

**Critical: You MUST move the camera for calibration to complete.**

1. **Wait for GPS fix** (30-60 seconds outdoors)
   ```bash
   ros2 topic echo /gps/fix --field status.status
   # Should show: 0 or greater
   ```

2. **Move in calibration pattern:**
   - ➡️ Forward 10-15 meters (straight line)
   - ↪️ Turn 30-45 degrees (smooth)
   - ➡️ Forward 10 more meters
   - ↩️ Optional: Return to start with another turn

3. **Watch for completion:**
   ```
   [INFO] [zed_gnss_fusion]: ✅ GNSS/VIO Calibration complete!
   [INFO] [zed_gnss_fusion]:    Translation uncertainty: 0.123 m
   [INFO] [zed_gnss_fusion]:    Yaw uncertainty: 0.067 rad
   ```

4. **Verify geo pose publishing:**
   ```bash
   ros2 topic hz /zed_gnss_fusion/geo_pose
   # Should show: ~15-30 Hz
   ```

---

## Advanced Configuration

### Custom Camera Settings

**Edit: `config/fusion_params.yaml`**

```yaml
camera_sn: 12345678  # Specific camera serial number
camera_resolution: 'HD720'  # Lower res = higher FPS
```

**Or via launch arguments:**

```bash
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py \
    camera_sn:=12345678 \
    gnss_topic:=/custom/gps/topic
```

### Calibration Tuning

**For faster calibration (less accurate):**

```yaml
target_yaw_uncertainty: 0.2  # Was 0.1
target_translation_uncertainty: 0.3  # Was 0.15
```

**For stricter calibration (more accurate):**

```yaml
target_yaw_uncertainty: 0.05  # Was 0.1 
target_translation_uncertainty: 0.08  # Was 0.15
```

**For challenging GPS environments:**

```yaml
enable_reinitialization: false  # Prevent recalibration
gnss_vio_reinit_threshold: 10.0  # More tolerant (was 5.0)
```

### RTK Configuration

**For centimeter-level accuracy:**

```bash
# Terminal 1: Start gpsd with NTRIP
pkill gpsd
gpsd -nG ntrip://user:pass@caster:port/mount -s 115200 /dev/ttyACM0

# Terminal 2: Launch fusion
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py

# Expected: Fix type shows "RTK_FIX" on map
```

---

## Monitoring & Debugging

### System Status Check

**Use the built-in checker:**

```bash
cd ~/workspaces/rover/src/perception/zed_gps_integration
./scripts/check_system.sh
```

Output shows:
- ✅ ROS environment
- ✅ GPS status
- ✅ Fusion node status
- ✅ Calibration state
- ✅ Map data freshness
- ✅ Web server status

### Topic Monitoring

**Check all fusion topics:**

```bash
ros2 topic list | grep zed_gnss
```

**Monitor calibration progress:**

```bash
ros2 topic echo /zed_gnss_fusion/calibration_status
```

**View fused odometry:**

```bash
ros2 topic echo /zed_gnss_fusion/fused_odom
```

**View geographic position:**

```bash
ros2 topic echo /zed_gnss_fusion/geo_pose
```

**Check diagnostics:**

```bash
ros2 topic echo /zed_gnss_fusion/diagnostics
```

### Performance Monitoring

**Check topic rates:**

```bash
# Fusion should run at camera FPS
ros2 topic hz /zed_gnss_fusion/fused_odom
# Expected: ~15-30 Hz

# Geo pose only after calibration
ros2 topic hz /zed_gnss_fusion/geo_pose
# Expected: ~15-30 Hz (after calibration)
```

**Check GPS quality:**

```bash
ros2 topic echo /gps/fix --field position_covariance
# Lower values = better accuracy
# RTK: < 0.01
# DGPS: < 1.0
# Standard GPS: < 10.0
```

### Visualization in RViz

```bash
ros2 run rviz2 rviz2
```

**Add displays:**
1. **Path** → Topic: `/zed_gnss_fusion/fused_path`
2. **TF** → Show camera transforms
3. **Odometry** → Topic: `/zed_gnss_fusion/fused_odom`

---

## Integration Examples

### With Nav2 Navigation

**Use fused odometry for navigation:**

```yaml
# nav2_params.yaml
controller_server:
  ros__parameters:
    odom_topic: "/zed_gnss_fusion/fused_odom"
```

### With Robot Localization

**As alternative to EKF (fusion already done):**

```yaml
# Use fusion output directly instead of EKF
# Set map→odom→base_link transforms from fusion
```

### Custom Application Integration

**Python example:**

```python
import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped

class MyApp(Node):
    def __init__(self):
        super().__init__('my_app')
        self.create_subscription(
            GeoPoseStamped,
            '/zed_gnss_fusion/geo_pose',
            self.pose_callback,
            10
        )
    
    def pose_callback(self, msg):
        lat = msg.pose.position.latitude
        lon = msg.pose.position.longitude
        # Use GPS coordinates for your application
        print(f"Current position: {lat:.6f}, {lon:.6f}")
```

### Recording Data

**Record fusion output:**

```bash
ros2 bag record \
    /zed_gnss_fusion/fused_odom \
    /zed_gnss_fusion/geo_pose \
    /zed_gnss_fusion/fused_path \
    /gps/fix
```

**Replay:**

```bash
ros2 bag play my_recording.db3
```

---

## Troubleshooting

### Calibration Issues

**Problem: "Calibration not converging"**

Symptoms:
- Stuck on "Calibrating..." for > 5 minutes
- No geo_pose topic publishing

Solutions:
1. **Move MORE**: Drive 20+ meters, turn 45+ degrees
2. **Check GPS quality**:
   ```bash
   ros2 topic echo /gnss/satellite_count
   # Need: > 6 satellites
   
   ros2 topic echo /gps/fix --field position_covariance
   # Need: values < 10.0
   ```

3. **Relax thresholds** (edit config):
   ```yaml
   target_yaw_uncertainty: 0.2
   target_translation_uncertainty: 0.3
   ```

4. **Check IMU**: Ensure camera IMU is working
   ```bash
   # IMU fusion must be enabled for GNSS fusion
   ```

**Problem: "Calibration lost after working"**

Cause: Large divergence between VIO and GPS

Solutions:
- Allow reinitialization:
  ```yaml
  enable_reinitialization: true
  gnss_vio_reinit_threshold: 5.0
  ```
- Improve GPS (use RTK)
- Check for VIO drift (poor lighting, fast motion)

### GPS Issues

**Problem: "No GNSS data"**

Check:
```bash
# GPS topic exists?
ros2 topic list | grep gps

# GPS publishing?
ros2 topic hz /gps/fix

# GPS has fix?
ros2 topic echo /gps/fix --field status.status
# Should be >= 0
```

Solutions:
- Go outdoors (no GPS indoors!)
- Wait for satellite lock (30-60s)
- Check antenna connection
- Verify gpsd running: `ps aux | grep gpsd`

**Problem: "GPS jumping/unstable"**

Symptoms:
- Position jumps on map
- Covariance circles very large

Solutions:
1. **Enable RTK** for stable cm-accuracy
2. **Increase covariance** if sensor reports too-low values
3. **Check multipath**: Move away from buildings/trees
4. **Wait longer**: Initial GPS can be unstable

### Map Display Issues

**Problem: "Blank web map"**

Check:
1. **Files exist and updating:**
   ```bash
   ls -lh ~/zed_map_data/
   watch cat ~/zed_map_data/data.txt
   ```

2. **Map server running:**
   ```bash
   ros2 node list | grep map_server
   ```

3. **Web server running:**
   ```bash
   lsof -i:8000
   ```

4. **Browser console** (F12 → Console) for errors

**Problem: "Old position on map"**

- Check file timestamps: `ls -lh ~/zed_map_data/`
- Verify fusion node running
- Check calibration completed
- Try hard refresh: Ctrl+F5

### Performance Issues

**Problem: "Low frame rate"**

Solutions:
1. **Lower camera resolution:**
   ```yaml
   camera_resolution: 'HD720'  # Was HD1080
   ```

2. **Check system load:**
   ```bash
   top
   # Look for high CPU usage
   ```

3. **Verify GPU available:**
   ```bash
   nvidia-smi
   ```

---

## Quick Reference Commands

```bash
# Build
colcon build --packages-select zed_gps_integration

# Launch (simple)
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py

# Check status
./scripts/check_system.sh

# Start map
./scripts/start_map_server.sh

# Monitor
ros2 topic echo /zed_gnss_fusion/calibration_status
ros2 topic hz /zed_gnss_fusion/geo_pose
ros2 topic echo /zed_gnss_fusion/diagnostics

# View
http://localhost:8000/
```

---

## Support

- **Documentation**: See `README.md` and `QUICK_START.md`
- **Integration Guide**: See `/home/rover/doc_integration.md`
- **Global Localization Guide**: See `/home/rover/global_localization_guide.md`
- **ZED SDK Docs**: See `docs.md`

---

© 2025 - ZED GPS Integration Complete Usage Guide

