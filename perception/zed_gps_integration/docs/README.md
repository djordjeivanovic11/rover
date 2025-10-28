# ZED GPS Integration - Global Localization

**Complete ZED Camera + GNSS Fusion with Live Map Visualization**

## Features

- ✅ **Real-time Fusion**: Combines ZED VIO with GNSS using ZED SDK Fusion module
- 🗺️ **Live Web Map**: Beautiful Leaflet-based visualization with dual trajectories
- 📡 **RTK Support**: Full support for RTK-corrected GNSS (centimeter accuracy)
- 🔄 **Automatic Calibration**: Self-calibrating VIO/GNSS reference frame alignment
- 📊 **Diagnostics**: Real-time status monitoring and health checks
- 🎯 **Production Ready**: Clean, documented code with proper error handling

## Quick Start

### 1. Build the Package

```bash
cd ~/workspaces/rover
colcon build --packages-select zed_gps_integration
source install/setup.bash
```

### 2. Launch the System

```bash
# If GPS is already running:
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py

# Or launch GPS together with fusion:
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py launch_gnss:=true
```

### 3. Calibrate

**Move the camera to calibrate VIO/GNSS alignment:**
1. Drive forward 10-15 meters in a straight line
2. Turn 30-45 degrees (smooth turn)
3. Drive forward another 10 meters
4. Watch console for "✅ Calibration complete!" message

### 4. View Live Map

```bash
cd ~/zed_map_data
python3 -m http.server 8000
```

Open browser to: **http://localhost:8000/**

You'll see:
- 🔵 **Blue marker/path**: Fused VIO+GNSS (high accuracy)
- 🟡 **Yellow marker/path**: Raw GNSS (for comparison)
- Real-time position updates
- GPS fix status and covariance visualization

## Architecture

```
┌─────────────────┐
│  GNSS Driver    │  /gps/fix (NavSatFix)
└────────┬────────┘
         │
         ▼
┌─────────────────────────────┐
│  ZED GNSS Fusion Node       │
│  - Opens ZED camera         │
│  - Initializes SDK Fusion   │
│  - Ingests GNSS data        │
│  - Publishes fused outputs  │
└────────┬────────────────────┘
         │
         ├─► /zed_gnss_fusion/fused_odom (Odometry)
         ├─► /zed_gnss_fusion/geo_pose (GeoPoseStamped)
         ├─► /zed_gnss_fusion/fused_path (Path)
         └─► /zed_gnss_fusion/calibration_status (String)
         
         ▼
┌─────────────────────────────┐
│  Map Server Bridge          │
│  - Subscribes to topics     │
│  - Writes files for web     │
└────────┬────────────────────┘
         │
         ├─► ~/zed_map_data/data.txt (fused)
         └─► ~/zed_map_data/raw_data.txt (raw GPS)
         
         ▼
┌─────────────────────────────┐
│  Web Browser (Leaflet.js)   │
│  - Polls data files         │
│  - Renders live map         │
│  - Shows dual trajectories  │
└─────────────────────────────┘
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/zed_gnss_fusion/fused_odom` | Odometry | Fused VIO+GNSS pose in map frame |
| `/zed_gnss_fusion/geo_pose` | GeoPoseStamped | Position in GPS coordinates (lat/lon) |
| `/zed_gnss_fusion/fused_path` | Path | Trajectory for visualization |
| `/zed_gnss_fusion/calibration_status` | String | Calibration state |
| `/zed_gnss_fusion/diagnostics` | DiagnosticArray | System health status |

## Configuration

Edit `config/fusion_params.yaml`:

```yaml
# Camera
camera_sn: 0  # Auto-detect, or specific SN
camera_resolution: 'HD1080'

# GNSS
gnss_topic: '/gps/fix'

# Calibration (adjust for your environment)
target_yaw_uncertainty: 0.1  # radians
target_translation_uncertainty: 0.15  # meters
enable_reinitialization: true
gnss_vio_reinit_threshold: 5.0  # meters

# Publishing
publish_tf: true
publish_path: true
```

## Troubleshooting

### Calibration Not Converging

**Problem**: Stuck in "Calibrating..." state

**Solution**:
- Move MORE: Drive 20+ meters, turn more
- Check GPS quality: `ros2 topic echo /gps/fix`
- Need 6+ satellites, HDOP < 2.0
- Relax thresholds in config (increase uncertainty targets)

### No Map Display

**Problem**: Browser shows blank map

**Solution**:
1. Check files are being written:
   ```bash
   ls -lh ~/zed_map_data/
   # Should see data.txt and raw_data.txt with recent timestamps
   ```

2. Verify map server is running:
   ```bash
   ros2 node list | grep map_server
   ```

3. Check browser console for errors (F12 → Console)

### GPS Not Detected

**Problem**: "No GNSS data" in diagnostics

**Solution**:
1. Verify GPS is publishing:
   ```bash
   ros2 topic hz /gps/fix
   # Should show update rate
   ```

2. Check topic name matches config
3. Ensure GPS has valid fix (not indoors)

## Advanced Usage

### Using with RTK Base Station

For centimeter-level accuracy:

```bash
# Start GPS with NTRIP corrections
pkill gpsd
gpsd -nG ntrip://user:pass@caster:port/mount -s 115200 /dev/ttyACM0

# Launch fusion
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py
```

### Custom Camera Configuration

```bash
ros2 launch zed_gps_integration zed_gnss_fusion.launch.py \
    camera_sn:=12345678 \
    gnss_topic:=/custom/gps/topic
```

### Integration with Navigation Stack

The fused odometry can be used with Nav2:

```bash
# Use /zed_gnss_fusion/fused_odom as odometry source
# Set in Nav2 config:
# odom_topic: "/zed_gnss_fusion/fused_odom"
```

## Files

```
zed_gps_integration/
├── zed_gps_integration/
│   ├── __init__.py
│   ├── zed_gnss_fusion.py      # Main fusion node
│   └── map_server.py            # Web map bridge
├── launch/
│   └── zed_gnss_fusion.launch.py
├── config/
│   ├── fusion_params.yaml
│   └── map_server_params.yaml
├── web_map/
│   └── index.html               # Leaflet map page
├── docs.md                      # ZED SDK documentation
├── setup.py
├── package.xml
└── README.md
```

## Dependencies

- ROS 2 Humble
- ZED SDK 4.1+ with Python API
- GNSS driver (gnss_launch package)
- Python packages: rclpy, geographic_msgs

## References

- [ZED SDK Global Localization](https://www.stereolabs.com/docs/api/group__GeoPose__group.html)
- [Integration Documentation](../../../doc_integration.md)
- [Global Localization Guide](../../../global_localization_guide.md)

---

© 2025 - ZED GPS Integration Package

