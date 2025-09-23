
# ZED 2i Launch Package

> **Launch wrapper and tuned parameters for the Stereolabs ZED 2i camera in the rover perception system**

## 🎯 Purpose

This package provides a carefully configured interface to the Stereolabs ZED 2i stereo camera, optimized specifically for our rover's autonomous navigation and human interaction missions. Rather than using the default ZED settings, we've created this wrapper to ensure consistent, mission-appropriate sensor behavior for the rover.

## 🏗️ Architecture Overview

The `zed2i_launch` package acts as a specialized configuration layer between our rover's perception stack and the official Stereolabs `zed_wrapper`. This design choice allows us to:

- **Maintain consistent sensor behavior** across different rover deployments
- **Optimize performance** for our specific hardware constraints and mission requirements  
- **Simplify integration** with our custom localization and mapping systems
- **Enable easy parameter tuning** without modifying upstream packages

```
Rover Navigation Stack
        ↓
   zed2i_launch (this package)
        ↓
   zed_wrapper (Stereolabs official)
        ↓
   ZED 2i Hardware
```

## 📊 Published Topics & Data Streams

The ZED 2i provides a comprehensive sensor suite that feeds multiple rover subsystems:

### Vision & Depth
- `/zed2i/left/image_rect_color` - Left stereo image (rectified)
- `/zed2i/right/image_rect_color` - Right stereo image (rectified)  
- `/zed2i/depth/depth_registered` - Depth map aligned to left camera
- `/zed2i/point_cloud/cloud_registered` - 3D point cloud (XYZRGBA format)

### Motion & Pose Estimation
- `/zed2i/odom` - 6-DoF visual-inertial odometry
- `/zed2i/pose` - Camera pose in world frame
- `/zed2i/imu/data` - Filtered IMU data (200 Hz)
- `/zed2i/imu/data_raw` - Raw IMU measurements

### Object Detection
- `/zed2i/objects/*` - Human detection results (when enabled)

**Key Integration Point**: The `loc_fusion` system specifically consumes `/zed2i/odom`, `/zed2i/imu/data`, and `/zed2i/point_cloud/cloud_registered` for robust localization and mapping.

## ⚙️ Configuration Choices & Rationale

Our parameter choices in `params/zed2i.yaml` reflect careful optimization for rover operations:

### Resolution & Performance
```yaml
resolution: 2  # 720p (not 2K or 1080p)
grab_frame_rate: 30
```
**Why 720p?** Balances image quality with computational load. Higher resolutions would strain our onboard processing while 720p provides sufficient detail for navigation and human detection.

### Depth Processing
```yaml
depth_mode: "ULTRA"
min_depth: 0.2
```
**Why ULTRA mode?** Maximum depth accuracy is critical for obstacle avoidance and safe navigation. The computational cost is justified by safety requirements.

### IMU Configuration
```yaml
imu_pub_rate: 200.0
```
**Why 200 Hz?** High-frequency IMU data enables smooth motion compensation in our localization filter, especially important during rapid rover movements.

### Point Cloud Settings
```yaml
pub_frequency: 10.0
format: "XYZRGBA"
```
**Why 10 Hz?** Point clouds are bandwidth-intensive. 10 Hz provides sufficient update rate for mapping while preserving network resources for other critical data streams.

### Object Detection
```yaml
detection_model: "HUMAN_BODY_FAST"
object_class_filter: ["PERSON"]
```
**Why human-focused?** Our rover missions involve human interaction and monitoring. This focused approach reduces false positives and computational overhead.

## 🔗 Localization Integration

### TF Frame Design Choice
```
map → odom (zed) → base_link → zed2i_camera_center
```

**Key Decision**: We enable ZED position tracking (`pos_tracking.enable: true`) but disable TF publishing (`publish_tf: false`). 

**Rationale**: Our rover uses a multi-sensor fusion approach where the `loc_fusion` system combines ZED odometry with other sensors (GPS, wheel encoders, etc.) to produce the final robot pose. This prevents conflicting TF publishers and ensures consistent coordinate frames across the system.

## 🚀 Usage

### Basic Launch
```bash
ros2 launch zed2i_launch zed2i_driver.launch.py
```

### Parameter Customization
Edit `params/zed2i.yaml` to adjust camera behavior for different mission profiles or hardware configurations.

### Integration with Rover Stack
This package is typically launched as part of the broader rover perception pipeline:
```bash
ros2 launch rover_bringup perception.launch.py  # Includes zed2i_launch
```

## Development Notes

- **Modular Design**: Parameters are externalized to YAML for easy mission-specific tuning
- **Performance Monitoring**: Use `ros2 topic hz` to verify publication rates match configuration
- **Debugging**: Set `output='screen'` in launch file to see ZED wrapper diagnostics

## References

- [ZED ROS 2 Wrapper Documentation](https://github.com/stereolabs/zed-ros2-wrapper)
- [Stereolabs ROS 2 Integration Guide](https://www.stereolabs.com/docs/ros2)
- [ZED SDK API Reference](https://www.stereolabs.com/docs/api)

---
*Maintained by: Djordje Ivanovic (dorde_ivanovic@college.harvard.edu)*




## Integration with Localization and Mapping Systems

The ZED 2i serves as a primary sensor for the rover's localization and mapping pipeline, particularly interfacing with the loc_fusion system mentioned in the launch file comments (lines 31-34). The camera provides three key data streams for this integration: visual-inertial odometry through /zed2i/odom for continuous pose estimation, high-frequency IMU data via /zed2i/imu/data for motion compensation and filtering, and dense 3D point clouds through /zed2i/point_cloud/cloud_registered for environmental mapping and obstacle avoidance. The system maintains a proper TF frame tree rooted at map → odom (zed) → base_link → zed2i_camera_center (line 37), ensuring proper coordinate transformations throughout the rover's navigation stack.