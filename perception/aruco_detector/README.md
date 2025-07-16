```markdown
# ArUco Detector – ROS 2 Package

Detections of **4 × 4, 50‑ID ArUco tags** from the **left RGB stream of a ZED 2i** and publication of **3‑D tag poses** as `vision_msgs/Detection3DArray`.  
The package is self‑contained and can also be used with any other RGB camera after you supply its calibration.

---

## Folder layout

```

perception/aruco\_detector
├── aruco\_detector/            # Python module (ROS 2 node lives here)
│   ├── **init**.py
│   └── node.py
├── launch/
│   └── aruco\_detector.launch.py
├── config/
│   └── detector\_params.yaml   # marker\_size + topic names
├── scripts/                   # developer tools (not launched on rover)
│   ├── cam\_cal.py             # capture chessboard frames
│   ├── process.py             # compute camera intrinsics
│   └── generate\_markers.py    # print‑ready PNGs of 10 tags
├── resource/aruco\_detector    # ament resource marker (empty)
├── setup.py
└── package.xml

````

---

## Quick‑start (ZED 2i)

1. **Build**

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select aruco_detector
   source install/setup.bash
````

2. **Run the ZED driver** (already in your repo):

   ```bash
   ros2 launch zed2i_launch zed2i_driver.launch.py
   ```

3. **Start the detector**:

   ```bash
   ros2 launch aruco_detector aruco_detector.launch.py
   ```

4. **Visualise in RViz**
   Fixed frame = `zed2i_left_camera_frame`
   Add a *Detection3D* display subscribed to `/aruco_detections`.

---

## Runtime interface

| Topic                          | Type                           | Direction |
| ------------------------------ | ------------------------------ | --------- |
| `/zed2i/left/image_rect_color` | `sensor_msgs/Image`            | **Sub**   |
| `/zed2i/left/camera_info`      | `sensor_msgs/CameraInfo`       | **Sub**   |
| `/aruco_detections`            | `vision_msgs/Detection3DArray` | **Pub**   |

Default parameters (see `config/detector_params.yaml`):

```yaml
marker_size:    0.20             # metres (edge length)
rgb_topic:      "/zed2i/left/image_rect_color"
cam_info_topic: "/zed2i/left/camera_info"
```

Override at launch with `ros2 launch ... marker_size:=0.15`.

---

## Optional: camera calibration workflow

You **do not** need to calibrate the ZED 2i – its factory intrinsics are already published in every `CameraInfo`.
Use the chessboard only when:

* working with an uncalibrated USB cam, or
* you physically changed the ZED optics/lens, or
* you need sub‑millimetre accuracy.

Steps:

```bash
# 1. Capture 30‑40 chessboard images (press q to stop)
ros2 run aruco_detector cam_cal.py

# 2. Compute intrinsics from those images
ros2 run aruco_detector process.py
# -> writes camera_cal.npz (K + distortion)

# 3. Edit detector_params.yaml to point to the new camera topics
```

---

## Generate printable tags

```bash
ros2 run aruco_detector generate_markers.py
# Creates ./markers/marker_0.png ... marker_9.png (400 px, ID 0‑9)
```

Print at any size; just measure the true edge length and set `marker_size` to that value in metres.

---

## Integration into the rover launch

Add this include to `system/rover_boot/rover.launch.py`:

```python
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

aruco = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [FindPackageShare('aruco_detector'),
         '/launch/aruco_detector.launch.py']))
ld.add_action(aruco)
```

Now the node starts automatically whenever the full rover stack launches.
