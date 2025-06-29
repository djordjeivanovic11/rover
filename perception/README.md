### Perception module

#### **`zed2i_launch/` ― the sensor gateway**

This package is nothing but a **launch wrapper and a YAML** for Stereolabs’ `zed_camera` node. The YAML (`params/zed2i.yaml`) tells the ZED 2i SDK to run 720 p @ 30 Hz, publish depth in **ULTRA** mode, enable the on-board IMU and visual-inertial odometry, and stream a registered point cloud. The launch script (`launch/zed2i_driver.launch.py`) starts that node and nothing else. As soon as it is up your ROS graph gains:

* `/zed2i/left|right/image_rect_color` – rectified stereo images
* `/zed2i/depth/depth_registered` – per-pixel depth
* `/zed2i/imu/data` – 200 Hz fused IMU
* `/zed2i/odom` – 6-DoF visual-inertial Odometry
* `/zed2i/point_cloud/cloud_registered` – live XYZRGBA cloud

Every other perception component consumes these topics instead of talking to the camera directly, so if you ever swap the ZED for another depth sensor you only touch **this** folder.

---

#### **`gnss_launch/` ― global position source**

`gnss_launch` is the symmetric twin of `zed2i_launch`: a **launch-only helper** that starts u-blox GNSS driver with the parameters in `params/ublox.yaml`. The node is remapped so its `fix` output lands on the canonical `/gnss/fix` topic. In other words, this folder injects one message into the graph—`sensor_msgs/NavSatFix`—and leaves all fusion logic to the next layer.

---

#### **`loc_fusion/` ― anchoring odometry to GPS**

This package hosts the **localisation backbone**. Three standard `robot_localization` nodes are launched in `launch/loc_fusion.launch.py`:

| Node                    | Inputs                             | Output / TF                                 |
| ----------------------- | ---------------------------------- | ------------------------------------------- |
| `ekf_local`             | ZED odom + IMU                     | `/odometry/filtered`, **odom → base\_link** |
| `navsat_transform_node` | `/gnss/fix` + `/odometry/filtered` | `/odom/gps`                                 |
| `ekf_global`            | `/odometry/filtered` + `/odom/gps` | `/odometry/global`, **map → odom**          |

The single YAML (`config/ekf.yaml`) defines both EKF instances, while `navsat_transform.yaml` sets the GPS projection rules (zero altitude, declination, etc.). Because this layer also **includes** the ZED and GNSS launch files, an upstream package only needs to include `loc_fusion` to obtain a rock-solid TF tree:

```
map ─► odom (GPS-corrected) ─► base_link (smooth VIO)
```

Everything downstream—SLAM, Nav2, MoveIt—relies on that tree for consistent geometry.

---

#### **`pointcloud_tools/` ― 2-D products from 3-D data**

SLAM provides a global grid but local planners often need faster, planar information. `pointcloud_tools` offers two pure-Python nodes:

* **`depth_to_scan_node.py`** slices the front half of the ZED cloud, bins points by angle, and publishes a **synthetic LaserScan** on `/scan`. Nav2’s local costmap or RViz’s lidar display can consume it without any custom plugins.
* **`grid_builder_node.py`** transforms an incoming cloud (default `/rtabmap/cloud_map`) into a rolling **OccupancyGrid** centred on the robot (`/local_map`). Cells with z > 0.15 m become **occupied** (100), ground-level cells become **free** (0), unobserved stay −1.

Execution and ROS registration are handled by `setup.py`, and default parameters (FOV, z-slice, grid size) live in `config/pointcloud_params.yaml`. Launching `launch/pointcloud_tools.launch.py` spins up both nodes; you can then feed `/scan` to Nav2’s obstacle layer and `/local_map` to the gap-guidance module.

---

#### **`slam_launch/` ― the one-button perception stack**

`slam_launch` glues everything:

1. **Camera** – includes `zed2i_launch`.
2. **Localisation** – includes `loc_fusion`.
3. **Global mapping** – starts **RTAB-Map** with the tuning file `config/rtabmap_params.yaml`. RGB and depth topics are remapped from the ZED namespace; odometry comes from `/odometry/filtered`.

RTAB-Map outputs a continually-optimised 3-D cloud (`/rtabmap/cloud_map`) and a latched 2-D grid on `/map`. Thus, after

```bash
ros2 launch slam_launch slam.launch.py
```

the ROS graph contains **every** perception artefact required by autonomy:

* TF: `map → odom → base_link`
* Local odom (`/odometry/filtered`) and global odom (`/odometry/global`)
* Global grid `/map` (Nav2 StaticLayer)
* Live local grid `/local_map` + LaserScan `/scan` (Nav2 obstacle layer)
* Raw point cloud for gap steering (`/zed2i/point_cloud/cloud_registered`)

Higher-level folders—`global_nav`, `local_nav`, `mission_control`—only need to **include this single launch description** to inherit a fully-functional, sensor-agnostic perception stack. Swapping hardware (another GNSS, a different depth camera) or changing mapping parameters now affects exactly one sub-package, keeping the rest of the repository isolated and easy to maintain.
