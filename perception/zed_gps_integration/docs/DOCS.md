# ZED GNSS Integration - Global Localization Documentation

## Table of Contents

1. [Overview](#overview)
2. [How It Works](#how-it-works)
   - [Data Synchronization](#data-synchronization)
   - [VIO/GNSS Calibration](#viognss-calibration)
   - [Sensor Fusion](#sensor-fusion)
3. [Getting Started](#getting-started)
   - [Prerequisites](#prerequisites)
   - [Setting Up Visual-Inertial Tracking](#setting-up-visual-inertial-tracking)
   - [Setting Up Global Positional Tracking](#setting-up-global-positional-tracking)
   - [Retrieving Fused Data](#retrieving-fused-data)
4. [GNSS/RTK Setup on Linux](#gnssrtk-setup-on-linux)
   - [Installation](#installation)
   - [Usage of GPSD](#usage-of-gpsd)
   - [Enabling RTK](#enabling-rtk-on-your-gnss-module)
   - [Using GNSS in Applications](#using-gnss-in-your-applications)
5. [Coordinate Systems](#coordinate-systems)
6. [Troubleshooting](#troubleshooting)
7. [Complete Examples](#complete-examples)

---

## Overview

The ZED SDK's Global Localization module enables real-time camera tracking and localization in a global coordinate system. It leverages the power of the ZED stereo camera and GNSS data to provide accurate and robust localization capabilities for your applications.

### Key Features

- **Sensor Fusion**: Combines visual odometry with GNSS positioning for precise position and orientation
- **GNSS-Denied Operation**: Operates with centimeter precision even when satellite data is unavailable
- **Real-Time Tracking**: Provides continuous global localization in challenging environments
- **Multiple Coordinate Formats**: Supports Latitude/Longitude, ECEF, and UTM coordinate systems

---

## How It Works

Global localization in the ZED SDK is achieved by fusing data from three key sources:

1. **Stereo Vision**: Visual odometry from the ZED camera
2. **IMU**: Inertial measurement data
3. **GNSS**: Global positioning satellite data

The fusion process involves advanced algorithms that dynamically adjust the weight assigned to each sensor's input based on its real-time reliability and accuracy. Each data source compensates for the weaknesses of the others:

- When GNSS data is unreliable or unavailable, stereo vision and IMU continue to provide accurate positioning
- GNSS helps correct any drift that accumulates in visual odometry and IMU data over time

### Data Synchronization

#### General Concept

Global localization requires precise timing alignment between the ZED Camera and external GNSS data. This synchronization can be achieved through:

1. **Hardware Synchronization**: External signal triggers all sensors simultaneously
2. **Timestamp-Based Synchronization**: Data acquired in close proximity have similar timestamps

> **⚠️ Remote Data Acquisition**: If acquiring data from a remote computer, ensure proper clock synchronization using tools like Chrony or PTP on Ubuntu.

#### How Synchronization Works

The synchronization process matches data with closely related timestamps:

- **Synchronization Window**: Determined by camera FPS
  - 30 FPS → 33 milliseconds window
  - 60 FPS → 16 milliseconds window
- Data within the same window are synchronized and sent to fusion
- Adjust camera FPS using `camera_fps` attribute in `InitParameters`

#### Handling Data Drops

In networked setups, a timeout mechanism excludes camera sources when no data is received within a defined timeframe. This prevents blocking the fusion pipeline while waiting for dropped data.

Configure timeout using `timeout_period_number` attribute in `InitFusionParameters`.

---

### VIO/GNSS Calibration

#### Summary

GNSS and Visual Inertial Odometry (VIO) calibration is crucial for fusion:

- **GNSS**: Globally referenced, aligned to North
- **VIO**: Reference depends on camera's starting point

Calibration determines the world transformation to project VIO into GNSS reference frame and vice versa.

#### Calibration Parameters

The system estimates 4 parameters to align VIO and GNSS:

1. **Yaw rotation** (1 parameter): Between GNSS and VIO coordinate systems
2. **Translation** (3 parameters: X-Y-Z): Between VIO and GNSS acquisition start points

#### Coordinate System Model

The calibration transformation `T_calib` is estimated during initialization and can be retrieved using `getGeoTrackingCalibration()` method.

**Key Transformations:**

```
T_VIO_projected_in_GNSS = T_calib × T_VIO × T_antenna
T_GNSS_projected_in_VIO = inverse(T_calib) × T_GNSS × inverse(T_antenna)
```

> **💡 Tip**: Use `Camera2Geo()` and `Geo2Camera()` functions for projections instead of manual calculations.

#### Calibration Stop Criteria

Calibration halts when sufficient confidence in the `T_calib` transformation is achieved, based on uncertainty thresholds:

- **Target yaw uncertainty**: Set using `target_yaw_uncertainty` attribute
- **Target translation uncertainty**: Set using `target_translation_uncertainty` attribute
- Enable translation criteria with `enable_translation_uncertainty_target = true`

#### Quick Calibration

For applications requiring quick GeoPose estimation, enable `enable_rolling_calibration` to use initial calibration estimates even before optimal accuracy is achieved.

---

### Sensor Fusion

When VIO and GNSS are aligned, the system fuses visual-inertial data with GNSS. If GNSS drops, VIO takes over during the outage.

#### Global Localization Output

During sensor fusion, you can:

- Retrieve device's global position and orientation
- Convert between coordinate system formats using SDK conversion functions
- Access GeoPose data once calibration is complete

---

## Getting Started

This section provides step-by-step instructions for using the GeoTracking API with the ZED SDK.

### Prerequisites

- Windows 10, Ubuntu LTS, or L4T
- [ZED SDK](https://www.stereolabs.com/developers/) and dependencies ([CUDA](https://developer.nvidia.com/cuda-downloads))
- [ZED SDK Python API](https://www.stereolabs.com/docs/app-development/python/install/) (for Python)

### Setting Up Visual-Inertial Tracking

#### 1. Open the ZED Camera

**Python:**

```python
import pyzed.sl as sl

init_params = sl.InitParameters(
    depth_mode=sl.DEPTH_MODE.ULTRA,
    coordinate_units=sl.UNIT.METER,
    coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
)

zed = sl.Camera()
status = zed.open(init_params)
if status != sl.ERROR_CODE.SUCCESS:
    print("[ZED][ERROR] Camera Open: " + repr(status) + ". Exit program.")
    exit()
```

#### 2. Enable Positional Tracking

```python
pose_tracking_params = sl.PositionalTrackingParameters()
pose_tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_2()
pose_tracking_params.enable_area_memory = False

positional_init = zed.enable_positional_tracking(pose_tracking_params)
if positional_init != sl.ERROR_CODE.SUCCESS:
    print("[ZED][ERROR] Can't start tracking: " + repr(status) + ". Exit program.")
    exit()
```

#### 3. Set Region of Interest (Optional)

For precise tracking when parts of the camera view remain static:

```python
mask_roi = sl.Mat()
err = mask_roi.read(opt.roi_mask_file)
if err == sl.ERROR_CODE.SUCCESS:
    zed.set_region_of_interest(mask_roi, [sl.MODULE.ALL])
else:
    print(f"Error loading ROI file {opt.roi_mask_file}. Check the path.")
```

#### 4. Start Publishing VIO Data

```python
configuration = sl.CommunicationParameters()
zed.start_publishing(configuration)
```

---

### Setting Up Global Positional Tracking

#### 1. Initialize Fusion Object

```python
fusion = sl.Fusion()
init_fusion_param = sl.InitFusionParameters()
init_fusion_param.coordinate_units = sl.UNIT.METER
init_fusion_param.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
init_fusion_param.verbose = True

fusion_init_code = fusion.init(init_fusion_param)
if fusion_init_code != sl.FUSION_ERROR_CODE.SUCCESS:
    print("[ZED][ERROR] Failed to initialize fusion: " + repr(fusion_init_code))
    exit()
```

#### 2. Subscribe to ZED Camera Data

```python
uuid = sl.CameraIdentifier(zed.get_camera_information().serial_number)
fusion.subscribe(uuid, configuration, sl.Transform(0, 0, 0))
```

#### 3. Configure GNSS Calibration Parameters

```python
gnss_calibration_parameters = sl.GNSSCalibrationParameters()
gnss_calibration_parameters.target_yaw_uncertainty = 7e-3
gnss_calibration_parameters.enable_translation_uncertainty_target = False
gnss_calibration_parameters.target_translation_uncertainty = 15e-2
gnss_calibration_parameters.enable_reinitialization = False
gnss_calibration_parameters.gnss_vio_reinit_threshold = 5
```

#### 4. Enable Fusion Positional Tracking

```python
positional_tracking_fusion_parameters = sl.PositionalTrackingFusionParameters()
positional_tracking_fusion_parameters.enable_GNSS_fusion = True
positional_tracking_fusion_parameters.gnss_calibration_parameters = gnss_calibration_parameters

tracking_error_code = fusion.enable_positionnal_tracking(positional_tracking_fusion_parameters)
if tracking_error_code != sl.FUSION_ERROR_CODE.SUCCESS:
    print("[Fusion][ERROR] Could not start tracking: ", tracking_error_code)
    exit()
```

---

### Retrieving Fused Data

Start a loop to continuously retrieve and process fused data:

```python
while True:
    # Grab camera data
    status = zed.grab()

    # Ingest GNSS data
    input_gnss = sl.GNSSData()  # Set with your GNSS data
    ingest_error = fusion.ingest_gnss_data(input_gnss)
    if ingest_error != sl.FUSION_ERROR_CODE.SUCCESS and \
       ingest_error != sl.FUSION_ERROR_CODE.NO_NEW_DATA_AVAILABLE:
        print("Ingest error: ", ingest_error)

    # Process fusion data
    if fusion.process() == sl.FUSION_ERROR_CODE.SUCCESS:
        # Retrieve fused position
        fused_position = sl.Pose()
        current_state = fusion.get_position(fused_position)

        # Monitor calibration progress
        calibration_std = fusion.get_current_gnss_calibration_std()
        print("Calibration std:", calibration_std)

        # Retrieve GeoPose once calibration is complete
        current_geopose = sl.GeoPose()
        current_geopose_status = fusion.get_geo_pose(current_geopose)
```

---

## GNSS/RTK Setup on Linux

This guide covers setting up your GNSS/RTK module on Linux for use with the ZED SDK's Global Localization module. While focused on the u-blox ZED F9P GNSS module, these instructions apply to most GNSS modules.

### Installation

Using `gpsd` is the simplest way to retrieve GNSS data on Linux. It manages USB GPS devices and shares the receiver across all applications.

#### Install gpsd from Source

```bash
# Install dependencies
sudo apt update && sudo apt install scons libgtk-3-dev

# Compile latest gpsd version
git clone https://gitlab.com/gpsd/gpsd.git
cd gpsd && git checkout 8910c2b60759490ed98970dcdab8323b957edf48
sudo ./gpsinit vcan
scons && scons check && sudo scons udev-install

# Add Python path to .bashrc
echo 'export PYTHONPATH="$PYTHONPATH:/usr/local/lib/python3/dist-packages"' >> ~/.bashrc
```

#### Configure User Permissions

Add your user to required groups to run without root:

```bash
sudo adduser $USER dialout
sudo adduser $USER tty
```

**Log out and back in** for changes to take effect.

#### Verify GNSS Module Detection

```bash
ls /dev/tty*
```

You should see `/dev/ttyACM0` or `/dev/ttyUSB0`. Test raw data stream:

```bash
cat /dev/ttyACM0
```

Expected output:

```
$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,1*33
$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,2*30
$GPGSV,1,1,00,0*65
$GLGSV,1,1,00,0*79
$GNGLL,,,,,082018.00,V,N*57
$GNRMC,082019.00,V,,,,,,,230623,,,N,V*1D
```

---

### Usage of GPSD

Run the daemon manually instead of using systemctl service (more reliable for USB modules):

```bash
gpsd -nG -s 115200 /dev/ttyACM0
```

> **📌 Note**: Replace `ttyACM0` with your GNSS sensor's communication port.

#### Start GPSD at Boot

Add a cron job with `crontab -e`:

```cron
@reboot sleep 10 && /usr/local/sbin/gpsd -nG -s 115200 /dev/ttyACM0
```

#### Test GPSD

Use the `xgps` tool to verify proper operation:

```bash
xgps
```

---

### Enabling RTK on Your GNSS Module

RTK (Real-Time Kinematic) provides centimeter-level accuracy compared to meter-level accuracy of standard GNSS. It uses carrier-phase measurements and corrections from reference stations to overcome atmospheric disturbances and signal multipath.

#### Using NTRIP

`gpsd` can act as an NTRIP client to retrieve RTK corrections from a base station.

**Required Information:**

- `url`: NTRIP URL address of base station
- `port`: NTRIP port of base station
- `mountpoint`: Mountpoint to use (base station should be < 25km from rover)
- `username`: NTRIP connection username (optional)
- `password`: NTRIP connection password (optional)

**Run gpsd as NTRIP Client:**

```bash
pkill gpsd  # Kill existing gpsd instance
gpsd -nG ntrip://<username>:<password>@<url>:<port>/<mountpoint> -s 115200 /dev/ttyACM0
```

> **📌 Note**: Replace `ttyACM0` with your GNSS sensor's communication port.

Use `xgps` to monitor RTK fix status. ECEF pAcc (horizontal accuracy) should be a few centimeters with RTK FIX status.

#### Set RTK Configuration at Boot

Update the cron job to connect to base station at every boot:

```bash
crontab -e
```

Replace the gpsd line with:

```cron
@reboot sleep 10 && /usr/local/sbin/gpsd -nG ntrip://<username>:<password>@<url>:<port>/<mountpoint> -s 115200 /dev/ttyACM0
```

---

### Using GNSS in Your Applications

#### Python

Use the `gpsdclient` library to access GNSS data:

```bash
pip install gpsdclient
```

**Example:**

```python
from gpsdclient import GPSDClient

# Get data as JSON strings
with GPSDClient(host="127.0.0.1") as client:
    for result in client.json_stream():
        print(result)

# Get data as Python dicts with datetime conversion
with GPSDClient() as client:
    for result in client.dict_stream(convert_datetime=True, filter=["TPV"]):
        print("Latitude: %s" % result.get("lat", "n/a"))
        print("Longitude: %s" % result.get("lon", "n/a"))

# Filter by report class
with GPSDClient() as client:
    for result in client.dict_stream(filter=["TPV", "SKY"]):
        print(result)
```

#### C++

Use the `libgpsmm` library:

```bash
sudo apt install libgps-dev
```

**Example:**

```cpp
#include <libgpsmm.h>
#include <iostream>

int main() {
    gpsmm gps_data("localhost", DEFAULT_GPSD_PORT);

    if (gps_data.stream(WATCH_ENABLE | WATCH_JSON) == nullptr) {
        std::cerr << "Failed to open GPS connection." << std::endl;
        return 1;
    }

    while (true) {
        if (gps_data.waiting(500)) {
            if (gps_data.read() == nullptr) {
                std::cerr << "Error reading GPS data." << std::endl;
            } else {
                std::cout << "Latitude: " << gps_data.fix->latitude 
                          << ", Longitude: " << gps_data.fix->longitude << std::endl;
            }
        }
    }

    gps_data.stream(WATCH_DISABLE);
    gps_data.close();
    return 0;
}
```

---

## Coordinate Systems

Global coordinates can be represented in various formats supported by the SDK:

### Latitude/Longitude

Geographic coordinate system for specifying Earth surface locations:

- **Latitude**: North-south angular position relative to equator
- **Longitude**: East-west angular position relative to Greenwich

Default format for GeoPose. Represented as `LatLng` object in SDK.

[More information on Wikipedia](https://en.wikipedia.org/wiki/Geographic_coordinate_system)

### ECEF (Earth-Centered Earth-Fixed)

Three-dimensional Cartesian coordinate system:

- Origin: Earth's center of mass
- Axes: Aligned with Earth's rotation
- Use cases: Satellite navigation, geodesy, aerospace engineering

Represented as `ECEF` object in SDK.

[More information on Wikipedia](https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system)

### UTM (Universal Transverse Mercator)

Two-dimensional representation dividing Earth into 60 zones (6° longitude each):

- Uses metric grid (eastings and northings)
- Coordinates relative to zone's central meridian and equator
- Use cases: Topographic mapping, surveying, military operations

Represented as `UTM` object in SDK.

[More information on Wikipedia](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system)

### Converting Coordinate Systems

Use the ZED SDK's Geo-Converter functions to convert between coordinate formats.

---

## Troubleshooting

### MODULE_NOT_ENABLED

**Message:**
```
Positional tracking not enabled for the GeoTracking module. 
Did you call enablePositionalTracking() on the sl::Fusion object?
```

**What's Happening:**
VPS functions cannot be used without enabling the module.

**Solution:**
Call `enablePositionalTracking()` on the `sl::Fusion` object before using fusion functions.

---

### INVALID_COVARIANCE

**Message:**
```
Ingested GNSS data with very low covariance value (< 1mm). 
Covariance will be clamped to 1mm. Please verify covariance values.
```

**What's Happening:**
Covariance value for ingested `GNSSData` is below 1e-6 threshold. Fusion clamps it to 1e-6.

**Impact:**
Warning message only. Fusion proceeds with clamped covariance.

**Solution:**
Verify covariance values before ingesting GNSS data.

---

### INVALID_TIMESTAMP

**Messages:**
- `Ingested GNSS data without timestamp (timestamp = 0)`
- `Ingested GNSS data with timestamp far from current timestamp`

**What's Happening:**
Fusion synchronizes data from multiple sources based on timestamps. Error occurs when:

1. Timestamp equals zero (not set)
2. Timestamp is in the past relative to synchronization cursor
3. Timestamp is too far from current synchronization cursor

**Impact:**
Fusion ignores invalid data since it cannot be synchronized.

**Solution:**
- Check `GNSSData` timestamp before ingesting
- Retrieve current synchronization cursor using `getCurrentTimeStamp()` method

---

### NO_NEW_DATA_AVAILABLE

**What's Happening:**
Fusion consumed all available camera data and is waiting for new data. Usually occurs when `process()` runs at higher FPS than camera `grab()`.

**Impact:**
Warning message only. Fusion waits for new camera data.

**Solution:**
Normal behavior. If no SUCCESS after several seconds, refer to synchronization documentation.

---

### GNSS_DATA_NEED_FIX

**Message:**
```
gnss_status attribute not filled. Please fill it for improved accuracy 
or set gnss_status to SINGLE.
```

**What's Happening:**
`gnss_status` attribute of `GNSSData` is set to `GNSS_STATUS::UNKNOWN`.

**Impact:**
Warning message. Fusion processes data in degraded mode without status.

**Solution:**
Set `gnss_status` attribute properly. If status unavailable, set to `SINGLE`.

---

### GNSS_DATA_COVARIANCE_MUST_VARY

**Message:**
```
Ingested GNSS data with same covariance multiple times. 
Verify you're filling it with correct values.
```

**What's Happening:**
More than 15 consecutive `GNSSData` entries have identical covariance values.

**Impact:**
Warning message only. Prevents using fixed "hand-crafted" covariance values.

**Solution:**
Verify covariance values are correct. To disable warning:

```bash
export FUSION_SDK_DISABLE_GNSS_COVARIANCE_CHECK=1
```

---

### SENSORS_DATA_REQUIRED

**Messages:**
- `Positional Tracking GEN2 with IMU fusion requires high frequency sensors data (Streaming v2, SDK >=4.1)`
- `Positional Tracking GEN2 with IMU fusion requires high frequency sensors data (SVO v2, SDK >=4.1)`

**What's Happening:**
Positional tracking GEN 2 needs high-frequency IMU data when `enable_imu_fusion` is activated. Input (SVO/streaming) doesn't provide this data.

**Check SVO Version:**

```bash
ZED_SVO_Editor --version <path_of_your_svo>
```

**Impact:**
Fatal error. Positional tracking deactivated.

**Solution:**
Set `enable_imu_fusion = false` to use data source with positional tracking GEN 2.

---

## Complete Examples

### Python Example: Global Localization with GNSS

```python
########################################################################
#
# Copyright (c) 2025, STEREOLABS.
#
# All rights reserved.
#
########################################################################

"""
This sample shows how to fuse the ZED camera position with external GNSS
"""

import sys
import pyzed.sl as sl


if __name__ == "__main__":

    # Variables
    camera_pose = sl.Pose()    
    odometry_pose = sl.Pose()    
    py_translation = sl.Translation()
    pose_data = sl.Transform()
    text_translation = ""
    text_rotation = ""   

    # Create ZED camera object
    init_params = sl.InitParameters(
        camera_resolution=sl.RESOLUTION.AUTO,
        coordinate_units=sl.UNIT.METER,
        coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    )
                                 
    # Step 1: Create camera for odometry input
    zed = sl.Camera()
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print("Camera Open: " + repr(status) + ". Exit program.")
        exit()
    
    # Set up communication and start publishing
    communication_parameters = sl.CommunicationParameters()
    communication_parameters.set_for_shared_memory()
    zed.start_publishing(communication_parameters)

    # Warmup camera 
    if zed.grab() != sl.ERROR_CODE.SUCCESS:
        print("Camera grab: " + repr(status) + ". Exit program.")
        exit()
    else:
        zed.get_position(odometry_pose, sl.REFERENCE_FRAME.WORLD)

    tracking_params = sl.PositionalTrackingParameters()
    # Mandatory parameters for GNSS/ZED transformation
    tracking_params.enable_imu_fusion = True
    tracking_params.set_gravity_as_origin = True
    err = zed.enable_positional_tracking(tracking_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera positional tracking: " + repr(status) + ". Exit program.")
        exit()
    camera_info = zed.get_camera_information()

    # Step 2: Initialize fusion module
    fusion = sl.Fusion()
    init_fusion_parameters = sl.InitFusionParameters()
    init_fusion_parameters.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_fusion_parameters.coordinate_units = sl.UNIT.METER

    fusion.init(init_fusion_parameters)
    positional_tracking_fusion_parameters = sl.PositionalTrackingFusionParameters()
    fusion.enable_positionnal_tracking(positional_tracking_fusion_parameters)
    
    uuid = sl.CameraIdentifier(camera_info.serial_number)
    print("Subscribing to", uuid.serial_number, communication_parameters.comm_type)
    status = fusion.subscribe(uuid, communication_parameters, sl.Transform(0, 0, 0))
    if status != sl.FUSION_ERROR_CODE.SUCCESS:
        print("Failed to subscribe to", uuid.serial_number, status)
        exit(1)

    x = 0
    i = 0
    
    while i < 200:
        # Get odometry information
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.get_position(odometry_pose, sl.REFERENCE_FRAME.WORLD)
        elif zed.grab() == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            break

        # Dummy GPS value (replace with actual GNSS data)
        x = x + 0.000000001
        gnss_data = sl.GNSSData()
        gnss_data.ts = sl.get_current_timestamp()

        # Set GPS coordinates: latitude, longitude, altitude
        gnss_data.set_coordinates(x, 0, 0)

        # Set covariance (3x3 matrix in row-major order)
        covariance = [  
            1, 0.1, 0.1,
            0.1, 1, 0.1,
            0.1, 0.1, 1
        ]

        gnss_data.position_covariances = covariance
        fusion.ingest_gnss_data(gnss_data)

        # Get fused position
        if fusion.process() == sl.FUSION_ERROR_CODE.SUCCESS:
            fused_tracking_state = fusion.get_position(camera_pose, sl.REFERENCE_FRAME.WORLD)
            if fused_tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                
                rotation = camera_pose.get_rotation_vector()
                translation = camera_pose.get_translation(py_translation)
                text_rotation = str((round(rotation[0], 2), round(rotation[1], 2), round(rotation[2], 2)))
                text_translation = str((round(translation.get()[0], 2), round(translation.get()[1], 2), round(translation.get()[2], 2)))
                pose_data = camera_pose.pose_data(sl.Transform())
                print("Position translation =", text_translation, ", rotation =", text_rotation)
        
        i += 1

    zed.close()
```

### Tutorial Overview

This tutorial demonstrates using the ZED as a positional tracker with GNSS equipment. The program loops until 200 positions are captured.

**Key Steps:**

1. **Create Camera**: Configure and open the ZED camera
2. **Setup Fusion**: Initialize fusion module for camera and GNSS
3. **Capture Pose Data**: Loop to grab and retrieve fused camera position

The example displays camera translation in meters and rotation in the console before exiting.

---

## Additional Resources

- [ZED SDK Documentation](https://www.stereolabs.com/developers/)
- [Global Localization Samples on GitHub](https://github.com/stereolabs/zed-examples)
- Complete samples available for:
  - Live fused GNSS global data visualization
  - GNSS sequence recording
  - Replay for testing purposes

---

© 2025 Stereolabs Inc. All Rights Reserved.
