# ZED GPS Integration - Quick Start Guide

## What You'll Get
- **Live GPS tracking** with ZED visual-inertial odometry
- **Real-time web map** showing your position and trajectory
- **Data fusion** combining GPS accuracy with ZED smoothness

## Prerequisites
- ZED 2i camera connected
- u-blox F9P GPS receiver connected to `/dev/ttyACM0`
- Clear sky view for GPS signal (go outside!)

---

## 🚀 OUTDOOR TEST - Complete Setup

### Step 1: Kill Any Running Processes (5 seconds)
```bash
pkill -9 -f ros && sleep 2
```

### Step 2: Launch GPS Driver (15 seconds)
**Terminal 1:**
```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/ros2-ublox-zedf9p/install/setup.bash
ros2 launch ~/workspaces/rover/install/gnss_launch/share/gnss_launch/launch/gnss.launch.py
```

**Wait for:** `U-Blox configured successfully` (~15 seconds)

### Step 3: Verify GPS Signal (wait 30-60 seconds)
**Terminal 2:**
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /gps/fix --once
```

**Look for:**
- `status: 0` or higher (NOT -1)
- Non-zero latitude/longitude
- If still `NO_FIX`, wait longer or move to better location

### Step 4: Launch ZED Camera (30 seconds)
**Terminal 3:**
```bash
cd ~/workspaces/rover
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zed2i_launch zed2i_driver.launch.py
```

**Wait for:** `Camera successfully opened` message

### Step 5: Start Map Server (5 seconds)
**Terminal 4:**
```bash
cd ~/workspaces/rover
source /opt/ros/humble/setup.bash
source install/setup.bash
~/workspaces/rover/install/zed_gps_integration/bin/map_server
```

**You should see:**
```
🗺️  Map Server Bridge started
   Data directory: /home/rover/zed_map_data
```

### Step 6: Launch Web Map (10 seconds)
**Terminal 5:**
```bash
cd ~/zed_map_data
python3 -m http.server 8000
```

### Step 7: Open Map in Browser
**On Jetson desktop:**
```bash
DISPLAY=:0 xdg-open http://localhost:8000/
```

**Or manually open:** `http://localhost:8000/`

---

## 📍 What You Should See

### On the Web Map:
- **🟡 Yellow marker**: Your current GPS position
- **Yellow trail**: Path you've traveled (GPS data)
- **Status panel** (top-right): 
  - Current latitude, longitude, altitude
  - GPS fix status (NO_FIX → SINGLE → DGPS → RTK)
  - Number of satellites visible
- **Auto-centering**: Map follows your position
- **Layer toggle**: Switch between OpenStreetMap and Satellite view

### Current Status (Underground/Indoors):
```
Waiting for GPS data...
Status: NO_FIX
Position: 0.0, 0.0
```

### After Going Outside (30-60 seconds):
```
Status: SINGLE (or DGPS/RTK)
Position: 37.123456, -122.123456
Altitude: 123.4m
Satellites: 12
```

### In Terminals:
**GPS Terminal:** 
```
[INFO] [gnss_driver]: U-Blox configured successfully
```

**ZED Terminal:**
```
[INFO] [zed2i_camera]: Camera successfully opened
```

**Map Server Terminal:**
```
[INFO] [map_server]: 🗺️ Map Server Bridge started
[INFO] [map_server]:    Receiving GPS data at 10 Hz
```

---

## 🎯 How It Actually Works

### Current Implementation:
1. **GPS Driver** publishes raw GPS data to `/gps/fix` at 10 Hz
2. **ZED Camera** publishes visual-inertial odometry to `/zed2i/zed2i_camera/odom` at 30 Hz
3. **Map Server** reads GPS data and writes to `~/zed_map_data/raw_data.txt`
4. **Web Map** reads the text file every second and updates markers
5. **Both systems run independently** - no fusion yet (see note below)

### Data Flow:
```
GPS Receiver → ublox_gps_node → /gps/fix → map_server → raw_data.txt → Web Map
ZED Camera → zed_wrapper → /zed2i/zed2i_camera/odom (local frame only)
```

### Note on Fusion:
The current setup demonstrates **both systems working together**. True sensor fusion (combining GPS with ZED VIO into a single fused output) requires the ZED SDK's Fusion API, which needs the camera to be opened with fusion enabled from the start. This is prepared in the codebase but requires architectural changes to integrate properly with the existing ZED wrapper.

---

## 🔧 Troubleshooting

### "Waiting for GPS data..." (stays forever)
- **Outside?** Must have clear sky view, no buildings/trees blocking
- **Wait longer**: First fix can take 60-120 seconds
- **Check GPS**: `ros2 topic hz /gps/fix` (should show ~10 Hz)
- **Check hardware**: `ls /dev/ttyACM*` (should show `/dev/ttyACM0`)

### Map shows (0, 0) in Africa
- GPS has NO_FIX
- Go outside and wait for satellite lock
- Check: `ros2 topic echo /gps/fix --once` for status

### ZED camera won't start
- Another process using camera? Run: `pkill -f zed`
- Camera unplugged? Check: `lsusb | grep -i stereo`
- Wait 30 seconds after killing processes

### Map not updating
- Check web server running: `ps aux | grep http.server`
- Check data file: `cat ~/zed_map_data/raw_data.txt`
- Refresh browser (Ctrl+F5)
- Check map_server running: `ps aux | grep map_server`

---

## 📊 Quick Diagnostics

```bash
# All-in-one status check
source /opt/ros/humble/setup.bash
echo "=== ROS NODES ===" && ros2 node list | grep -E "(gnss|zed|map)"
echo "=== GPS STATUS ===" && ros2 topic echo /gps/fix --once | grep -E "(status|latitude|longitude)"
echo "=== ZED STATUS ===" && ros2 topic hz /zed2i/zed2i_camera/odom --window 10
echo "=== MAP DATA ===" && cat ~/zed_map_data/raw_data.txt
```

---

## 🎬 Complete Single-Command Outdoor Test

Once outside, use this **one-liner** to quickly test everything:

```bash
# Save this as ~/test_gps_integration.sh
pkill -9 -f ros && sleep 3 && \
(source /opt/ros/humble/setup.bash && source ~/workspaces/ros2-ublox-zedf9p/install/setup.bash && ros2 launch ~/workspaces/rover/install/gnss_launch/share/gnss_launch/launch/gnss.launch.py &) && \
sleep 15 && \
(cd ~/workspaces/rover && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch zed2i_launch zed2i_driver.launch.py &) && \
sleep 25 && \
(cd ~/workspaces/rover && source /opt/ros/humble/setup.bash && source install/setup.bash && ~/workspaces/rover/install/zed_gps_integration/bin/map_server &) && \
sleep 5 && \
(cd ~/zed_map_data && python3 -m http.server 8000 &) && \
sleep 2 && \
DISPLAY=:0 xdg-open http://localhost:8000/ && \
echo "✅ All systems launched! Check browser for live map."
```

---

**Pro Tip:** Once you have GPS fix, walk around in a circle or drive the rover. You'll see the yellow trail drawing your path in real-time! 🗺️✨

