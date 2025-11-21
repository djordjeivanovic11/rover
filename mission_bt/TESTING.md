# GPS Waypoint Navigation - Field Testing Guide

Complete testing procedure for GPS waypoint navigation on the URC rover with ZED camera and GPS hardware.

## Pre-Flight Checklist

### 1. Verify Hardware Connections

```bash
# Check GPS is publishing
ros2 topic list | grep gps
# Should see: /gps/fix

ros2 topic echo /gps/fix --once
# Should show actual GPS coordinates with status >= 0

# Check ZED is running
ros2 topic list | grep zed
# Should see: /zed/zed_node/odom, /zed/zed_node/pose, etc.
```

### 2. Verify Localization is Running

```bash
# Check if robot_localization is running
ros2 node list | grep ekf
# Should see: /ekf_node or similar

# Check TF tree exists
ros2 run tf2_ros tf2_echo map base_link
# Should show live transform updates
```

---

## Launch Sequence for Real Hardware

### Terminal 1: Launch Localization

```bash
cd ~/workspaces/rover
source install/setup.bash

# Launch your localization stack (adjust to your setup)
ros2 launch loc_fusion loc_fusion.launch.py
# This should fuse ZED odometry + IMU + GPS
```

**Wait for:** "Localization active" or similar confirmation

---

### Terminal 2: Launch Nav2 + Drive Control

```bash
source install/setup.bash

# Launch complete navigation stack
ros2 launch nav2_launch nav2_bringup.launch.py \
  track_width:=0.42 \
  wheel_radius:=0.105
```

**Wait for:** All Nav2 lifecycle nodes to become active (~10-15 seconds)

**Verify Nav2 is ready:**
```bash
ros2 node list | grep -E "(controller|planner|bt_navigator)"
# Should show: controller_server, planner_server, bt_navigator
```

---

### Terminal 3: Launch GPS Waypoint Navigator

```bash
source install/setup.bash

ros2 launch gps_waypoint_navigator gps_navigator.launch.py
```

**Expected output:**
```
[gps_navigator]: GPSNavigatorNode ready (action: navigate_to_gps)
[waypoint_sequencer]: WaypointSequencer ready (action: follow_gps_waypoints)
[named_waypoint_node]: NamedWaypointNode ready (action: go_to_named_waypoint)
[named_waypoint_node]: Loading waypoints from: .../waypoints.yaml
```

**Verify GPS fix:**
```bash
# Terminal 4
ros2 topic echo /gps/fix --once
```

Check:
- `status.status` should be `>= 0` (means GPS has fix)
- `latitude` and `longitude` show real coordinates
- `position_covariance` shows reasonable accuracy (< 10m)

---

## Record Your Current Position as a Waypoint

Before testing navigation, record where you are:

```bash
source install/setup.bash

# Record current position as "test_start"
ros2 run gps_waypoint_navigator record_waypoint --name test_start

# This saves to: src/navigation/gps_waypoint_navigator/config/waypoints.yaml
```

**Verify it was saved:**
```bash
cat src/navigation/gps_waypoint_navigator/config/waypoints.yaml
# Should show test_start with your current lat/lon
```

---

## Test 1: Navigate to a Nearby Point

### Step 1: Mark a target location

Walk the rover 10-20 meters away and record that position:

```bash
ros2 run gps_waypoint_navigator record_waypoint --name test_target
```

### Step 2: Drive back to start

Drive the rover back to your original position manually.

### Step 3: Command autonomous navigation

```bash
source install/setup.bash

# Navigate to the test target
ros2 action send_goal /go_to_named_waypoint urc_msgs/action/GoToNamedWaypoint \
  "{waypoint_name: 'test_target', max_retries: 2}" \
  --feedback
```

**What should happen:**
1. Node looks up "test_target" coordinates
2. Converts GPS → map frame coordinates
3. Sends goal to Nav2
4. Rover plans path and drives to target
5. Action returns SUCCESS when close enough

**Monitor progress:**
```bash
# Terminal 5: Watch GPS navigator status
ros2 topic echo /gps_navigator/status

# Terminal 6: Watch Nav2 feedback
ros2 topic echo /navigate_to_pose/_action/feedback
```

---

## Test 2: Multi-Waypoint Mission

### Step 1: Record multiple waypoints

Create a simple route:

```bash
# Position 1
ros2 run gps_waypoint_navigator record_waypoint --name wp1

# Drive 10m north
# Position 2
ros2 run gps_waypoint_navigator record_waypoint --name wp2

# Drive 10m east
# Position 3
ros2 run gps_waypoint_navigator record_waypoint --name wp3

# Return to start
ros2 run gps_waypoint_navigator record_waypoint --name wp_home
```

### Step 2: Run the mission

```bash
source install/setup.bash

# Launch mission executor
ros2 launch mission_bt mission_bt.launch.py \
  waypoint_sequence:="['wp1','wp2','wp3','wp_home']"
```

**The rover should:**
1. Navigate to wp1 autonomously
2. Navigate to wp2
3. Navigate to wp3
4. Return to wp_home
5. Report mission complete

---

## Test 3: Follow GPS Waypoint Sequence (Direct Coordinates)

Test the `FollowGPSWaypoints` action directly:

```bash
# Get current lat/lon from GPS
ros2 topic echo /gps/fix --once | grep -E "(latitude|longitude)"
# Note these values

# Create a small square pattern (adjust values based on your location)
ros2 action send_goal /follow_gps_waypoints urc_msgs/action/FollowGPSWaypoints \
  "{latitudes: [38.4063, 38.4065, 38.4065, 38.4063], \
    longitudes: [-110.7918, -110.7918, -110.7920, -110.7920], \
    waypoint_names: ['corner1', 'corner2', 'corner3', 'corner4'], \
    max_retries: 2, \
    skip_on_failure: true, \
    abort_on_first_failure: false}" \
  --feedback
```

---

## Test 4: Raw GPS Coordinate Navigation

Test the lowest-level `NavigateToGPS` action:

```bash
# Navigate to specific GPS coordinates (replace with actual target)
ros2 action send_goal /navigate_to_gps urc_msgs/action/NavigateToGPS \
  "{latitude: 38.4070, longitude: -110.7920, waypoint_name: 'raw_test', max_retries: 2}" \
  --feedback
```

---

## Monitoring Commands During Test

### Check Current GPS Position
```bash
ros2 topic echo /gps/fix --once | grep -E "(latitude|longitude)"
```

### Check Robot Pose in Map
```bash
ros2 run tf2_ros tf2_echo map base_link | head -5
```

### Check GPS Navigator Status
```bash
ros2 topic echo /gps_navigator/status
# Shows: IDLE, NAVIGATING, RETRYING, SUCCESS, or FAILED
```

### Check Nav2 Goal Status
```bash
ros2 action list -t | grep navigate_to_pose
ros2 topic echo /navigate_to_pose/_action/status
```

### Check Distance to Goal
```bash
ros2 topic echo /navigate_to_gps/_action/feedback
# Shows: distance_remaining in feedback
```

### Check All Active Action Servers
```bash
ros2 action list
# Should show:
#   /follow_gps_waypoints
#   /go_to_named_waypoint
#   /navigate_to_gps
#   /navigate_to_pose (from Nav2)
```

---

## Emergency Stop / Cancel

If something goes wrong:

```bash
# Method 1: Cancel current waypoint goal
ros2 action cancel_goal /go_to_named_waypoint

# Method 2: Cancel GPS navigation goal
ros2 action cancel_goal /navigate_to_gps

# Method 3: Cancel mission sequence
# Ctrl+C the mission executor terminal

# Method 4: Kill Nav2 (emergency stop)
ros2 lifecycle set /controller_server deactivate

# Method 5: Physical E-stop
# Press emergency stop button on rover
```

---

## Troubleshooting Common Issues

### Issue: "No GPS fix yet - aborting"

**Check:**
```bash
ros2 topic echo /gps/fix --once
```

**Look for:**
- `status.status` should be `>= 0` (negative = no fix)
- `latitude` and `longitude` should be non-zero
- `position_covariance` indicates accuracy

**Solution:**
- Wait 30-60s for GPS to acquire satellites
- Make sure you're outdoors with clear sky view
- Check GPS antenna is connected properly
- Move away from buildings/trees that block signal

---

### Issue: "No TF map->base_link"

**Check:**
```bash
ros2 run tf2_ros view_frames
# Creates frames.pdf showing TF tree
evince frames.pdf
# Should show: map → odom → base_link chain
```

**Solution:**
- Ensure `loc_fusion` (robot_localization + navsat_transform) is running
- Check ZED odometry is publishing: `ros2 topic hz /zed/zed_node/odom`
- Verify navsat_transform node is active: `ros2 node list | grep navsat`

---

### Issue: "Nav2 not available"

**Check:**
```bash
ros2 node list | grep nav2
ros2 action list | grep navigate_to_pose
```

**Solution:**
- Launch Nav2: `ros2 launch nav2_launch nav2_bringup.launch.py`
- Wait for all lifecycle nodes to activate (~15 sec)
- Check for errors in Nav2 terminal
- Verify map is loaded (if using pre-built map)

---

### Issue: "NavigateToGPS action server not available"

**Check:**
```bash
ros2 node list | grep gps
ros2 action list | grep navigate_to_gps
```

**Solution:**
- Launch GPS navigator: `ros2 launch gps_waypoint_navigator gps_navigator.launch.py`
- Wait 5 seconds for nodes to initialize
- Check for errors in gps_navigator terminal

---

### Issue: Navigation starts but rover doesn't move

**Check:**
```bash
# Is Nav2 publishing cmd_vel?
ros2 topic echo /cmd_vel

# Is drive_control receiving it?
ros2 topic hz /cmd_vel

# Are wheel commands being sent?
ros2 topic echo /drive/left_rpm
ros2 topic echo /drive/right_rpm
```

**Solution:**
- Verify `drive_control` is running
- Check E-stop is not engaged (both software and hardware)
- Verify motors have power
- Check wheel_bridge is running and connected to motor controllers
- Test manual teleop to verify drive system works

---

### Issue: Rover drives in wrong direction

**Possible causes:**
- GPS coordinates flipped (lat/lon swapped)
- Reference point not initialized correctly
- Map orientation issue
- TF frame conventions incorrect

**Debug:**
```bash
# Check where GPS converter thinks target is:
# In gps_navigator logs, look for:
# "GPSConverter reference set: lat=X, lon=Y, map=(x, y)"
# "Target pose: x=X, y=Y"

# Check current position
ros2 run tf2_ros tf2_echo map base_link

# Compare with GPS
ros2 topic echo /gps/fix --once
```

**Solution:**
- Verify map is oriented correctly (North should be +Y typically)
- Check ZED camera forward direction matches base_link +X
- Ensure navsat_transform is configured correctly

---

### Issue: "Waypoint 'X' not found"

**Check:**
```bash
cat install/gps_waypoint_navigator/share/gps_waypoint_navigator/config/waypoints.yaml
```

**Solution:**
- Rebuild after adding waypoints: `colcon build --packages-select gps_waypoint_navigator`
- Source workspace: `source install/setup.bash`
- Or use absolute path for record_waypoint:
  ```bash
  ros2 run gps_waypoint_navigator record_waypoint \
    --file /home/rover/workspaces/rover/src/navigation/gps_waypoint_navigator/config/waypoints.yaml \
    --name X
  ```

---

### Issue: Navigation succeeds but lands far from target

**Possible causes:**
- GPS accuracy poor (> 5m error)
- Local ENU approximation breaking down over large distances
- Reference point drift

**Check:**
```bash
# Check GPS accuracy
ros2 topic echo /gps/fix --once | grep covariance
# Values < 25 (5m std dev) are good

# Check distance to target
ros2 topic echo /navigate_to_gps/_action/feedback
```

**Solution:**
- Wait for better GPS fix quality
- Keep navigation distances < 500m for best accuracy
- Consider using RTK GPS for cm-level accuracy
- Tune Nav2 goal tolerance in parameters

---

### Issue: Mission executor fails immediately

**Check:**
```bash
# Check mission executor logs
ros2 run mission_bt mission_executor

# Verify waypoint sequence parameter
ros2 param get /mission_executor waypoint_sequence
```

**Solution:**
- Ensure waypoint names exist in waypoints.yaml
- Check parameter format: `waypoint_sequence:="['wp1','wp2','wp3']"`
- Verify GPS navigator is running before launching mission
- Check `continue_on_failure` parameter setting

---

## Complete Field Test Script

Save this as `field_test.sh`:

```bash
#!/bin/bash
set -e

echo "=== GPS Waypoint Navigation Field Test ==="
cd ~/workspaces/rover
source install/setup.bash

echo ""
echo "1. Checking GPS fix..."
timeout 5 ros2 topic echo /gps/fix --once | grep -E "(latitude|longitude|status)" || {
    echo "❌ No GPS fix! Check GPS hardware."
    exit 1
}
echo "✓ GPS fix available"

echo ""
echo "2. Checking localization..."
timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | head -3 || {
    echo "❌ No map->base_link transform! Launch localization first."
    exit 1
}
echo "✓ Localization running"

echo ""
echo "3. Checking Nav2..."
ros2 node list | grep -q "controller_server" || {
    echo "❌ Nav2 not running! Launch nav2_bringup.launch.py first."
    exit 1
}
echo "✓ Nav2 active"

echo ""
echo "4. Checking GPS waypoint navigator..."
timeout 3 ros2 action list | grep -q "navigate_to_gps" || {
    echo "❌ GPS navigator not running! Launch gps_navigator.launch.py first."
    exit 1
}
echo "✓ GPS navigator active"

echo ""
echo "5. Recording current position as 'field_test_start'..."
ros2 run gps_waypoint_navigator record_waypoint --name field_test_start

echo ""
echo "=== ✓ System Ready for Field Test! ==="
echo ""
echo "Next steps:"
echo "1. Drive rover 10-20m away from current position"
echo "2. Run: ros2 run gps_waypoint_navigator record_waypoint --name field_test_target"
echo "3. Drive back to start position manually"
echo "4. Run: ros2 action send_goal /go_to_named_waypoint urc_msgs/action/GoToNamedWaypoint '{waypoint_name: \"field_test_target\", max_retries: 2}' --feedback"
echo "5. Watch rover navigate autonomously to target!"
echo ""
echo "To test multi-waypoint mission:"
echo "  ros2 launch mission_bt mission_bt.launch.py waypoint_sequence:=\"['field_test_start','field_test_target']\""
echo ""
```

Make it executable:
```bash
chmod +x field_test.sh
```

Run it:
```bash
./field_test.sh
```

---

## Data Collection During Test

### Record Full Test Session

```bash
# Record all topics (large file!)
ros2 bag record -a -o gps_nav_test_$(date +%Y%m%d_%H%M%S)

# Or record specific topics (recommended):
ros2 bag record \
  /gps/fix \
  /tf /tf_static \
  /cmd_vel \
  /drive/left_rpm /drive/right_rpm \
  /gps_navigator/status \
  /navigate_to_pose/_action/feedback \
  /navigate_to_gps/_action/feedback \
  -o gps_nav_test_$(date +%Y%m%d_%H%M%S)
```

### Playback Recorded Data

```bash
# Play back test data
ros2 bag play gps_nav_test_20250121_143022

# Play at 0.5x speed
ros2 bag play gps_nav_test_20250121_143022 --rate 0.5

# Play specific topics
ros2 bag play gps_nav_test_20250121_143022 --topics /gps/fix /cmd_vel
```

---

## Performance Benchmarks

### Expected Performance

| Metric | Expected Value | Acceptable Range |
|--------|---------------|------------------|
| GPS Fix Time | < 60 seconds | 30-90 seconds |
| Position Accuracy | 2-5 meters | < 10 meters |
| Navigation Success Rate | > 90% | > 80% |
| Goal Tolerance | 1-2 meters | < 3 meters |
| Max Navigation Distance | 500 meters | unlimited |
| Retry Success Rate | > 50% | > 30% |

### Timing Expectations

| Phase | Expected Duration |
|-------|------------------|
| GPS cold start | 30-60 seconds |
| System initialization | 10-20 seconds |
| Single waypoint navigation (20m) | 30-90 seconds |
| Multi-waypoint mission (4 waypoints) | 3-10 minutes |

---

## Success Criteria

Your GPS waypoint navigation system is working correctly if:

✅ **Waypoint Recording**
- Current GPS position is saved correctly
- Waypoints.yaml file updates
- Waypoint names can be looked up

✅ **Single Waypoint Navigation**
- Rover accepts waypoint goal
- Converts GPS coordinates to map frame
- Plans path using Nav2
- Drives autonomously to within 1-3m of target
- Reports SUCCESS

✅ **Multi-Waypoint Mission**
- Mission executor starts without errors
- Navigates through all waypoints in sequence
- Handles failures gracefully (skip or abort based on config)
- Reports completion status

✅ **Action Server Interface**
- All three action servers respond (/navigate_to_gps, /go_to_named_waypoint, /follow_gps_waypoints)
- Feedback is published during navigation
- Goals can be cancelled
- Retry logic works

✅ **Integration**
- Works with existing Nav2 stack
- Works with ZED localization
- Works with drive_control
- No crashes or freezes

---

## Launch Order Reference

Always launch in this order:

1. **Hardware drivers** (GPS, ZED, motor controllers)
2. **Localization** (`loc_fusion.launch.py`)
3. **Navigation** (`nav2_bringup.launch.py`)
4. **GPS Navigator** (`gps_navigator.launch.py`)
5. **Mission Executor** (`mission_bt.launch.py`) [optional]

---

## Quick Reference Commands

### System Status Check
```bash
# One-liner to check all systems
ros2 node list && ros2 action list && ros2 topic hz /gps/fix -c 1
```

### Record Test Waypoint
```bash
ros2 run gps_waypoint_navigator record_waypoint --name test_wp
```

### Navigate to Waypoint
```bash
ros2 action send_goal /go_to_named_waypoint urc_msgs/action/GoToNamedWaypoint \
  "{waypoint_name: 'test_wp', max_retries: 2}" --feedback
```

### Run Mission
```bash
ros2 launch mission_bt mission_bt.launch.py \
  waypoint_sequence:="['wp1','wp2','wp3']"
```

### Emergency Stop
```bash
ros2 action cancel_goal /go_to_named_waypoint
```

---

## Competition Day Checklist

### Pre-Competition (Day Before)
- ☐ Test GPS fix acquisition time at competition site
- ☐ Record all competition waypoints
- ☐ Test navigation to each waypoint individually
- ☐ Run full mission sequence
- ☐ Verify retry logic works
- ☐ Test emergency stop procedures
- ☐ Backup waypoints.yaml file

### Competition Day Morning
- ☐ Power on rover, wait for GPS fix
- ☐ Launch localization, verify TF tree
- ☐ Launch Nav2, verify all nodes active
- ☐ Launch GPS navigator
- ☐ Test single waypoint navigation
- ☐ Load mission sequence
- ☐ Ready for autonomous run!

### During Autonomous Run
- ☐ Monitor GPS navigator status
- ☐ Monitor Nav2 feedback
- ☐ Watch for obstacles
- ☐ Be ready to emergency stop
- ☐ Record bag file for analysis

### Post-Run Analysis
- ☐ Review bag file
- ☐ Check navigation accuracy
- ☐ Analyze failures (if any)
- ☐ Note improvements for next run

---

## Advanced Testing

### Test GPS Accuracy
```bash
# Record position for 5 minutes
timeout 300 ros2 topic echo /gps/fix > gps_static_test.txt

# Analyze position drift
cat gps_static_test.txt | grep -E "latitude|longitude" | \
  awk '{print $2}' | sort | uniq -c
```

### Test Localization Drift
```bash
# Record TF for 10 minutes while stationary
ros2 bag record /tf /tf_static -o localization_drift_test -d 600

# Analyze with:
ros2 bag play localization_drift_test
# Monitor map->base_link transform drift
```

### Stress Test (Rapid Waypoint Changes)
```bash
# Send goals rapidly
for wp in wp1 wp2 wp3 wp4; do
  ros2 action send_goal /go_to_named_waypoint urc_msgs/action/GoToNamedWaypoint \
    "{waypoint_name: '$wp', max_retries: 1}" &
  sleep 2
done
```

---

## Appendix: Simulation Testing (Without Hardware)

If you want to test the logic without GPS/ZED:

### 1. Fake GPS Publisher
```bash
ros2 topic pub /gps/fix sensor_msgs/msg/NavSatFix \
  "{header: {frame_id: 'gps'}, status: {status: 0}, \
    latitude: 38.4063, longitude: -110.7918, altitude: 1500.0}" --rate 1
```

### 2. Fake TF Tree
```bash
# Publish static map->odom->base_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &
```

### 3. Launch Without Hardware Dependencies
```bash
ros2 launch gps_waypoint_navigator gps_navigator.launch.py
# Will work with fake GPS/TF above
```

---

## Support and Debugging

If you encounter issues not covered here:

1. Check ROS logs: `ros2 run rqt_console rqt_console`
2. Review this guide's troubleshooting section
3. Record a bag file showing the issue
4. Check GPS waypoint navigator README: `../gps_waypoint_navigator/README.md`
5. Review Nav2 documentation for navigation issues

---

**Good luck with your field tests!** 🚀🤖

Remember: **Safety first!** Always be ready to hit the emergency stop button during autonomous navigation testing.

