# Switching from Simulation to Real Hardware

## Current Status: ✅ SIMULATION READY

Your MoveIt2 configuration is **fully tested in simulation** with:
- ✅ 5-DOF arm (AB_Rev, AS1_Rev, AS2_Rev, AW_Rev, AM_Rev)
- ✅ Continuous base rotation
- ✅ True 3D spherical shoulder (orthogonal AS1/AS2 axes)
- ✅ Vertical arm orientation
- ✅ Complete controller stack (ros2_control + MoveIt)

## Hardware Interface Status: ✅ UPDATED

The hardware interface (`hardware/rover_arm_hardware.py`) has been updated to support all 5 joints and will communicate with your motor controllers via:

**Published Topics:**
- `/arm_target_motor_positions` (Int32MultiArray) → Motor commands in encoder ticks

**Subscribed Topics:**
- `/get_arm_position` (Int32MultiArray) → Encoder feedback in ticks

**Conversions:**
- Radians ↔ Motor ticks using gear ratios
- Joint limits enforced by MoveIt
- Same trajectories work in sim and hardware

## To Switch to Real Hardware:

### Step 1: Physical Connections
1. Connect arm motors to motor controllers
2. Ensure encoder feedback is publishing to `/get_arm_position`
3. Verify motor controllers subscribe to `/arm_target_motor_positions`

### Step 2: Update URDF Hardware Plugin

Edit `urdf/rover_arm.urdf.xacro` line ~271:

```xml
<ros2_control name="RoverArmSystem" type="system">
  <hardware>
    <!-- COMMENT OUT mock hardware -->
    <!-- <plugin>mock_components/GenericSystem</plugin>
    <param name="mock_sensor_commands">true</param> -->
    
    <!-- UNCOMMENT real hardware -->
    <plugin>rover_arm_hardware/RoverArmHardware</plugin>
  </hardware>
```

### Step 3: Rebuild and Launch

```bash
cd /home/rover/workspaces/rover
source install/setup.bash
colcon build --packages-select moveit_config_rover
ros2 launch moveit_config_rover arm_moveit_sim.launch.py  # Same launch file!
```

The launch file name stays the same - it auto-detects which hardware plugin is active.

### Step 4: Verify Hardware Communication

```bash
# Check motor feedback is coming in
ros2 topic echo /get_arm_position

# Check commands are being sent
ros2 topic echo /arm_target_motor_positions

# Check joint states
ros2 topic echo /joint_states
```

### Step 5: Test Motion (Start Small!)

```python
# Use the SAME test script - works on both sim and hardware
python3 src/moveit/moveit_config_rover/scripts/test_arm_control.py
```

**Or test manually in RViz:**
1. Open Motion Planning plugin
2. Select a named pose (e.g., "home")
3. Click "Plan" then "Execute"
4. Watch the real arm move!

## Safety Notes:

⚠️ **Before first hardware test:**
1. Ensure **emergency stop** is accessible
2. Start with **small joint movements** (±0.1 rad)
3. Verify **joint limits** match physical constraints
4. Check **motor directions** are correct (may need to invert in firmware)
5. Test **one joint at a time** first

⚠️ **If motors move wrong direction:**
- Don't change URDF/MoveIt config
- Fix in motor controller firmware or invert encoder polarity

## What Works Identically in Sim and Hardware:

✅ **MoveIt planning** - Same trajectories
✅ **Named poses** - All SRDF poses work
✅ **Python control scripts** - `moveit_arm_controller.py`, `test_arm_control.py`
✅ **RViz visualization** - Same interface
✅ **Joint limits** - Same safety bounds
✅ **Controller parameters** - Same PID gains (adjust if needed)

## Gear Ratios (Verify Against Your Hardware):

```python
GEAR_RATIOS = {
    'AB_Rev': 50,         # Base rotation
    'AS1_Rev': 100,       # Shoulder pitch
    'AS2_Rev': 100,       # Shoulder yaw (NEW - verify this!)
    'AW_Rev': 50,         # Elbow pitch
    'AM_Rev': 122.22,     # Wrist roll (50 * 22/9)
}
```

**⚠️ AS2_Rev is new** - verify its gear ratio matches your physical hardware!

## Troubleshooting Hardware Issues:

**Motors don't move:**
- Check `/arm_target_motor_positions` is publishing
- Verify motor controllers are subscribed
- Check power to motors

**Motors move wrong direction:**
- Invert motor polarity in firmware
- OR: Invert encoder readings in `rover_arm_hardware.py`

**Arm drifts/doesn't hold position:**
- Tune PID gains in `config/rover_arm_controllers.yaml`
- Check encoder feedback quality

**Joint limits violated:**
- Hardware limits in `config/joint_limits.yaml`
- Physical hard stops on robot

**Trajectories jerky:**
- Reduce `max_velocity` and `max_acceleration` in `joint_limits.yaml`
- Increase controller `update_rate` if CPU allows

## Next Steps After Hardware Works:

1. **Tune PID gains** for smooth motion
2. **Calibrate zero positions** if needed
3. **Test all 10 challenging poses** from test script
4. **Add gripper control** (if applicable)
5. **Integrate with rover navigation** stack

---

**Your simulation is production-ready.** The same code, configs, and trajectories will work on real hardware with just the hardware plugin switch! 🎉

