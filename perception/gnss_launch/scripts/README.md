# GPS Scripts

Utility scripts for ZED-F9P GPS setup and monitoring.

## Base Station Scripts (Run on Mac/Laptop)

### configure_base_station.py
Initial setup of base station - run once per location.

```bash
# Install dependencies
pip3 install pyserial

# Configure base
python3 configure_base_station.py /dev/tty.usbmodem14201

# Takes 5-10 minutes to complete survey
```

### base_verify.py
Verify base station is configured correctly.

```bash
python3 base_verify.py /dev/tty.usbmodem14201
```

**Checks:**
- TMODE3 mode (should be FIXED)
- Saved position coordinates
- UART2 configuration for radio
- RTCM message output enabled

---

## Rover Scripts (Run on Jetson)

### rover_check_corrections.sh
Quick check if rover is receiving RTK corrections.

```bash
./rover_check_corrections.sh
```

**Shows:**
- GPS fix status (No Fix / GPS / RTK)
- Position coordinates
- Satellite count
- Health status

### monitor_rtk.sh
Live monitoring of RTK status (updates every second).

```bash
./monitor_rtk.sh
```

**Output:**
```
[14:23:45] ✅ RTK FIX | Sats: 16 | Lat: 42.123456 | Lon: -71.234567 | Alt: 45.2m
[14:23:46] ✅ RTK FIX | Sats: 17 | Lat: 42.123457 | Lon: -71.234568 | Alt: 45.3m
```

### test_radio_link.sh
Test radio connection between base and rover.

```bash
./test_radio_link.sh
```

**Checks:**
- Radio device present
- Data flowing through radio
- GPS receiving corrections
- RTK fix status

---

## Workflow

### One-Time Setup (At Home)

```bash
# 1. Configure base station
python3 ../configure_base_station.py /dev/tty.usbmodem14201

# 2. Verify configuration
python3 base_verify.py /dev/tty.usbmodem14201

# 3. Done! Base is ready for field use
```

### Field Deployment

**Base Station:**
1. Power on base + radio
2. Wait 30 seconds for startup
3. Radio LED should blink (transmitting)

**Rover:**
```bash
# 1. Launch GPS
cd ~/workspaces/rover/src/perception/gnss_launch
./launch_gps.sh

# 2. Test radio link
cd scripts
./test_radio_link.sh

# 3. Check corrections
./rover_check_corrections.sh

# 4. Monitor live (optional)
./monitor_rtk.sh
```

---

## Troubleshooting

**No RTK Fix:**
- Wait 1-5 minutes after base powers on
- Check radio LEDs are active
- Verify radios are paired/same frequency
- Ensure clear line of sight
- Check radio connected to F9P UART2

**No GPS Fix:**
- Go outside with clear sky view
- Wait 30-60 seconds for satellite lock
- Check antenna connection
- Verify antenna has good ground plane

**Radio Not Working:**
- Check device exists: `ls /dev/ttyUSB*`
- Verify baud rate matches (57600)
- Check wiring: TX→RX, RX→TX, GND→GND
- Test with loopback (connect TX to RX)

---

## Quick Reference

| Status | Meaning | Accuracy |
|--------|---------|----------|
| No Fix | No satellites | N/A |
| GPS Fix (status: 1) | Standard GPS | 1-5 meters |
| RTK Fix (status: 2) | With corrections | 1-2 cm |

**RTCM Message Types:**
- 1005: Base antenna position
- 1077: GPS corrections (MSM7)
- 1087: GLONASS corrections
- 1097: Galileo corrections
- 1127: BeiDou corrections

**Radio Connection:**
```
F9P UART2 TX → Radio RX
F9P UART2 RX → Radio TX (optional)
F9P GND      → Radio GND
```

