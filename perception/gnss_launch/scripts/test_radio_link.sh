#!/bin/bash
# Test radio link between base and rover
# Run this AFTER starting GPS on rover

echo "📻 Testing Radio Link"
echo ""
echo "This test checks if the rover is receiving data from base station"
echo ""

# Check if we can see the radio device
echo "🔍 Checking for radio connection..."

# Common radio serial devices
RADIO_DEVICES="/dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyAMA0 /dev/serial0"

FOUND=0
for DEV in $RADIO_DEVICES; do
    if [ -e "$DEV" ]; then
        echo "   Found: $DEV"
        FOUND=1
        
        # Try to read some data
        echo "   Testing data flow..."
        timeout 2 cat $DEV 2>/dev/null | head -c 100 > /tmp/radio_test.bin
        
        SIZE=$(stat -c%s /tmp/radio_test.bin 2>/dev/null)
        if [ "$SIZE" -gt "0" ]; then
            echo "   ✅ Receiving data ($SIZE bytes in 2 seconds)"
            echo "   Radio link appears to be working!"
        else
            echo "   ⚠️  No data received"
            echo "   Check base station is powered on and transmitting"
        fi
        echo ""
    fi
done

if [ $FOUND -eq 0 ]; then
    echo "   ❌ No radio device found"
    echo ""
    echo "   Expected devices: /dev/ttyUSB* or /dev/ttyAMA*"
    echo "   Check radio is connected to Jetson"
    echo ""
fi

# Now check GPS for RTK status
echo "📡 Checking GPS RTK Status..."
echo ""

source /opt/ros/humble/setup.bash 2>/dev/null

if ros2 node list 2>/dev/null | grep -q gnss_driver; then
    FIX_STATUS=$(timeout 2 ros2 topic echo /gps/fix --once 2>/dev/null | grep "status:" | head -1 | awk '{print $2}')
    
    case $FIX_STATUS in
        2)
            echo "   ✅ RTK FIX - Corrections being received!"
            ;;
        1)
            echo "   🟡 GPS Fix - No RTK corrections yet"
            echo "   Radio may not be connected to F9P UART2"
            ;;
        *)
            echo "   ❌ No GPS fix"
            ;;
    esac
else
    echo "   ⚠️  GPS driver not running"
fi

echo ""
echo "="*70
echo "📋 Summary"
echo "="*70
echo ""
echo "For RTK to work, you need:"
echo "  1. Base station powered on and transmitting"
echo "  2. Both radios paired and on same frequency"
echo "  3. Radio connected to F9P UART2 (RX/TX/GND)"
echo "  4. Radio receiving corrections (check LEDs)"
echo "  5. Clear line of sight between base and rover"
echo ""
echo "Typical RTK lock time: 30 seconds to 5 minutes"
echo ""

