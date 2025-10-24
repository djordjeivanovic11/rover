#!/bin/bash
# Check if rover is receiving RTK corrections
# Run on Jetson with rover GPS running

echo "🔍 Checking RTK Corrections on Rover..."
echo ""

# Source ROS
source /opt/ros/humble/setup.bash 2>/dev/null

# Check if GPS node is running
if ! ros2 node list 2>/dev/null | grep -q gnss_driver; then
    echo "❌ GPS driver not running!"
    echo ""
    echo "Start it first:"
    echo "  ~/workspaces/rover/src/perception/gnss_launch/launch_gps.sh"
    exit 1
fi

echo "✓ GPS driver running"
echo ""

# Get GPS fix status
echo "📡 GPS Fix Status:"
FIX_STATUS=$(timeout 2 ros2 topic echo /gps/fix --once 2>/dev/null | grep "status:" | head -1 | awk '{print $2}')

if [ -z "$FIX_STATUS" ]; then
    echo "   ❌ No GPS data received"
    echo "   Check antenna connection and sky view"
    exit 1
fi

case $FIX_STATUS in
    -1)
        echo "   ❌ No Fix"
        ;;
    0)
        echo "   ⚠️  No Fix"
        ;;
    1)
        echo "   🟡 Standard GPS Fix (meter accuracy)"
        echo "   Not receiving RTK corrections yet"
        ;;
    2)
        echo "   ✅ RTK FIX (centimeter accuracy!)"
        echo "   Corrections received and applied!"
        ;;
    *)
        echo "   Status: $FIX_STATUS"
        ;;
esac

echo ""
echo "📊 GPS Details:"

# Get position
timeout 2 ros2 topic echo /gps/fix --once 2>/dev/null | grep -E "latitude:|longitude:|altitude:" | head -3

echo ""
echo "🛰️  Signal Quality:"

# Get satellite count if available
SAT_COUNT=$(timeout 2 ros2 topic echo /gnss/satellite_count --once 2>/dev/null | grep "data:" | awk '{print $2}')
if [ ! -z "$SAT_COUNT" ]; then
    echo "   Satellites: $SAT_COUNT"
else
    echo "   Satellites: N/A (health monitor may not be running)"
fi

# Get health status if available
HEALTH=$(timeout 2 ros2 topic echo /gnss/health_status --once 2>/dev/null | grep "data:" | awk '{print $2}' | tr -d "'\"")
if [ ! -z "$HEALTH" ]; then
    echo "   Health: $HEALTH"
fi

echo ""
echo "="*60
echo "🎯 RTK Status Check Complete"
echo "="*60
echo ""

if [ "$FIX_STATUS" = "2" ]; then
    echo "✅ RTK WORKING PERFECTLY!"
    echo "   You have cm-level accuracy"
    echo "   Base station corrections are being received"
elif [ "$FIX_STATUS" = "1" ]; then
    echo "⚠️  NOT RECEIVING RTK CORRECTIONS"
    echo ""
    echo "Troubleshooting:"
    echo "1. Is base station powered on?"
    echo "2. Is radio connected to base UART2?"
    echo "3. Is rover radio receiving data?"
    echo "4. Check radio LEDs for activity"
    echo "5. Verify radios are paired/same frequency"
    echo ""
    echo "Wait 1-5 minutes after base powers on for RTK to lock"
else
    echo "⚠️  NO GPS FIX"
    echo ""
    echo "Troubleshooting:"
    echo "1. Go outside with clear sky view"
    echo "2. Wait 30-60 seconds for satellite lock"
    echo "3. Check antenna is properly connected"
    echo "4. Verify antenna has good ground plane"
fi

echo ""

