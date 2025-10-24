#!/bin/bash
# Continuous RTK monitoring - shows live GPS status
# Run on Jetson with rover GPS running

source /opt/ros/humble/setup.bash 2>/dev/null

echo "🛰️  Live RTK Monitor"
echo "Press Ctrl+C to stop"
echo ""
echo "="*70
echo ""

while true; do
    # Clear previous line
    tput cuu1 2>/dev/null
    tput el 2>/dev/null
    
    # Get fix status
    FIX_STATUS=$(timeout 1 ros2 topic echo /gps/fix --once 2>/dev/null | grep "status:" | head -1 | awk '{print $2}')
    
    # Get position
    LAT=$(timeout 1 ros2 topic echo /gps/fix --once 2>/dev/null | grep "latitude:" | awk '{print $2}')
    LON=$(timeout 1 ros2 topic echo /gps/fix --once 2>/dev/null | grep "longitude:" | awk '{print $2}')
    ALT=$(timeout 1 ros2 topic echo /gps/fix --once 2>/dev/null | grep "altitude:" | awk '{print $2}')
    
    # Get satellite count
    SAT=$(timeout 1 ros2 topic echo /gnss/satellite_count --once 2>/dev/null | grep "data:" | awk '{print $2}')
    
    # Format output
    TIMESTAMP=$(date "+%H:%M:%S")
    
    case $FIX_STATUS in
        2)
            STATUS="✅ RTK FIX"
            ;;
        1)
            STATUS="🟡 GPS FIX"
            ;;
        *)
            STATUS="❌ NO FIX"
            ;;
    esac
    
    printf "[%s] %s | Sats: %2s | Lat: %10.6f | Lon: %11.6f | Alt: %6.1fm\n" \
           "$TIMESTAMP" "$STATUS" "${SAT:-??}" "${LAT:-0}" "${LON:-0}" "${ALT:-0}"
    
    sleep 1
done

