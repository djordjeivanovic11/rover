#!/bin/bash
# Complete setup and test script for Nav2 integration

set -e  # Exit on error

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Nav2 Navigation Stack - Setup & Test                  ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════╝${NC}"
echo ""

# Step 1: Check Nav2 installation
echo -e "${YELLOW}[1/6] Checking Nav2 installation...${NC}"
if ! ros2 pkg list | grep -q "nav2_bringup"; then
    echo -e "${RED}✗ Nav2 not installed!${NC}"
    echo -e "${YELLOW}Installing Nav2...${NC}"
    sudo apt update
    sudo apt install -y ros-humble-navigation2
    echo -e "${GREEN}✓ Nav2 installed${NC}"
else
    echo -e "${GREEN}✓ Nav2 already installed${NC}"
fi
echo ""

# Step 2: Check serial devices
echo -e "${YELLOW}[2/6] Checking serial devices...${NC}"
echo "Connected serial devices:"
ls -l /dev/ttyACM* 2>/dev/null || echo "  No /dev/ttyACM* devices found"
echo ""
echo "Devices by ID:"
ls -l /dev/serial/by-id/ 2>/dev/null || echo "  No /dev/serial/by-id/ devices"
echo ""
if ls /dev/ttyACM* 1> /dev/null 2>&1; then
    echo -e "${GREEN}✓ Serial devices detected${NC}"
    echo -e "${BLUE}ℹ Make sure GPS (ttyACM0) and Teensy (ttyACM1) don't conflict!${NC}"
else
    echo -e "${YELLOW}⚠ No serial devices detected (Teensy may not be connected)${NC}"
fi
echo ""

# Step 3: Build navigation packages
echo -e "${YELLOW}[3/6] Building navigation packages...${NC}"
cd ~/workspaces/rover
colcon build --packages-select nav2_launch nav2_teensy_bridge --symlink-install
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Navigation packages built successfully${NC}"
else
    echo -e "${RED}✗ Build failed!${NC}"
    exit 1
fi
echo ""

# Step 4: Source workspace
echo -e "${YELLOW}[4/6] Sourcing workspace...${NC}"
source ~/workspaces/rover/install/setup.bash
echo -e "${GREEN}✓ Workspace sourced${NC}"
echo ""

# Step 5: Verify perception topics
echo -e "${YELLOW}[5/6] Checking perception stack status...${NC}"
echo "Required topics for Nav2:"
echo ""

# Function to check topic
check_topic() {
    local topic=$1
    local desc=$2
    if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        echo -e "  ${GREEN}✓${NC} $topic - $desc"
        return 0
    else
        echo -e "  ${RED}✗${NC} $topic - $desc ${YELLOW}(MISSING)${NC}"
        return 1
    fi
}

all_topics_ok=true
check_topic "/odometry/filtered" "Local odometry" || all_topics_ok=false
check_topic "/scan" "LaserScan for obstacles" || all_topics_ok=false
check_topic "/gps/fix" "GPS position" || all_topics_ok=false

echo ""
if [ "$all_topics_ok" = true ]; then
    echo -e "${GREEN}✓ All required topics available!${NC}"
else
    echo -e "${YELLOW}⚠ Some topics missing. Make sure perception stack is running:${NC}"
    echo -e "${BLUE}   ~/workspaces/rover/src/perception/run_full_stack.sh${NC}"
fi
echo ""

# Step 6: Navigation instructions
echo -e "${YELLOW}[6/6] Next Steps${NC}"
echo ""
echo -e "${BLUE}══════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}SETUP COMPLETE!${NC}"
echo -e "${BLUE}══════════════════════════════════════════════════════${NC}"
echo ""
echo "To launch navigation:"
echo ""
echo -e "${YELLOW}Option 1: Manual (2 terminals)${NC}"
echo "  Terminal 1:"
echo "    ~/workspaces/rover/src/perception/run_full_stack.sh"
echo ""
echo "  Terminal 2:"
echo "    source ~/workspaces/rover/install/setup.bash"
echo "    ros2 launch nav2_launch complete_nav.launch.py"
echo ""
echo -e "${YELLOW}Option 2: With visualization (3 terminals)${NC}"
echo "  Terminal 1: Perception (as above)"
echo "  Terminal 2: Navigation (as above)"
echo "  Terminal 3:"
echo "    rviz2"
echo "    # In RViz: Add displays for Map, LaserScan, Path"
echo "    # Use '2D Nav Goal' tool to send navigation goals"
echo ""
echo -e "${BLUE}══════════════════════════════════════════════════════${NC}"
echo ""
echo "Quick diagnostics:"
echo "  ros2 topic list | grep -E '(odom|scan|cmd_vel)'"
echo "  ros2 topic hz /odometry/filtered"
echo "  ros2 topic hz /scan"
echo "  ros2 node list | grep nav2"
echo ""
echo "Test navigation:"
echo "  ros2 topic pub /goal_pose geometry_msgs/PoseStamped \\"
echo "    '{header: {frame_id: \"map\"}, pose: {position: {x: 2.0, y: 0.0}}}' --once"
echo ""
echo -e "${GREEN}For more info: ~/workspaces/rover/src/navigation/README.md${NC}"
echo ""

