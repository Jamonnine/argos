#!/bin/bash

##############################################################################
# ROS 2 Environment Check Script
# Quick diagnostic tool to verify ROS 2 installation and environment
##############################################################################

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "=========================================================================="
echo "                    ROS 2 Environment Check                              "
echo "=========================================================================="
echo ""

# Source ROS 2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo -e "${RED}✗ ROS 2 Jazzy not found at /opt/ros/jazzy${NC}"
    exit 1
fi

# 1. OS Version
echo -e "${BLUE}[1] Operating System${NC}"
lsb_release -d | sed 's/Description:\t/    /'
echo ""

# 2. ROS 2 Environment
echo -e "${BLUE}[2] ROS 2 Environment${NC}"
echo "    ROS_DISTRO: $ROS_DISTRO"
echo "    ROS_VERSION: $ROS_VERSION"
echo "    ROS_PYTHON_VERSION: $ROS_PYTHON_VERSION"
echo ""

# 3. ROS 2 Command
echo -e "${BLUE}[3] ROS 2 Command${NC}"
if command -v ros2 &> /dev/null; then
    echo -e "    ${GREEN}✓ ros2 command available${NC}"
    echo "    Location: $(which ros2)"
else
    echo -e "    ${RED}✗ ros2 command not found${NC}"
fi
echo ""

# 4. Python Version
echo -e "${BLUE}[4] Python${NC}"
python3 --version | sed 's/Python /    Python: /'
echo ""

# 5. Colcon Build Tool
echo -e "${BLUE}[5] Colcon Build Tool${NC}"
if command -v colcon &> /dev/null; then
    echo -e "    ${GREEN}✓ colcon available${NC}"
    colcon version | sed 's/^/    /'
else
    echo -e "    ${RED}✗ colcon not found${NC}"
fi
echo ""

# 6. Key Packages
echo -e "${BLUE}[6] Key ROS 2 Packages${NC}"
PACKAGES=("turtlesim" "demo_nodes_cpp" "demo_nodes_py" "rviz2" "nav2_bringup" "moveit" "ros_gz")
for pkg in "${PACKAGES[@]}"; do
    if ros2 pkg list 2>/dev/null | grep -q "^$pkg$"; then
        echo -e "    ${GREEN}✓${NC} $pkg"
    else
        echo -e "    ${YELLOW}○${NC} $pkg (not installed)"
    fi
done
echo ""

# 7. Workspace
echo -e "${BLUE}[7] ROS 2 Workspace${NC}"
if [ -d ~/ros2_ws ]; then
    echo -e "    ${GREEN}✓${NC} Workspace exists at ~/ros2_ws"
    if [ -d ~/ros2_ws/install ]; then
        echo -e "    ${GREEN}✓${NC} Workspace has been built"
    else
        echo -e "    ${YELLOW}○${NC} Workspace not yet built"
    fi
else
    echo -e "    ${YELLOW}○${NC} No workspace at ~/ros2_ws (will be created)"
fi
echo ""

# 8. DDS Implementation
echo -e "${BLUE}[8] DDS Middleware${NC}"
echo "    RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-default}"
echo ""

# 9. Quick Functionality Test
echo -e "${BLUE}[9] Quick Test${NC}"
echo "    Testing ros2 topic list..."
if timeout 2s ros2 topic list &> /dev/null; then
    echo -e "    ${GREEN}✓${NC} ROS 2 commands working"
else
    echo -e "    ${YELLOW}○${NC} ROS 2 daemon starting (normal on first run)"
fi
echo ""

echo "=========================================================================="
echo -e "${GREEN}Environment check complete!${NC}"
echo "=========================================================================="
echo ""
echo "Next steps:"
echo "  • Run a demo: ros2 run turtlesim turtlesim_node"
echo "  • Start learning: cd /mnt/c/Users/USER/Desktop/ARGOS/learning-log/week01"
echo ""
