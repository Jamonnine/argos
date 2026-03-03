#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash
echo "=== Navigation Full Activation Status ==="
grep -a -E "planner|bt_navigator.*bond|Managed nodes|navigation.*active|navigation.*Failed" /tmp/nav2_final.log 2>/dev/null | tail -15
echo ""
echo "=== Active ROS2 nodes ==="
ros2 lifecycle list 2>/dev/null | grep -E 'active|inactive|unconfigured' | head -10
echo ""
echo "=== Navigation action server ==="
ros2 action list 2>/dev/null | grep navigate
