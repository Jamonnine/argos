#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash

echo "=== Current slam_toolbox lifecycle state ==="
ros2 lifecycle get /slam_toolbox 2>&1

echo ""
echo "=== Configuring slam_toolbox ==="
ros2 lifecycle set /slam_toolbox configure 2>&1
sleep 2

echo ""
echo "=== Activating slam_toolbox ==="
ros2 lifecycle set /slam_toolbox activate 2>&1
sleep 2

echo ""
echo "=== Final state ==="
ros2 lifecycle get /slam_toolbox 2>&1

echo ""
echo "=== /map publisher check ==="
timeout 3 ros2 topic info /map 2>&1 | head -5
