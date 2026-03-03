#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash

echo "=== slam_toolbox process ==="
ps aux | grep -E "slam_toolbox|async_slam" | grep -v grep

echo ""
echo "=== ROS2 nodes ==="
timeout 3 ros2 node list 2>&1 | grep -E "slam|rviz"

echo ""
echo "=== slam_toolbox node info ==="
timeout 3 ros2 node info /slam_toolbox 2>&1 | head -20

echo ""
echo "=== Full SLAM log (slam_toolbox lines only) ==="
cat /tmp/slam.log 2>/dev/null | grep -v rviz2 | tail -20
