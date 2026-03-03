#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash

echo "=== SLAM-related topics ==="
timeout 3 ros2 topic list 2>&1 | grep -E "map|slam" | head -10

echo ""
echo "=== /map topic info ==="
timeout 3 ros2 topic info /map 2>&1 | head -5

echo ""
echo "=== SLAM log tail ==="
cat /tmp/slam.log 2>/dev/null | tail -10

echo ""
echo "=== TF: map -> odom (SLAM provides this!) ==="
timeout 3 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -E "Translation|Rotation: in RPY" | head -4
