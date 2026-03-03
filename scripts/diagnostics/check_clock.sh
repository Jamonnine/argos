#!/bin/bash
source /opt/ros/jazzy/setup.bash
echo "=== Gazebo clock ==="
timeout 5 ros2 topic echo /clock --once 2>&1 | head -4

echo ""
echo "=== TF odom ==="
timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -6

echo ""
echo "=== Topics from Gazebo ==="
timeout 3 ros2 topic list 2>&1 | grep -E "clock|scan|odom|cmd_vel|tf" | head -8
