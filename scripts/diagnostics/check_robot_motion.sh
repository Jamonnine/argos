#!/bin/bash
source /opt/ros/jazzy/setup.bash

echo "=== Sim time check ==="
timeout 3 ros2 topic echo /clock --once 2>&1 | head -3

echo ""
echo "=== cmd_vel (is Nav2 sending velocity commands?) ==="
echo "Watching for 3 seconds..."
timeout 3 ros2 topic echo /cmd_vel 2>&1 | head -10

echo ""
echo "=== /cmd_vel topic Hz ==="
timeout 3 ros2 topic hz /cmd_vel 2>&1

echo ""
echo "=== Gazebo process running? ==="
ps aux | grep gz | grep -v grep | awk '{print $1, $2, $3, $11}' | head -3

echo ""
echo "=== TF: odom -> base_footprint ==="
timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -8
