#!/bin/bash
echo "=== slam_toolbox ==="
dpkg -l ros-jazzy-slam-toolbox 2>/dev/null | grep "^ii" | awk '{print $1, $2, $3}' || echo "NOT installed"

echo ""
echo "=== teleop_twist_keyboard ==="
dpkg -l ros-jazzy-teleop-twist-keyboard 2>/dev/null | grep "^ii" | awk '{print $1, $2, $3}' || echo "NOT installed"

echo ""
echo "=== turtlebot3_teleop ==="
dpkg -l ros-jazzy-turtlebot3-teleop 2>/dev/null | grep "^ii" | awk '{print $1, $2, $3}' || echo "NOT installed"
