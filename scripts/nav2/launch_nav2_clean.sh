#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash
export DISPLAY=:0

echo "Starting Nav2 + RViz2..."
nohup ros2 launch my_robot_bringup nav2_sim.launch.py \
  > /tmp/nav2_final.log 2>&1 &
NAV2_PID=$!
disown
echo "Nav2 PID: $NAV2_PID"
sleep 3
echo "=== First 10 lines of nav2 log ==="
head -10 /tmp/nav2_final.log 2>/dev/null
