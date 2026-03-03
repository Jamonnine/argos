#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash
export DISPLAY=:0

echo "Starting SLAM Toolbox + RViz2..."
nohup ros2 launch my_robot_bringup slam_sim.launch.py \
  > /tmp/slam.log 2>&1 &
SLAM_PID=$!
disown
echo "SLAM PID: $SLAM_PID"
sleep 3
echo "=== First lines of SLAM log ==="
head -10 /tmp/slam.log 2>/dev/null
