#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash
export DISPLAY=:0
export TURTLEBOT3_MODEL=burger

TB3_NAV2_DIR=$(ros2 pkg prefix turtlebot3_navigation2 2>/dev/null)/share/turtlebot3_navigation2
RVIZ_CONFIG="${TB3_NAV2_DIR}/rviz/tb3_navigation2.rviz"

echo "Starting RViz2 with config: $RVIZ_CONFIG"
nohup rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=True > /tmp/rviz2.log 2>&1 &
echo "RViz2 PID: $!"
disown
sleep 2
echo "RViz2 started. Check /tmp/rviz2.log for status."
