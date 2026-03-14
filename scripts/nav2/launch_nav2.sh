#!/bin/bash
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
echo "Launching nav2_sim.launch.py..."
nohup ros2 launch argos_bringup nav2_sim.launch.py > /tmp/nav2_final.log 2>&1 &
echo "PID: $!"
disown
sleep 12
echo "=== Map server status ==="
grep -a -E "map_server|Loading yaml|Failed|Activated|Configuring" /tmp/nav2_final.log 2>/dev/null | head -20
