#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash
echo "=== Nav2 Lifecycle Status ==="
grep -a -E "Activated|lifecycle_manager.*nav|lifecycle_manager.*local|Failed|ERROR" /tmp/nav2_final.log 2>/dev/null | tail -20
echo ""
echo "=== bt_navigator use_sim_time ==="
ros2 param get /bt_navigator use_sim_time 2>&1
echo "=== rviz2 use_sim_time ==="
ros2 param get /rviz2 use_sim_time 2>&1
echo "=== Current sim time ==="
ros2 topic echo /clock --once 2>&1 | head -3
