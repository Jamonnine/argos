#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash

echo "=== Final lifecycle states ==="
ros2 lifecycle get /map_server 2>/dev/null
ros2 lifecycle get /amcl 2>/dev/null
ros2 lifecycle get /controller_server 2>/dev/null
ros2 lifecycle get /planner_server 2>/dev/null
ros2 lifecycle get /bt_navigator 2>/dev/null

echo ""
echo "=== Log summary ==="
grep -a -E "Managed nodes|lifecycle_manager_nav|bond|all.*active|active" /tmp/nav2_final.log 2>/dev/null | tail -10

echo ""
echo "=== Sim time ==="
ros2 topic echo /clock --once 2>&1 | head -3
