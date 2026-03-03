#!/bin/bash
echo "=== Killing all Nav2, Gazebo, RViz2, Bridge processes ==="

for pid in $(ps aux | grep -E 'nav2|bt_navigator|planner_server|controller_server|lifecycle_manager|amcl|map_server|ros_gz_bridge|parameter_bridge|rviz2|gzserver|gz sim|turtlebot3_gazebo|robot_state_publisher|ros2 launch' | grep -v grep | awk '{print $2}'); do
    echo "Killing PID $pid: $(cat /proc/$pid/cmdline 2>/dev/null | tr '\0' ' ' | cut -c1-60)"
    kill -9 "$pid" 2>/dev/null
done

sleep 2
echo ""
echo "=== Remaining ROS/Gazebo processes ==="
ps aux | grep -E 'nav2|gz|rviz2|ros2' | grep -v grep | head -5 || echo "All cleared!"
