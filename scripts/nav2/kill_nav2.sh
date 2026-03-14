#!/bin/bash
# Kill all Nav2-related processes
for f in /proc/*/cmdline; do
    pid=${f%/cmdline}
    pid=${pid#/proc/}
    [[ ! "$pid" =~ ^[0-9]+$ ]] && continue
    cmdline=$(cat "$f" 2>/dev/null | tr '\0' ' ')
    if echo "$cmdline" | grep -qE 'ros2 launch argos_bringup|nav2_map_server|nav2_amcl|rviz2/rviz2|opennav_docking|nav2_bt_navigator|nav2_planner|nav2_controller|lifecycle_manager|nav2_smoother|nav2_behaviors|waypoint_follower|velocity_smoother|collision_monitor|route_server|nav2_costmap'; then
        echo "Killing $pid: ${cmdline:0:70}"
        kill -9 "$pid" 2>/dev/null
    fi
done
sleep 2
echo "Done. Remaining:"
ps aux | grep -E 'nav2|rviz2|lifecycle_manager|bt_nav|amcl' | grep -v grep | wc -l
