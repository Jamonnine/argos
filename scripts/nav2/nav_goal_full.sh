#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash

echo "=== Sending Nav2 Goal to (1.0, 0.0) - waiting for completion ==="
timeout 60 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{ pose: { header: { frame_id: 'map' }, pose: { position: { x: 1.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } } }" \
  --feedback 2>&1

echo ""
echo "=== Final robot pose ==="
timeout 3 ros2 topic echo /odom --once 2>&1 | grep -A 5 "position:"
