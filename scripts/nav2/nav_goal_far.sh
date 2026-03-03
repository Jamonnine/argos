#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash

echo "=== Current robot position ==="
timeout 3 ros2 topic echo /amcl_pose --once 2>&1 | grep -A 4 "position:"

echo ""
echo "=== Sending goal to (-1.0, -1.0) - across the map ==="
timeout 90 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{ pose: { header: { frame_id: 'map' }, pose: { position: { x: -1.0, y: -1.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.707, w: 0.707 } } } }" \
  --feedback 2>&1 | grep -E "position:|distance_remaining:|status:|SUCCEEDED|FAILED|error_code|x:|y:" | head -60
