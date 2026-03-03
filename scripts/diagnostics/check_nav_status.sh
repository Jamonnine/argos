#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash

echo "=== Current AMCL pose (map frame) ==="
timeout 5 ros2 topic echo /amcl_pose --once 2>&1 | grep -A 5 "position:" | head -7

echo ""
echo "=== Sim time ==="
timeout 3 ros2 topic echo /clock --once 2>&1 | head -3

echo ""
echo "=== Live navigation test: goal to (-2.0, -0.5) [spawn point] ==="
echo "(30 second timeout - watch distance_remaining decrease!)"
timeout 45 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{ pose: { header: { frame_id: 'map' }, pose: { position: { x: -2.0, y: -0.5, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } } }" \
  --feedback 2>&1 | grep -E "x: |y: |distance_remaining:|SUCCEEDED|FAILED|error_code: [^0]" | head -50

echo ""
echo "=== Final result ==="
timeout 3 ros2 topic echo /amcl_pose --once 2>&1 | grep -A 4 "position:" | head -6
