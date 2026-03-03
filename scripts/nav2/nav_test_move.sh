#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash

echo "=== Current AMCL pose (map frame) ==="
timeout 5 ros2 topic echo /amcl_pose --once 2>&1 | grep -A 5 "position:" | head -8

echo ""
echo "=== Current sim time ==="
timeout 3 ros2 topic echo /clock --once 2>&1 | head -3

echo ""
echo "=== Sending goal to (0.0, 0.0) - map center ==="
echo "Tracking position changes:"
timeout 45 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{ pose: { header: { frame_id: 'map' }, pose: { position: { x: 0.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } } }" \
  --feedback 2>&1 | awk '/current_pose:/,/distance_remaining:/' | grep -E "x:|y:|distance_remaining:" | head -30

echo ""
echo "=== Navigation result ==="
