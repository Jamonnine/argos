#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/USER/Desktop/ARGOS/ros2_ws/install/setup.bash

echo "=== Current sim time ==="
timeout 3 ros2 topic echo /clock --once 2>&1 | head -3

echo ""
echo "=== Checking navigate_to_pose action ==="
ros2 action list 2>/dev/null | grep navigate

echo ""
echo "=== Sending Nav2 Goal: (1.0, 0.0, 0 degrees) ==="
echo "TurtleBot3 world spawn: (-2.0, -0.5), goal: (1.0, 0.0)"
timeout 20 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{ pose: { header: { frame_id: 'map' }, pose: { position: { x: 1.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } } }" \
  --feedback 2>&1 | head -40
