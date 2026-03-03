#!/bin/bash
source /opt/ros/jazzy/setup.bash
export DISPLAY=:0

echo "Starting ros_gz_bridge..."
nohup ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=/opt/ros/jazzy/share/turtlebot3_gazebo/params/turtlebot3_burger_bridge.yaml \
  > /tmp/bridge.log 2>&1 &
echo "Bridge PID: $!"
disown

sleep 5
echo "=== Checking odom TF ==="
timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -5
echo "=== Bridge log ==="
cat /tmp/bridge.log | head -10
