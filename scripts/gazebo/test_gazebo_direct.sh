#!/bin/bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
export DISPLAY=:0

echo "=== Testing Gazebo launch directly (5 second timeout) ==="
echo "Any errors will show here:"
timeout 8 ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 2>&1 | head -30
echo "=== End of test ==="
