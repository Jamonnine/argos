#!/bin/bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
export DISPLAY=:0

echo "Starting Gazebo with TurtleBot3 world..."
nohup ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py > /tmp/gazebo.log 2>&1 &
echo "Gazebo PID: $!"
disown
echo "Done. Check /tmp/gazebo.log for status."
