#!/bin/bash
echo "=== Gazebo log ==="
cat /tmp/gazebo.log 2>/dev/null | tail -20 || echo "No gazebo log found"

echo ""
echo "=== PID 26571 status ==="
cat /proc/26571/status 2>/dev/null | head -5 || echo "PID 26571 not running"

echo ""
echo "=== All gz processes ==="
ps aux | grep gz | grep -v grep
