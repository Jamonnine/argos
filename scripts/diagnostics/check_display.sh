#!/bin/bash
echo "=== DISPLAY env ==="
echo "DISPLAY=$DISPLAY"
echo ""
echo "=== X lock files ==="
ls /tmp/.X*-lock 2>/dev/null || echo "none"
echo ""
echo "=== VNC/web ports ==="
ss -tlnp 2>/dev/null | grep -E "590|608|5900" | head -5 || echo "none"
echo ""
echo "=== RViz2 process check ==="
ps aux | grep -E "rviz2" | grep -v grep | head -5
echo ""
echo "=== DISPLAY from rviz2 env ==="
cat /proc/$(ps aux | grep rviz2 | grep -v grep | awk '{print $2}' | head -1)/environ 2>/dev/null | tr '\0' '\n' | grep DISPLAY
