#!/bin/bash
echo "=== Display environment ==="
echo "DISPLAY=$DISPLAY"
echo "WAYLAND_DISPLAY=$WAYLAND_DISPLAY"
echo "XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"
echo ""
echo "=== Wayland socket ==="
ls -la $XDG_RUNTIME_DIR/wayland* 2>/dev/null || echo "no wayland socket"
echo ""
echo "=== X11 socket ==="
ls -la /tmp/.X11-unix/ 2>/dev/null || echo "no X11 socket"
echo ""
echo "=== Try Gazebo server-only (headless) ==="
timeout 5 gz sim --help 2>&1 | head -5
