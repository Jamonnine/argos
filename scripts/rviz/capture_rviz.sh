#!/bin/bash
# WSLg Wayland 환경에서 RViz2 화면 캡처
source /opt/ros/jazzy/setup.bash
export DISPLAY=:0

# grim (Wayland screenshotter) 시도
if command -v grim &>/dev/null; then
    echo "Using grim..."
    grim /tmp/rviz_capture.png && echo "Captured with grim"
# scrot (X11 screenshotter) 시도
elif command -v scrot &>/dev/null; then
    echo "Using scrot..."
    scrot /tmp/rviz_capture.png && echo "Captured with scrot"
# import (ImageMagick) 시도
elif command -v import &>/dev/null; then
    echo "Using ImageMagick import..."
    import -window root /tmp/rviz_capture.png && echo "Captured with import"
else
    echo "No screenshot tool found"
    echo "Available: $(which grim scrot import 2>/dev/null)"
fi

# 파일 크기 확인
ls -la /tmp/rviz_capture.png 2>/dev/null
