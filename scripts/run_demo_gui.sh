#!/bin/bash
# ARGOS 단일 로봇 Nav2 데모 (GUI 모드 - WSLg)
# 사용법: WSL 터미널에서 bash /mnt/c/Users/USER/Desktop/SUPERAGENT/projects/argos/scripts/run_demo_gui.sh
#
# 녹화: Windows Game Bar (Win+G) 또는 OBS로 Gazebo 창 캡처
# Gazebo 카메라: 마우스 드래그(회전), 스크롤(줌), Shift+드래그(이동)

source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
source install/setup.bash

echo "============================================="
echo "  ARGOS UGV Nav2 + SLAM Demo"
echo "  Single UGV — indoor_test world"
echo "  Ctrl+C로 종료"
echo "============================================="
echo ""
echo "Gazebo GUI가 열리면 약 60초 대기 (SLAM 초기화)"
echo "그 후 별도 터미널에서 Nav2 goal 전송 또는 teleop 사용:"
echo ""
echo "  # Nav2 goal (map 프레임 좌표):"
echo "  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\"
echo "    \"{pose: {header: {frame_id: map}, pose: {position: {x: -2.0, y: 0.0}, orientation: {w: 1.0}}}}\""
echo ""
echo "  # 직접 조작 (TwistStamped):"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \"{twist: {linear: {x: 0.5}}}\" --rate 10"
echo ""

export LIBGL_ALWAYS_SOFTWARE=0
export GZ_SIM_RENDER_ENGINE_GUI=ogre2

ros2 launch argos_description navigation.launch.py headless:=false explore:=true
