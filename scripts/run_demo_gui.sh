#!/bin/bash
# ARGOS 전체 데모 (GUI 모드 - WSLg)
# 사용법: wsl -d Ubuntu -- bash /mnt/c/Users/USER/Desktop/SUPERAGENT/projects/argos/scripts/run_demo_gui.sh
#
# 녹화 방법:
#   1. Windows OBS Studio로 WSLg 창 캡처
#   2. 또는 Windows Game Bar (Win+G) 사용
#   3. Gazebo 내 카메라 조작: 마우스 드래그(회전), 스크롤(줌), Shift+드래그(이동)

source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
source install/setup.bash

echo "============================================="
echo "  ARGOS Demo - 이종 군집 소방 로봇 시연"
echo "  UGV x2 + Drone x1"
echo "  Ctrl+C로 종료"
echo "============================================="
echo ""
echo "시작 후 약 60초 뒤 오케스트레이터가 EXPLORING 진입합니다."
echo "약 90초 뒤 화재 시뮬레이션이 시작됩니다."
echo ""

# Gazebo GUI 렌더링 모드 (WSLg)
export LIBGL_ALWAYS_SOFTWARE=0
export GZ_SIM_RENDER_ENGINE_GUI=ogre2

ros2 launch argos_description demo.launch.py headless:=false
