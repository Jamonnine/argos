#!/bin/bash
# ARGOS rosbridge 원클릭 런처
# ================================
# WSL 터미널에서 실행하면:
# 1. Gazebo 시뮬레이션 시작 (headless)
# 2. Nav2 + SLAM 시작
# 3. rosbridge WebSocket 서버 시작 (포트 9090)
# 4. 포털(daegufire.ai.kr/argos)에서 자동 연결됨
#
# 사용법:
#   bash /mnt/c/Users/USER/Desktop/SUPERAGENT/projects/argos/scripts/start-rosbridge.sh
#
# 종료: Ctrl+C

set -e

source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
source install/setup.bash

echo "============================================="
echo "  ARGOS ROS Bridge Server"
echo "  포털 연결: daegufire.ai.kr/argos"
echo "  WebSocket: ws://localhost:9090"
echo "============================================="
echo ""
echo "[1/3] Gazebo headless 시작..."
ros2 launch argos_description navigation.launch.py headless:=true &
GAZEBO_PID=$!
sleep 5

echo "[2/3] rosbridge WebSocket 서버 시작 (포트 9090)..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &
BRIDGE_PID=$!
sleep 3

echo "[3/3] 오케스트레이터 + 시나리오 러너 시작..."
ros2 run argos_bringup robot_status --ros-args -p robot_id:=argos1 -p robot_type:=ugv -p use_sim_time:=true &
ros2 run argos_bringup orchestrator --ros-args -p use_sim_time:=true -p expected_robots:="['argos1']" &
sleep 2

echo ""
echo "============================================="
echo "  준비 완료!"
echo "  브라우저에서 daegufire.ai.kr/argos 접속"
echo "  → '다시 시도' 또는 새로고침"
echo "  → 실시간 모니터링 시작"
echo "============================================="
echo ""
echo "Ctrl+C로 종료"

# Wait for all background processes
wait $GAZEBO_PID $BRIDGE_PID
