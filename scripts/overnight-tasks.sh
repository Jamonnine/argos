#!/bin/bash
# ARGOS 야간 자동 작업 스크립트
# 사용: wsl -d Ubuntu -- bash /mnt/c/Users/USER/Desktop/SUPERAGENT/projects/argos/scripts/overnight-tasks.sh
# 모든 결과는 /tmp/overnight_*.log에 기록

set -x
exec > /tmp/overnight_master.log 2>&1
echo "=== ARGOS 야간 작업 시작: $(date) ==="

source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws && source install/setup.bash

# ─── 1. onnx 설치 + TensorRT 변환 ───
echo "=== [1/7] TensorRT 변환 ==="
pip install onnx 2>&1 | tail -3
python3 -c "
from ultralytics import YOLO
model = YOLO('/mnt/c/Users/USER/Desktop/SUPERAGENT/projects/argos/models/fire-smoke/pretrained/luminous0219_best.pt')
model.export(format='engine', half=True, device=0, imgsz=640)
print('TensorRT 변환 완료')
" > /tmp/overnight_tensorrt.log 2>&1
echo "TensorRT: $(tail -1 /tmp/overnight_tensorrt.log)"

# ─── 2. Nav2 + SLAM 맵 생성 + goal 이동 ───
echo "=== [2/7] Nav2 goal 이동 테스트 ==="
ros2 launch argos_description navigation.launch.py headless:=true explore:=false > /tmp/overnight_nav2.log 2>&1 &
NAV2_PID=$!
sleep 240  # 4분 대기 (DDC+Nav2 전체 활성화)

# 맵 확인
MAP=$(timeout 10 ros2 topic echo /map --once 2>/dev/null | grep "width:" | head -1)
echo "MAP: $MAP"

# goal 전송
timeout 120 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 2.0}, orientation: {w: 1.0}}}}" \
  > /tmp/overnight_goal.log 2>&1
echo "GOAL: $(cat /tmp/overnight_goal.log | head -5)"

# odom 확인
sleep 30
ODOM=$(timeout 5 ros2 topic echo /odom --once 2>/dev/null | grep "x:" | head -1)
echo "ODOM: $ODOM"

kill $NAV2_PID 2>/dev/null
wait $NAV2_PID 2>/dev/null
sleep 5

# ─── 3. Frontier 자율 탐색 ───
echo "=== [3/7] Frontier 탐색 테스트 ==="
ros2 launch argos_description navigation.launch.py headless:=true explore:=true > /tmp/overnight_frontier.log 2>&1 &
FRONT_PID=$!
sleep 300  # 5분 탐색

COVERAGE=$(grep "coverage" /tmp/overnight_frontier.log | tail -1)
FRONTIERS=$(grep "frontier" /tmp/overnight_frontier.log | grep -v "process\|SHM" | tail -3)
echo "COVERAGE: $COVERAGE"
echo "FRONTIERS: $FRONTIERS"

kill $FRONT_PID 2>/dev/null
wait $FRONT_PID 2>/dev/null
sleep 5

# ─── 4. Zenoh 테스트 (설치 시도) ───
echo "=== [4/7] Zenoh 테스트 ==="
if dpkg -l ros-jazzy-rmw-zenoh-cpp 2>/dev/null | grep -q "^ii"; then
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  ros2 launch argos_description gazebo.launch.py headless:=true > /tmp/overnight_zenoh.log 2>&1 &
  ZENOH_PID=$!
  sleep 60
  ros2 topic list > /tmp/overnight_zenoh_topics.txt 2>/dev/null
  echo "Zenoh topics: $(wc -l < /tmp/overnight_zenoh_topics.txt)"
  kill $ZENOH_PID 2>/dev/null
  wait $ZENOH_PID 2>/dev/null
else
  echo "Zenoh: NOT INSTALLED (skip)"
fi

# ─── 5. RTF 측정 ───
echo "=== [5/7] RTF 측정 ==="
ros2 launch argos_description gazebo.launch.py headless:=true > /dev/null 2>&1 &
RTF_PID=$!
sleep 60
SIM_TIME_1=$(timeout 3 ros2 topic echo /clock --once 2>/dev/null | grep "sec:" | head -1 | awk '{print $2}')
sleep 30
SIM_TIME_2=$(timeout 3 ros2 topic echo /clock --once 2>/dev/null | grep "sec:" | head -1 | awk '{print $2}')
if [ -n "$SIM_TIME_1" ] && [ -n "$SIM_TIME_2" ]; then
  RTF=$(echo "scale=2; ($SIM_TIME_2 - $SIM_TIME_1) / 30" | bc 2>/dev/null)
  echo "RTF: $RTF (sim ${SIM_TIME_1}→${SIM_TIME_2} in 30 wall-sec)"
else
  echo "RTF: measurement failed"
fi
kill $RTF_PID 2>/dev/null
wait $RTF_PID 2>/dev/null
sleep 5

# ─── 6. 멀티로봇 테스트 ───
echo "=== [6/7] 멀티로봇 2대 테스트 ==="
ros2 launch argos_description multi_robot.launch.py > /tmp/overnight_multi.log 2>&1 &
MULTI_PID=$!
sleep 120
ROBOTS=$(ros2 topic list 2>/dev/null | grep -c "argos")
echo "Multi-robot topics: $ROBOTS"
kill $MULTI_PID 2>/dev/null
wait $MULTI_PID 2>/dev/null

# ─── 7. 전체 pytest ───
echo "=== [7/7] pytest 전체 실행 ==="
cd ~/ros2_ws
python3 -m pytest src/argos_bringup/test/ \
  --ignore=src/argos_bringup/test/test_copyright.py \
  --ignore=src/argos_bringup/test/test_flake8.py \
  --ignore=src/argos_bringup/test/test_pep257.py \
  -q --tb=short > /tmp/overnight_pytest.log 2>&1
echo "PYTEST: $(tail -1 /tmp/overnight_pytest.log)"

# ─── 결과 요약 ───
echo ""
echo "============================================"
echo "  ARGOS 야간 작업 완료: $(date)"
echo "============================================"
echo "  TensorRT: $(tail -1 /tmp/overnight_tensorrt.log)"
echo "  MAP: $MAP"
echo "  NAV2 GOAL: $(head -1 /tmp/overnight_goal.log)"
echo "  ODOM: $ODOM"
echo "  FRONTIER: $(echo $FRONTIERS | head -1)"
echo "  RTF: ${RTF:-N/A}"
echo "  MULTI: ${ROBOTS:-N/A} topics"
echo "  PYTEST: $(tail -1 /tmp/overnight_pytest.log)"
echo "============================================"
echo "로그 파일: /tmp/overnight_*.log"
