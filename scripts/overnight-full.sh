#!/bin/bash
# ARGOS 야간 풀 자동화 스크립트 (~4시간)
# 사용: wsl -d Ubuntu -- nohup bash /mnt/c/.../overnight-full.sh &
# 결과: /tmp/overnight/summary.txt

set -o pipefail
LOGDIR="/tmp/overnight"
mkdir -p $LOGDIR
exec > "$LOGDIR/master.log" 2>&1
set -x

echo "=== ARGOS 야간 풀 자동화 시작: $(date) ==="
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws && source install/setup.bash

kill_all() { pkill -f gz_sim 2>/dev/null; pkill -f ros2 2>/dev/null; sleep 5; }
log() { echo "[$(date +%H:%M:%S)] $1" | tee -a "$LOGDIR/summary.txt"; }

# ═══════════════════════════════════════════
# Phase 1: 패키지 설치 + 빌드 (10분)
# ═══════════════════════════════════════════
log "Phase 1: 빌드 + 패키지"
pip install onnx 2>&1 | tail -3
colcon build --symlink-install 2>&1 | tail -5
source install/setup.bash
log "Phase 1 완료"

# ═══════════════════════════════════════════
# Phase 2: TensorRT 변환 — 모든 모델 (20분)
# ═══════════════════════════════════════════
log "Phase 2: TensorRT 변환"
MODELS_DIR="/mnt/c/Users/USER/Desktop/SUPERAGENT/projects/argos/models/fire-smoke/pretrained"
for pt in "$MODELS_DIR"/*.pt; do
  NAME=$(basename "$pt" .pt)
  log "  변환: $NAME"
  python3 -c "
from ultralytics import YOLO
import time
model = YOLO('$pt')
start = time.time()
model.export(format='engine', half=True, device=0, imgsz=640)
elapsed = time.time() - start
print(f'$NAME: {elapsed:.1f}초')
" > "$LOGDIR/tensorrt_${NAME}.log" 2>&1
  log "  결과: $(tail -1 $LOGDIR/tensorrt_${NAME}.log)"
done

# ═══════════════════════════════════════════
# Phase 3: Nav2 Goal 이동 — 5개 좌표 (40분)
# ═══════════════════════════════════════════
log "Phase 3: Nav2 goal 다중 좌표 테스트"
ros2 launch argos_description navigation.launch.py headless:=true explore:=false > "$LOGDIR/nav2.log" 2>&1 &
NAV2_PID=$!
sleep 240  # Nav2 전체 활성화 대기

MAP=$(timeout 10 ros2 topic echo /map --once 2>/dev/null | grep "width:" | head -1)
log "  MAP: $MAP"

GOALS=("3.0 2.0" "1.0 4.0" "-2.0 1.0" "5.0 0.5" "0.0 3.0")
for i in "${!GOALS[@]}"; do
  IFS=' ' read -r GX GY <<< "${GOALS[$i]}"
  log "  Goal $((i+1))/5: ($GX, $GY)"

  BEFORE=$(timeout 5 ros2 topic echo /odom --once 2>/dev/null | grep "x:" | head -1)

  timeout 120 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY}, orientation: {w: 1.0}}}}" \
    > "$LOGDIR/goal_${i}.log" 2>&1

  sleep 30
  AFTER=$(timeout 5 ros2 topic echo /odom --once 2>/dev/null | grep "x:" | head -1)
  log "  Before: $BEFORE → After: $AFTER"
  log "  Result: $(head -3 $LOGDIR/goal_${i}.log)"
done

kill $NAV2_PID 2>/dev/null; wait $NAV2_PID 2>/dev/null; sleep 5

# ═══════════════════════════════════════════
# Phase 4: Frontier 자율 탐색 — 파라미터 스윕 (60분)
# ═══════════════════════════════════════════
log "Phase 4: Frontier 파라미터 스윕"
FRONTIER_SIZES=(5 10 20 40)
for fs in "${FRONTIER_SIZES[@]}"; do
  log "  min_frontier_size=$fs 테스트"
  kill_all

  ros2 launch argos_description navigation.launch.py headless:=true explore:=true \
    > "$LOGDIR/frontier_fs${fs}.log" 2>&1 &
  FP=$!
  # frontier_explorer 파라미터 동적 변경
  sleep 180
  ros2 param set /frontier_explorer min_frontier_size $fs 2>/dev/null
  sleep 420  # 7분 탐색

  DROPS=$(grep -c "dropping" "$LOGDIR/frontier_fs${fs}.log" 2>/dev/null)
  COMPLETE=$(grep -c "EXPLORATION COMPLETE" "$LOGDIR/frontier_fs${fs}.log" 2>/dev/null)
  FRONTIERS=$(grep "frontier" "$LOGDIR/frontier_fs${fs}.log" | grep -c "target")
  log "  fs=$fs: drops=$DROPS, complete=$COMPLETE, frontiers_visited=$FRONTIERS"

  kill $FP 2>/dev/null; wait $FP 2>/dev/null; sleep 5
done

# ═══════════════════════════════════════════
# Phase 5: 멀티로봇 TF frame_prefix 검증 (20분)
# ═══════════════════════════════════════════
log "Phase 5: 멀티로봇 2대 TF 검증"
kill_all
ros2 launch argos_description multi_robot.launch.py > "$LOGDIR/multi.log" 2>&1 &
MP=$!
sleep 120

TOPICS=$(ros2 topic list 2>/dev/null | wc -l)
ARGOS_TOPICS=$(ros2 topic list 2>/dev/null | grep -c "argos")
TF_FRAMES=$(timeout 5 ros2 topic echo /tf --once 2>/dev/null | grep "frame_id:" | head -5)
log "  Topics: $TOPICS total, $ARGOS_TOPICS argos-specific"
log "  TF frames: $TF_FRAMES"

kill $MP 2>/dev/null; wait $MP 2>/dev/null; sleep 5

# ═══════════════════════════════════════════
# Phase 6: RTF 벤치마크 — 로봇 1/2/3대 (30분)
# ═══════════════════════════════════════════
log "Phase 6: RTF 벤치마크"
for NROBOTS in 1 2; do
  kill_all

  if [ $NROBOTS -eq 1 ]; then
    ros2 launch argos_description gazebo.launch.py headless:=true > /dev/null 2>&1 &
  else
    ros2 launch argos_description multi_robot.launch.py > /dev/null 2>&1 &
  fi
  RP=$!
  sleep 90

  T1=$(timeout 5 ros2 topic echo /clock --once 2>/dev/null | grep "sec:" | head -1 | awk '{print $2}')
  sleep 60
  T2=$(timeout 5 ros2 topic echo /clock --once 2>/dev/null | grep "sec:" | head -1 | awk '{print $2}')

  if [ -n "$T1" ] && [ -n "$T2" ]; then
    RTF=$(echo "scale=3; ($T2 - $T1) / 60" | bc 2>/dev/null)
    log "  ${NROBOTS}대: RTF=$RTF (sim ${T1}→${T2})"
  else
    log "  ${NROBOTS}대: RTF 측정 실패"
  fi

  kill $RP 2>/dev/null; wait $RP 2>/dev/null; sleep 5
done

# ═══════════════════════════════════════════
# Phase 7: 센서 노드 스트레스 테스트 (20분)
# ═══════════════════════════════════════════
log "Phase 7: 센서 노드 기동 테스트"
kill_all
ros2 launch argos_description navigation.launch.py headless:=true explore:=false > /dev/null 2>&1 &
SP=$!
sleep 180

# 8중 센싱 노드 각각 토픽 발행 확인
SENSORS=("scan" "camera/image_raw" "depth_camera/depth_image" "imu/data" "thermal/image_raw" "odom")
for sensor in "${SENSORS[@]}"; do
  HZ=$(timeout 10 ros2 topic hz /$sensor --window 3 2>/dev/null | grep "average" | head -1)
  log "  /$sensor: $HZ"
done

kill $SP 2>/dev/null; wait $SP 2>/dev/null; sleep 5

# ═══════════════════════════════════════════
# Phase 8: 전체 pytest (5분)
# ═══════════════════════════════════════════
log "Phase 8: pytest 전체"
kill_all
python3 -m pytest src/argos_bringup/test/ \
  --ignore=src/argos_bringup/test/test_copyright.py \
  --ignore=src/argos_bringup/test/test_flake8.py \
  --ignore=src/argos_bringup/test/test_pep257.py \
  -q --tb=short > "$LOGDIR/pytest.log" 2>&1
log "PYTEST: $(tail -1 $LOGDIR/pytest.log)"

# ═══════════════════════════════════════════
# 최종 요약
# ═══════════════════════════════════════════
kill_all

echo "" >> "$LOGDIR/summary.txt"
echo "============================================" >> "$LOGDIR/summary.txt"
echo "  ARGOS 야간 작업 완료: $(date)" >> "$LOGDIR/summary.txt"
echo "  소요 시간: $SECONDS 초" >> "$LOGDIR/summary.txt"
echo "  로그: $LOGDIR/" >> "$LOGDIR/summary.txt"
echo "============================================" >> "$LOGDIR/summary.txt"

log "=== 전체 완료 ($SECONDS 초) ==="
