#!/bin/bash
# G-4: Gazebo SIL (Software-in-the-Loop) 테스트 5종
# 실행: bash scripts/sil_tests.sh
# 환경: WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic
# 예상 소요: ~30분 (RTF 0.7x 기준)

set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

PASS=0
FAIL=0
RESULTS=""

run_test() {
    local name=$1
    local timeout_sec=$2
    shift 2
    echo ""
    echo "========== SIL TEST: $name =========="
    if timeout $timeout_sec bash -c "$@"; then
        echo "  [PASS] $name"
        PASS=$((PASS+1))
        RESULTS="$RESULTS\n  [PASS] $name"
    else
        echo "  [FAIL] $name"
        FAIL=$((FAIL+1))
        RESULTS="$RESULTS\n  [FAIL] $name"
    fi
    # 정리
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f ros2 2>/dev/null || true
    sleep 5
}

# ── SIL-1: Nav2 Goal SUCCEEDED ──
run_test "Nav2 Goal" 240 '
ros2 launch argos_description navigation.launch.py headless:=true &
sleep 100
for i in $(seq 1 15); do
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.5}}}" 2>/dev/null
  sleep 0.3
done
sleep 25
RESULT=$(timeout 60 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" 2>&1)
echo "$RESULT" | grep -q "SUCCEEDED"
'

# ── SIL-2: cmd_vel 물리 이동 ──
run_test "cmd_vel Movement" 180 '
ros2 launch argos_description navigation.launch.py headless:=true &
sleep 60
START=$(timeout 10 ros2 topic echo /odom --once 2>/dev/null | grep "x:" | head -1 | awk "{print \$2}")
for i in $(seq 1 10); do
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
    "{header: {frame_id: base_link}, twist: {linear: {x: 0.5}, angular: {z: 0.0}}}" 2>/dev/null
  sleep 0.5
done
sleep 5
END=$(timeout 10 ros2 topic echo /odom --once 2>/dev/null | grep "x:" | head -1 | awk "{print \$2}")
echo "Start=$START End=$END"
python3 -c "import sys; sys.exit(0 if abs(float(\"${END:-0}\") - float(\"${START:-0}\")) > 0.1 else 1)"
'

# ── SIL-3: LiDAR scan 발행 확인 ──
run_test "LiDAR Scan" 120 '
ros2 launch argos_description navigation.launch.py headless:=true &
sleep 60
HZ=$(timeout 15 ros2 topic hz /scan 2>/dev/null | head -1)
echo "Scan Hz: $HZ"
echo "$HZ" | grep -q "average rate"
'

# ── SIL-4: 셰르파 스폰 ──
run_test "Sherpa Spawn" 120 '
gz sim -s -r ~/ros2_ws/install/argos_description/share/argos_description/worlds/indoor_test.sdf &
sleep 10
python3 /mnt/c/Users/USER/Desktop/SUPERAGENT/projects/argos/scripts/test_sherpa_spawn.py 2>&1 | grep -q "successful"
'

# ── SIL-5: 멀티로봇 스폰 (3대) ──
run_test "Multi-Robot Spawn" 180 '
ros2 launch argos_description exploration.launch.py headless:=true &
sleep 90
TOPICS=$(ros2 topic list 2>/dev/null)
echo "$TOPICS" | grep -q "/argos1/odom" && echo "$TOPICS" | grep -q "/argos2/odom" && echo "$TOPICS" | grep -q "/drone1"
'

# ── 결과 ──
echo ""
echo "================================="
echo "  G-4 SIL 테스트 결과"
echo "================================="
echo -e "$RESULTS"
echo ""
echo "  PASS: $PASS / $((PASS+FAIL))"
echo "================================="
