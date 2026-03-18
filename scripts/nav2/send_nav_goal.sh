#!/bin/bash
# ARGOS UGV Nav2 Goal 전송 스크립트
# 사용법:
#   ./send_nav_goal.sh              (기본 목표: 방 B 중앙 2.0, 5.0)
#   ./send_nav_goal.sh 3.0 4.0     (커스텀 좌표)
#   ./send_nav_goal.sh check        (사전 점검만)

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# --- 사전 점검 ---
echo "=== ARGOS Nav2 Goal 사전 점검 ==="

# 1. /clock 확인 (Gazebo 실행 여부)
echo -n "  [1] /clock 토픽: "
if timeout 3 ros2 topic echo /clock --once &>/dev/null; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FAIL${NC} — Gazebo가 실행 중이 아닙니다"
    exit 1
fi

# 2. /scan 확인 (gz_bridge 정상 여부)
echo -n "  [2] /scan 토픽: "
SCAN_COUNT=$(timeout 3 ros2 topic hz /scan 2>&1 | grep "average rate" | head -1)
if [ -n "$SCAN_COUNT" ]; then
    echo -e "${GREEN}OK${NC} ($SCAN_COUNT)"
else
    echo -e "${YELLOW}WARN${NC} — /scan 데이터 없음. gz_bridge 재시작 필요?"
    echo "         ros2 topic list | grep scan 으로 확인하세요"
fi

# 3. Nav2 action server 확인
echo -n "  [3] navigate_to_pose 액션: "
NAV_ACTION=$(ros2 action list 2>/dev/null | grep navigate_to_pose)
if [ -n "$NAV_ACTION" ]; then
    echo -e "${GREEN}OK${NC} ($NAV_ACTION)"
else
    echo -e "${RED}FAIL${NC} — Nav2 action server 미발견"
    echo "         Nav2 lifecycle이 아직 활성화 안 됐을 수 있습니다 (90초+ 대기)"
    exit 1
fi

# 4. odom TF 확인
echo -n "  [4] odom→base_footprint TF: "
TF_CHECK=$(timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -3)
if echo "$TF_CHECK" | grep -q "Translation"; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${YELLOW}WARN${NC} — TF 체인 미확인 (DDC 초기화 대기 중?)"
fi

# 5. SLAM 맵 확인
echo -n "  [5] /map 토픽: "
MAP_CHECK=$(timeout 3 ros2 topic echo /map --once 2>&1 | head -1)
if echo "$MAP_CHECK" | grep -q "header"; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${YELLOW}WARN${NC} — 맵 미생성 (SLAM 아직 초기화 중?)"
fi

echo ""

# check 모드면 여기서 종료
if [ "$1" = "check" ]; then
    echo "사전 점검 완료. Goal 전송은 좌표를 지정하세요."
    exit 0
fi

# --- Goal 전송 ---
# ARGOS indoor_test.sdf 맵 좌표계:
#   방 A: (1.0~4.0, 0.5~4.5)   로봇 스폰: (5.0, 2.5)
#   방 B: (1.0~4.0, 5.5~9.5)   복도: (5.0~6.0, 0.5~9.5)
#   방 C: (7.0~10.0, 0.5~4.5)
GOAL_X=${1:-2.0}
GOAL_Y=${2:-5.0}

echo "=== Nav2 Goal 전송: ($GOAL_X, $GOAL_Y) ==="
echo "    스폰 위치: (5.0, 2.5) → 목표: ($GOAL_X, $GOAL_Y)"
echo ""

# 현재 위치 기록
echo "--- 전송 전 odom ---"
timeout 2 ros2 topic echo /odom --once 2>&1 | grep -A3 "position:" | head -4

echo ""
echo "--- Goal 전송 중 (30초 타임아웃) ---"
timeout 30 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{ pose: { header: { frame_id: 'map' }, pose: { position: { x: $GOAL_X, y: $GOAL_Y, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } } }" \
  --feedback 2>&1 | head -60

echo ""
echo "--- 전송 후 odom ---"
timeout 2 ros2 topic echo /odom --once 2>&1 | grep -A3 "position:" | head -4
