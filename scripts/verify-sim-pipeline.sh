#!/bin/bash
##############################################################################
# ARGOS 시뮬레이션 파이프라인 종합 검증
# navigation.launch.py 실행 후 이 스크립트로 전체 파이프라인 동작을 확인한다.
#
# 사용법:
#   1. 터미널1: ros2 launch argos_description navigation.launch.py explore:=true
#   2. 터미널2: ./verify-sim-pipeline.sh
#   3. 결과 확인 후 실패 항목 디버깅
#
# 각 항목은 5단계까지 재시도하며, RTF 0.28x 환경을 고려한 타임아웃 적용.
##############################################################################

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PASS=0; FAIL=0; WARN=0

ok()   { echo -e "  ${GREEN}[OK]${NC}   $1"; ((PASS++)); }
fail() { echo -e "  ${RED}[FAIL]${NC} $1"; ((FAIL++)); }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $1"; ((WARN++)); }
info() { echo -e "  ${BLUE}[INFO]${NC} $1"; }

echo "================================================================="
echo "  ARGOS 시뮬레이션 파이프라인 종합 검증"
echo "  $(date '+%Y-%m-%d %H:%M:%S')"
echo "================================================================="
echo ""

# --- 1. Gazebo 시뮬레이션 ---
echo -e "${BLUE}[1] Gazebo 시뮬레이션${NC}"

# /clock 토픽 (Gazebo 실행 증거)
if timeout 5 ros2 topic echo /clock --once &>/dev/null; then
    ok "/clock 수신 — Gazebo 실행 중"
else
    fail "/clock 없음 — Gazebo가 실행 중이 아닙니다"
    echo "  >>> ros2 launch argos_description navigation.launch.py 를 먼저 실행하세요"
    exit 1
fi

# RTF 측정
RTF_LINE=$(timeout 5 ros2 topic hz /clock 2>&1 | grep "average rate" | head -1)
if [ -n "$RTF_LINE" ]; then
    info "Clock rate: $RTF_LINE"
fi
echo ""

# --- 2. gz_bridge 센서 토픽 ---
echo -e "${BLUE}[2] gz_bridge 센서 토픽${NC}"

TOPICS=("/scan" "/camera/image_raw" "/imu/data" "/thermal/image_raw")
TOPIC_LABELS=("LiDAR /scan" "RGB Camera" "IMU" "Thermal Camera")

for i in "${!TOPICS[@]}"; do
    TOPIC="${TOPICS[$i]}"
    LABEL="${TOPIC_LABELS[$i]}"
    if timeout 5 ros2 topic echo "$TOPIC" --once &>/dev/null; then
        ok "$LABEL ($TOPIC)"
    else
        if [ "$TOPIC" = "/scan" ]; then
            fail "$LABEL ($TOPIC) — SLAM/Nav2 작동 불가!"
            echo "  >>> 해결: gz_bridge 재시작 또는 navigation.launch.py 재실행"
        else
            warn "$LABEL ($TOPIC) — 데이터 미수신 (비필수)"
        fi
    fi
done
echo ""

# --- 3. ros2_control (차량 제어) ---
echo -e "${BLUE}[3] ros2_control (DDC)${NC}"

# controller_manager 확인
if ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller"; then
    DDC_STATE=$(ros2 control list_controllers 2>/dev/null | grep "diff_drive_controller" | awk '{print $NF}')
    if [ "$DDC_STATE" = "active" ]; then
        ok "diff_drive_controller active"
    else
        warn "diff_drive_controller 상태: $DDC_STATE (active 아님)"
    fi
else
    fail "diff_drive_controller 미등록"
fi

# odom TF 확인
if timeout 5 ros2 topic echo /odom --once &>/dev/null; then
    ODOM_X=$(timeout 3 ros2 topic echo /odom --once 2>/dev/null | grep -A1 "x:" | head -1 | awk '{print $2}')
    ok "/odom 발행 중 (x=$ODOM_X)"
else
    fail "/odom 없음 — DDC가 odom TF를 발행하지 않음"
fi

# cmd_vel 타입 확인 (TwistStamped 필수)
CMD_TYPE=$(ros2 topic info /cmd_vel 2>/dev/null | grep "Type:" | awk '{print $2}')
if echo "$CMD_TYPE" | grep -q "TwistStamped"; then
    ok "/cmd_vel 타입: TwistStamped"
elif [ -n "$CMD_TYPE" ]; then
    fail "/cmd_vel 타입: $CMD_TYPE (TwistStamped 필요!)"
else
    warn "/cmd_vel 토픽 미존재"
fi
echo ""

# --- 4. SLAM ---
echo -e "${BLUE}[4] SLAM (slam_toolbox)${NC}"

if timeout 10 ros2 topic echo /map --once &>/dev/null; then
    MAP_INFO=$(timeout 5 ros2 topic echo /map --once 2>/dev/null | grep -E "(width|height|resolution)" | head -3)
    ok "/map 발행 중"
    echo "       $MAP_INFO" | tr '\n' ' '
    echo ""
else
    fail "/map 없음 — slam_toolbox가 맵을 생성하지 못함"
    echo "  >>> 확인: /scan_base 토픽 존재 여부, scan_frame_relay 실행 여부"
fi
echo ""

# --- 5. Nav2 ---
echo -e "${BLUE}[5] Nav2 자율주행${NC}"

# Lifecycle 노드 상태
NAV_NODES=("controller_server" "planner_server" "bt_navigator" "behavior_server")
for node in "${NAV_NODES[@]}"; do
    STATE=$(ros2 lifecycle get "/$node" 2>/dev/null | grep "current state" | awk -F'[' '{print $2}' | tr -d ']')
    if [ "$STATE" = "active" ]; then
        ok "$node: active"
    elif [ -n "$STATE" ]; then
        warn "$node: $STATE (active 필요)"
    else
        fail "$node: 미발견 (Nav2 미실행?)"
    fi
done

# navigate_to_pose 액션
NAV_ACTION=$(ros2 action list 2>/dev/null | grep navigate_to_pose)
if [ -n "$NAV_ACTION" ]; then
    ok "navigate_to_pose 액션 서버 가용"
else
    fail "navigate_to_pose 액션 미발견"
fi
echo ""

# --- 6. 열화상 감지 ---
echo -e "${BLUE}[6] 열화상 화점 감지${NC}"

HOTSPOT_NODE=$(ros2 node list 2>/dev/null | grep hotspot)
if [ -n "$HOTSPOT_NODE" ]; then
    ok "hotspot_detector 노드 실행 중"
else
    warn "hotspot_detector 미실행 (NumPy 2.x 크래시 확인)"
    echo "  >>> 해결: pip install 'numpy<2'"
fi
echo ""

# --- 7. Frontier 탐색 (explore:=true 시) ---
echo -e "${BLUE}[7] Frontier 탐색${NC}"

FRONTIER_NODE=$(ros2 node list 2>/dev/null | grep frontier)
if [ -n "$FRONTIER_NODE" ]; then
    ok "frontier_explorer 노드 실행 중"

    # 프론티어 감지 개수
    FRONTIER_COUNT=$(timeout 5 ros2 topic echo /exploration/frontier_count --once 2>/dev/null | grep "data:" | awk '{print $2}')
    if [ -n "$FRONTIER_COUNT" ] && [ "$FRONTIER_COUNT" -gt 0 ] 2>/dev/null; then
        ok "프론티어 $FRONTIER_COUNT개 감지"
    else
        warn "프론티어 0개 — min_frontier_size 조정 필요하거나 맵이 아직 작음"
    fi

    # 탐색 상태
    STATUS=$(timeout 5 ros2 topic echo /exploration/status --once 2>/dev/null | grep "data:" | awk -F'"' '{print $2}')
    if [ -n "$STATUS" ]; then
        info "탐색 상태: $STATUS"
    fi
else
    info "frontier_explorer 미실행 (explore:=true로 실행했는지 확인)"
fi
echo ""

# --- 8. TF 트리 ---
echo -e "${BLUE}[8] TF 트리 무결성${NC}"

# map → odom → base_footprint 체인
for CHAIN in "map odom" "odom base_footprint"; do
    PARENT=$(echo "$CHAIN" | awk '{print $1}')
    CHILD=$(echo "$CHAIN" | awk '{print $2}')
    TF_RESULT=$(timeout 3 ros2 run tf2_ros tf2_echo "$PARENT" "$CHILD" 2>&1 | head -3)
    if echo "$TF_RESULT" | grep -q "Translation"; then
        ok "TF: $PARENT → $CHILD"
    else
        fail "TF: $PARENT → $CHILD 끊김"
    fi
done
echo ""

# --- 최종 결과 ---
echo "================================================================="
echo -e "  결과:  ${GREEN}OK ${PASS}${NC}  /  ${YELLOW}WARN ${WARN}${NC}  /  ${RED}FAIL ${FAIL}${NC}"
echo "================================================================="

if [ "$FAIL" -eq 0 ]; then
    echo ""
    echo -e "${GREEN}전체 파이프라인 정상! Nav2 Goal 테스트 가능:${NC}"
    echo "  ./scripts/nav2/send_nav_goal.sh 2.0 5.0"
    echo ""
    echo "  또는 RViz2에서 2D Goal Pose 클릭:"
    echo "  ros2 launch nav2_bringup rviz_launch.py"
elif [ "$FAIL" -le 2 ]; then
    echo ""
    echo -e "${YELLOW}일부 실패 항목 있음 — 위 해결책 참고${NC}"
else
    echo ""
    echo -e "${RED}주요 컴포넌트 실패 — launch 재실행 권장${NC}"
fi
