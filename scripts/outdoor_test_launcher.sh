#!/bin/bash
# outdoor_test_launcher.sh
# ARGOS Phase F — 달성군 실증공역 야외 테스트 런처
#
# 사용법:
#   ./outdoor_test_launcher.sh             # 실제 기동
#   ./outdoor_test_launcher.sh --dry-run   # 절차 검증만 (시뮬 환경)
#
# 전제:
#   - ROS2 Jazzy 환경 source 완료
#   - UGV: TurtleBot4 2대 (ugv1, ugv2) 전원 ON + WiFi 연결
#   - 드론: PX4 드론 2대 (drone1, drone2) + QGC 사전 파라미터 설정 완료
#   - 비행 승인: 달성군 실증공역 허가 확인 완료

set -euo pipefail

# ============================================================
# 설정 영역
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARGOS_ROOT="$(dirname "$SCRIPT_DIR")"
ROS2_WS="${ARGOS_ROOT}/ros2_ws"

# 로봇 네임스페이스
UGV_NAMESPACES=("ugv1" "ugv2")
DRONE_NAMESPACES=("drone1" "drone2")

# 드론 이륙 고도 (미터)
TAKEOFF_ALTITUDE=3.0

# 임무 타임아웃 (초)
MISSION_TIMEOUT=600

# 배터리 RTL 임계값 (%)
BATTERY_RTL_THRESHOLD=30

# 로그 파일
LOG_DIR="${ARGOS_ROOT}/logs/outdoor_tests"
LOG_FILE="${LOG_DIR}/test_$(date +%Y%m%d_%H%M%S).log"

# dry-run 플래그
DRY_RUN=false
if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=true
fi

# ============================================================
# 헬퍼 함수
# ============================================================

# 색상 출력
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log() {
    local level="$1"
    local msg="$2"
    local timestamp
    timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "[${timestamp}] [${level}] ${msg}" | tee -a "$LOG_FILE"
}

info()    { log "${GREEN}INFO ${NC}" "$1"; }
warn()    { log "${YELLOW}WARN ${NC}" "$1"; }
error()   { log "${RED}ERROR${NC}" "$1"; }
section() { echo -e "\n${BLUE}========================================${NC}"; log "${BLUE}STEP ${NC}" "$1"; echo -e "${BLUE}========================================${NC}\n"; }

# dry-run 래퍼: --dry-run 시 명령을 실행하지 않고 출력만
run_cmd() {
    if [[ "$DRY_RUN" == "true" ]]; then
        info "[DRY-RUN] 실행 예정: $*"
    else
        info "실행: $*"
        eval "$@"
    fi
}

# ROS2 토픽 확인 (타임아웃 내 존재 여부)
wait_for_topic() {
    local topic="$1"
    local timeout="${2:-10}"
    local count=0
    info "토픽 대기: ${topic} (최대 ${timeout}초)"
    while ! ros2 topic list 2>/dev/null | grep -q "$topic"; do
        sleep 1
        ((count++))
        if [[ $count -ge $timeout ]]; then
            error "토픽 ${topic} 미발견 (${timeout}초 초과)"
            return 1
        fi
    done
    info "토픽 확인: ${topic} ✅"
}

# 배터리 레벨 확인
check_battery() {
    local namespace="$1"
    local topic="/${namespace}/battery_state"
    if [[ "$DRY_RUN" == "true" ]]; then
        info "[DRY-RUN] 배터리 확인 생략: ${namespace}"
        return 0
    fi
    local level
    level=$(ros2 topic echo --once "$topic" 2>/dev/null | grep -oP 'percentage: \K[0-9.]+' || echo "0")
    local level_pct
    level_pct=$(echo "$level * 100" | bc | cut -d. -f1)
    if [[ "$level_pct" -lt 80 ]]; then
        warn "${namespace} 배터리 ${level_pct}% — 80% 미만. 충전 후 재시도 권장"
        read -r -p "계속 진행하시겠습니까? (y/N): " confirm
        [[ "$confirm" =~ ^[Yy]$ ]] || { error "사용자 중단"; exit 1; }
    else
        info "${namespace} 배터리 ${level_pct}% ✅"
    fi
}

# 긴급 정지 함수 (Ctrl+C 또는 오류 시 호출)
emergency_stop() {
    echo ""
    error "=== 긴급 정지 발동 ==="
    warn "모든 로봇에 정지 명령 전송 중..."

    # UGV 정지
    for ns in "${UGV_NAMESPACES[@]}"; do
        run_cmd "ros2 topic pub --once /${ns}/cmd_vel geometry_msgs/TwistStamped \
            '{header: {frame_id: base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}' \
            2>/dev/null || true"
    done

    # 드론 RTL
    for ns in "${DRONE_NAMESPACES[@]}"; do
        run_cmd "ros2 topic pub --once /${ns}/emergency_stop std_msgs/Bool '{data: true}' \
            2>/dev/null || true"
    done

    warn "오케스트레이터 종료 중..."
    run_cmd "ros2 lifecycle set /orchestrator_node shutdown 2>/dev/null || true"

    error "긴급 정지 완료. 모든 로봇 상태를 육안으로 확인하십시오."
    exit 1
}

# 트랩 설정 (Ctrl+C, 오류 시 긴급 정지)
trap emergency_stop INT TERM ERR

# ============================================================
# 사전 설정
# ============================================================

# 로그 디렉토리 생성
mkdir -p "$LOG_DIR"

echo -e "${BLUE}"
echo "=================================================="
echo "  ARGOS Phase F — 야외 테스트 런처"
echo "  달성군 실증공역"
if [[ "$DRY_RUN" == "true" ]]; then
    echo "  모드: DRY-RUN (절차 검증만, 실제 기동 없음)"
fi
echo "=================================================="
echo -e "${NC}"
info "로그 파일: ${LOG_FILE}"

# ROS2 환경 확인
if [[ -z "${ROS_DISTRO:-}" ]]; then
    error "ROS2 환경이 source되지 않았습니다. 'source /opt/ros/jazzy/setup.bash' 실행 후 재시도"
    exit 1
fi
info "ROS2 배포판: ${ROS_DISTRO} ✅"

# ============================================================
# Phase 1: Pre-flight 체크
# ============================================================

section "Phase 1: Pre-flight 체크"

info "1-1. 비행 허가 확인"
if [[ "$DRY_RUN" == "false" ]]; then
    echo ""
    echo -e "${YELLOW}[체크리스트] 아래 항목을 모두 확인하셨습니까?${NC}"
    echo "  [ ] 달성군 실증공역 비행 승인서 지참"
    echo "  [ ] 항공안전법 제127조 기준 준수 확인"
    echo "  [ ] 안전요원 2인 이상 현장 배치 확인"
    echo "  [ ] 드론 보험 유효기간 확인 (대인·대물)"
    echo "  [ ] 비상 착륙 지점 사전 지정 완료"
    echo "  [ ] 기상 확인: 풍속 10m/s 이하, 시정 500m 이상"
    echo ""
    read -r -p "모든 항목 확인 완료 후 'yes'를 입력하세요: " preflight_confirm
    [[ "$preflight_confirm" == "yes" ]] || { error "Pre-flight 확인 미완료. 중단."; exit 1; }
fi
info "Pre-flight 확인 완료 ✅"

info "1-2. 배터리 레벨 확인"
for ns in "${UGV_NAMESPACES[@]}" "${DRONE_NAMESPACES[@]}"; do
    check_battery "$ns"
done

info "1-3. 통신 링크 확인 (ROS2 노드 목록)"
if [[ "$DRY_RUN" == "false" ]]; then
    active_nodes=$(ros2 node list 2>/dev/null | wc -l)
    info "활성 ROS2 노드: ${active_nodes}개"
    if [[ "$active_nodes" -lt 4 ]]; then
        warn "활성 노드 수 부족. UGV/드론 시스템이 기동됐는지 확인하세요."
    fi
fi

info "1-4. Emergency Stop 버튼 테스트"
if [[ "$DRY_RUN" == "false" ]]; then
    warn "E-STOP 버튼을 한 번 눌렀다가 해제하세요 (5초 대기)..."
    sleep 5
    info "E-STOP 응답 확인 (수동 확인 필요)"
fi

info "Phase 1 완료 ✅"

# ============================================================
# Phase 2: UGV 배치
# ============================================================

section "Phase 2: UGV 배치"

info "2-1. UGV 시스템 기동 (launch)"
run_cmd "ros2 launch argos_bringup multi_ugv.launch.py \
    use_sim_time:=false \
    robot_namespaces:='[ugv1, ugv2]' \
    &"
UGV_LAUNCH_PID=$!
info "UGV launch PID: ${UGV_LAUNCH_PID}"

sleep 3

info "2-2. UGV Nav2 준비 확인"
for ns in "${UGV_NAMESPACES[@]}"; do
    wait_for_topic "/${ns}/amcl_pose" 30 || {
        error "${ns} AMCL 위치추정 실패"
        emergency_stop
    }
done

info "2-3. UGV 초기 위치 설정 (야외 GPS 기반)"
for ns in "${UGV_NAMESPACES[@]}"; do
    info "${ns} 초기 위치 설정 중..."
    # 실물에서는 AMCL initialpose 또는 GPS → map 변환 사용
    run_cmd "ros2 topic pub --once /${ns}/initialpose geometry_msgs/PoseWithCovarianceStamped \
        '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
        orientation: {w: 1.0}}}}' 2>/dev/null"
done

info "Phase 2 완료 ✅"

# ============================================================
# Phase 3: 드론 Arm + 이륙
# ============================================================

section "Phase 3: 드론 Arm + 이륙"

info "3-1. 드론 uXRCE-DDS 연결 확인"
for ns in "${DRONE_NAMESPACES[@]}"; do
    wait_for_topic "/fmu/${ns}/out/vehicle_status" 15 || {
        error "${ns} PX4 연결 실패"
        emergency_stop
    }
done

info "3-2. 드론 Arm 및 Offboard 전환"
for ns in "${DRONE_NAMESPACES[@]}"; do
    info "${ns} Arm 중..."
    run_cmd "ros2 service call /${ns}/arm std_srvs/srv/SetBool '{data: true}' 2>/dev/null"
    sleep 2

    info "${ns} Offboard 모드 전환..."
    run_cmd "ros2 service call /${ns}/set_offboard std_srvs/srv/Trigger '{}' 2>/dev/null"
    sleep 1
done

info "3-3. 드론 이륙 (목표 고도: ${TAKEOFF_ALTITUDE}m)"
if [[ "$DRY_RUN" == "false" ]]; then
    warn "드론 이륙 시작. 로터에서 5m 이상 거리를 유지하세요."
    sleep 3
fi

for ns in "${DRONE_NAMESPACES[@]}"; do
    info "${ns} 이륙 명령 전송..."
    run_cmd "ros2 topic pub --once /${ns}/takeoff_altitude std_msgs/Float32 \
        '{data: ${TAKEOFF_ALTITUDE}}' 2>/dev/null"
    sleep 3
done

info "3-4. 호버링 안정화 대기 (10초)"
sleep 10

info "Phase 3 완료 ✅"

# ============================================================
# Phase 4: 임무 실행
# ============================================================

section "Phase 4: 임무 실행"

info "4-1. 오케스트레이터 기동"
run_cmd "ros2 lifecycle set /orchestrator_node configure"
sleep 2
run_cmd "ros2 lifecycle set /orchestrator_node activate"
sleep 2

info "4-2. 임무 파라미터 전송"
# 탐색 구역: 달성군 실증공역 좌표 (예시 — 실측 후 수정)
SEARCH_AREA_JSON='{
  "x_min": 0.0, "x_max": 50.0,
  "y_min": 0.0, "y_max": 50.0,
  "resolution": 0.5
}'
run_cmd "ros2 topic pub --once /mission/search_area std_msgs/String \
    '{data: \"${SEARCH_AREA_JSON}\"}' 2>/dev/null"

info "4-3. 임무 시작"
run_cmd "ros2 service call /mission/start std_srvs/srv/Trigger '{}' 2>/dev/null"
info "임무 시작됨. 타임아웃: ${MISSION_TIMEOUT}초"

info "4-4. 배터리 모니터링 루프 (백그라운드)"
{
    while true; do
        sleep 30
        for ns in "${UGV_NAMESPACES[@]}" "${DRONE_NAMESPACES[@]}"; do
            if [[ "$DRY_RUN" == "false" ]]; then
                level=$(ros2 topic echo --once "/${ns}/battery_state" 2>/dev/null | \
                    grep -oP 'percentage: \K[0-9.]+' || echo "1")
                level_pct=$(echo "$level * 100" | bc | cut -d. -f1)
                if [[ "$level_pct" -lt "$BATTERY_RTL_THRESHOLD" ]]; then
                    warn "${ns} 배터리 ${level_pct}% — RTL 임계값 도달!"
                    ros2 topic pub --once "/${ns}/rtl_trigger" std_msgs/Bool '{data: true}' \
                        2>/dev/null || true
                fi
            fi
        done
    done
} &
MONITOR_PID=$!

info "4-5. 임무 완료 대기 (최대 ${MISSION_TIMEOUT}초)"
if [[ "$DRY_RUN" == "false" ]]; then
    timeout_count=0
    while true; do
        mission_status=$(ros2 topic echo --once /mission/status 2>/dev/null | \
            grep -oP 'data: \K\w+' || echo "RUNNING")
        info "임무 상태: ${mission_status}"

        if [[ "$mission_status" == "COMPLETE" ]] || [[ "$mission_status" == "FAILED" ]]; then
            info "임무 종료: ${mission_status}"
            break
        fi

        sleep 10
        ((timeout_count += 10))
        if [[ $timeout_count -ge $MISSION_TIMEOUT ]]; then
            warn "임무 타임아웃 (${MISSION_TIMEOUT}초). RTL 명령 전송."
            break
        fi
    done
else
    info "[DRY-RUN] 임무 대기 시뮬레이션 (5초)"
    sleep 5
fi

# 배터리 모니터 종료
kill "$MONITOR_PID" 2>/dev/null || true

info "Phase 4 완료 ✅"

# ============================================================
# Phase 5: RTL + 종료
# ============================================================

section "Phase 5: RTL + 종료"

info "5-1. 오케스트레이터 임무 중단"
run_cmd "ros2 service call /mission/stop std_srvs/srv/Trigger '{}' 2>/dev/null"
sleep 2

info "5-2. 드론 RTL (Return to Launch)"
for ns in "${DRONE_NAMESPACES[@]}"; do
    info "${ns} RTL 명령..."
    run_cmd "ros2 topic pub --once /${ns}/rtl_trigger std_msgs/Bool '{data: true}' 2>/dev/null"
    sleep 1
done

info "5-3. 드론 착륙 대기"
if [[ "$DRY_RUN" == "false" ]]; then
    warn "드론 RTL 중... 착륙 완료까지 안전 거리 유지 (30초 대기)"
    sleep 30
else
    info "[DRY-RUN] RTL 대기 시뮬레이션 (3초)"
    sleep 3
fi

info "5-4. UGV 홈 복귀"
for ns in "${UGV_NAMESPACES[@]}"; do
    info "${ns} 홈 복귀 중..."
    run_cmd "ros2 service call /${ns}/navigate_to_pose nav2_msgs/srv/NavigateToPose \
        '{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
        orientation: {w: 1.0}}}}' 2>/dev/null"
done

info "5-5. 시스템 정상 종료"
run_cmd "ros2 lifecycle set /orchestrator_node deactivate"
sleep 1
run_cmd "ros2 lifecycle set /orchestrator_node cleanup"
sleep 1
run_cmd "ros2 lifecycle set /orchestrator_node shutdown"

# UGV launch 종료
if [[ -n "${UGV_LAUNCH_PID:-}" ]]; then
    kill "$UGV_LAUNCH_PID" 2>/dev/null || true
fi

info "Phase 5 완료 ✅"

# ============================================================
# 완료 보고
# ============================================================

section "테스트 완료"

info "로그 파일 저장 위치: ${LOG_FILE}"
if [[ "$DRY_RUN" == "true" ]]; then
    info "DRY-RUN 모드 완료 — 절차 검증 통과. 실제 기동 시 '--dry-run' 옵션 제거"
else
    info "야외 테스트 정상 완료"
fi

echo -e "\n${GREEN}[ARGOS Phase F 야외 테스트 완료]${NC}\n"

# 트랩 해제 (정상 종료)
trap - INT TERM ERR
exit 0
