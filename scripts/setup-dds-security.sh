#!/bin/bash
# ARGOS DDS 보안 설정 스크립트 (SROS2)
#
# 목적: ROS2 노드 간 암호화 통신을 위한 키스토어 및 엔클레이브 생성
# 전제: ros-jazzy-ros-base 및 sros2 패키지 설치 완료
#
# 사용법:
#   bash scripts/setup-dds-security.sh [키스토어 경로]
#   예) bash scripts/setup-dds-security.sh ~/argos_keystore
#
# 이후 런치 환경변수 설정:
#   export ROS_SECURITY_KEYSTORE=~/argos_keystore
#   export ROS_SECURITY_ENABLE=true
#   export ROS_SECURITY_STRATEGY=Enforce
#   ros2 launch argos_description navigation.launch.py
#
# 보안 모드 설명:
#   Enforce  : 미등록 노드 실행 거부 (현장 운용 권장)
#   Permissive: 미등록 노드도 허용하되 경고 (개발·테스트용)

set -e  # 오류 발생 시 즉시 종료

KEYSTORE_DIR="${1:-$HOME/argos_keystore}"

echo "=== ARGOS DDS Security Setup (SROS2) ==="
echo "키스토어 경로: $KEYSTORE_DIR"
echo ""

# ROS2 환경 소싱 확인
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        # shellcheck disable=SC1091
        source /opt/ros/jazzy/setup.bash
        echo "[OK] ROS2 Jazzy 환경 소싱 완료"
    else
        echo "[ERROR] ROS2 환경이 설정되지 않았습니다."
        echo "  source /opt/ros/jazzy/setup.bash 후 재실행하세요."
        exit 1
    fi
else
    echo "[OK] ROS2 $ROS_DISTRO 환경 감지"
fi

# 기존 키스토어 백업 (재생성 시 덮어쓰기 방지)
if [ -d "$KEYSTORE_DIR" ]; then
    BACKUP_DIR="${KEYSTORE_DIR}_backup_$(date +%Y%m%d_%H%M%S)"
    echo "[WARN] 기존 키스토어 발견 → 백업: $BACKUP_DIR"
    mv "$KEYSTORE_DIR" "$BACKUP_DIR"
fi

# 키스토어 루트 생성
echo ""
echo "[1/2] 키스토어 생성 중..."
ros2 security create_keystore "$KEYSTORE_DIR"
echo "  완료: $KEYSTORE_DIR"

# ARGOS 노드별 엔클레이브 생성
# 엔클레이브 경로는 /argos/<노드명> 형식 (ROS2 네임스페이스와 일치)
echo ""
echo "[2/2] 노드 엔클레이브 생성 중..."
NODES=(
    "orchestrator"        # 멀티에이전트 오케스트레이터
    "frontier_explorer"   # 프론티어 탐색 노드
    "hotspot_detector"    # 화점 감지 노드
    "gas_sensor"          # 가스 센서 노드
    "drone_controller"    # 드론 제어 노드
    "scenario_runner"     # 시나리오 실행 노드
    "robot_status"        # 로봇 상태 모니터
    "perception_bridge"   # 인지 브리지 노드
)

for node in "${NODES[@]}"; do
    ros2 security create_enclave "$KEYSTORE_DIR" /argos/"$node"
    echo "  [OK] /argos/$node"
done

# 생성 결과 요약
echo ""
echo "=== 설정 완료 ==="
echo ""
echo "생성된 엔클레이브 목록:"
find "$KEYSTORE_DIR/enclaves" -name "*.pem" -path "*/identity_ca*" \
    | sed 's|.*/enclaves/||; s|/identity_ca.*||' \
    | sort \
    | sed 's/^/  /'
echo ""
echo "런치 전 환경변수 설정:"
echo "  export ROS_SECURITY_KEYSTORE=$KEYSTORE_DIR"
echo "  export ROS_SECURITY_ENABLE=true"
echo "  export ROS_SECURITY_STRATEGY=Enforce"
echo ""
echo "테스트 (보안 적용 토픽 리스트):"
echo "  ros2 topic list"
