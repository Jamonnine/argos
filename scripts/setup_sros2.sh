#!/bin/bash
# ARGOS SROS2 키스토어 자동 생성 (C-4)
# =========================================
# SROS2(Secure ROS2): DDS-Security 기반 노드 인증·암호화·접근제어.
# 각 노드에 X.509 인증서 + 개인키를 발급하여 도청·위장·주입 공격을 방지한다.
#
# 사전 조건:
#   - ROS 2 Jazzy 설치 및 source 완료
#   - OpenSSL 설치 (인증서 서명에 사용)
#
# 사용법:
#   bash scripts/setup_sros2.sh
#   bash scripts/setup_sros2.sh /opt/argos/keystore   # 저장 경로 지정
#
# 활성화:
#   export ROS_SECURITY_KEYSTORE=$KEYSTORE
#   export ROS_SECURITY_ENABLE=true
#   export ROS_SECURITY_STRATEGY=Enforce   # Permissive(경고만) | Enforce(강제)
#   launch 파일에 security=True 파라미터도 병행 설정 필요
#
# 참고:
#   - 키스토어는 /tmp 에 생성하므로 재부팅 시 소멸 → 영구 보존 필요 시 경로 변경
#   - 노드 추가 시 이 스크립트에 노드명을 추가한 뒤 재실행

set -euo pipefail

# ── 키스토어 경로 설정 ──
KEYSTORE="${1:-/tmp/argos_keystore}"

echo "=== ARGOS SROS2 키스토어 생성 ==="
echo "경로: $KEYSTORE"

# 기존 키스토어 백업 (재실행 안전성)
if [ -d "$KEYSTORE" ]; then
    BACKUP="${KEYSTORE}_backup_$(date +%Y%m%d_%H%M%S)"
    echo "기존 키스토어 백업: $BACKUP"
    mv "$KEYSTORE" "$BACKUP"
fi

# ── 키스토어 초기화 ──
ros2 security create_keystore "$KEYSTORE"
echo "키스토어 초기화 완료: $KEYSTORE"

# ── 노드별 키 생성 ──
# 형식: /네임스페이스/노드명 (launch 파일의 namespace + node_name과 일치해야 함)
NODES=(
    "/orchestrator"
    "/argos1/frontier_explorer"
    "/argos2/frontier_explorer"
    "/argos1/slam_toolbox"
    "/argos2/slam_toolbox"
    "/drone1/drone_controller"
)

echo ""
echo "--- 노드별 인증서 생성 ---"
for node in "${NODES[@]}"; do
    echo "  생성 중: $node"
    ros2 security create_key "$KEYSTORE" "$node"
done

echo ""
echo "=== SROS2 설정 완료 ==="
echo ""
echo "활성화 명령 (터미널에 붙여넣기):"
echo "  export ROS_SECURITY_KEYSTORE=$KEYSTORE"
echo "  export ROS_SECURITY_ENABLE=true"
echo "  export ROS_SECURITY_STRATEGY=Enforce"
echo ""
echo "확인 명령:"
echo "  ls $KEYSTORE"
echo "  ros2 security list_keys $KEYSTORE"
echo ""
echo "주의: launch 파일에 security:=True 인수 추가 필요"
echo "      예) ros2 launch argos_bringup multi_robot.launch.py security:=True"
