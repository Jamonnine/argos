#!/bin/bash
# ARGOS gz_bridge 헬스체크 + 자동 재시작
# ==========================================
# Gazebo headless 모드에서 gz_bridge 간헐적 실패 대응.
# /scan, /clock 토픽 존재 확인 → 미존재 시 gz_bridge 재시작.
#
# 사용법:
#   bash check_gz_bridge_health.sh              # 1회 확인
#   bash check_gz_bridge_health.sh --watch      # 반복 모니터링 (10초 간격)
#   bash check_gz_bridge_health.sh --restart    # 미존재 시 자동 재시작
#
# 근거: 2026-03-16 디버깅에서 slam_toolbox lidar_link 실패의 근본 원인이
#       gz_bridge 간헐적 실패 (/scan, /clock 미브리지)로 확정됨.

set -e

REQUIRED_TOPICS=("/scan" "/clock" "/odom" "/tf")
BRIDGE_CONFIG="$(ros2 pkg prefix argos_description)/share/argos_description/config/gz_bridge.yaml"
RETRY_MAX=3
RETRY_DELAY=5

check_topics() {
    local missing=0
    local topics
    topics=$(ros2 topic list 2>/dev/null)

    for topic in "${REQUIRED_TOPICS[@]}"; do
        if echo "$topics" | grep -q "^${topic}$"; then
            echo "  [OK] $topic"
        else
            echo "  [MISSING] $topic"
            missing=$((missing + 1))
        fi
    done

    return $missing
}

restart_bridge() {
    echo "gz_bridge 재시작 중..."
    pkill -f "parameter_bridge" 2>/dev/null || true
    sleep 2
    ros2 run ros_gz_bridge parameter_bridge --ros-args -p "config_file:=${BRIDGE_CONFIG}" &
    echo "gz_bridge PID: $!"
    sleep 5
}

echo "=== ARGOS gz_bridge Health Check ==="
echo "필수 토픽: ${REQUIRED_TOPICS[*]}"
echo ""

case "${1:-}" in
    --watch)
        while true; do
            echo "--- $(date +%H:%M:%S) ---"
            check_topics || echo "  ⚠ 누락 토픽 있음"
            sleep 10
        done
        ;;
    --restart)
        for attempt in $(seq 1 $RETRY_MAX); do
            echo "확인 #${attempt}/${RETRY_MAX}"
            if check_topics; then
                echo "✅ 모든 필수 토픽 정상"
                exit 0
            else
                echo "⚠ 토픽 누락 — gz_bridge 재시작 (${attempt}/${RETRY_MAX})"
                restart_bridge
                sleep $RETRY_DELAY
            fi
        done
        echo "❌ ${RETRY_MAX}회 재시작 후에도 토픽 누락"
        exit 1
        ;;
    *)
        if check_topics; then
            echo ""
            echo "✅ 모든 필수 토픽 정상"
        else
            echo ""
            echo "⚠ 토픽 누락 — --restart 옵션으로 자동 재시작 가능"
            exit 1
        fi
        ;;
esac
