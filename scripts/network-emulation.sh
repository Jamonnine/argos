#!/bin/bash
# ARGOS 소방 현장 네트워크 에뮬레이션
# 통신 전문가 권고: WiFi 불안정 환경 시뮬레이션
#
# 사용법:
#   ./network-emulation.sh start    # 소방 현장 WiFi 시뮬 시작
#   ./network-emulation.sh stop     # 원복
#   ./network-emulation.sh status   # 현재 상태
#
# 소방 현장 통신 특성:
#   패킷 손실: 5~15%
#   지연: 100~500ms
#   대역폭 변동: ±70%
#   신호 차단: 3~10초 연속

INTERFACE="${ARGOS_NET_INTERFACE:-lo}"  # 기본: loopback (안전)

case "$1" in
    start)
        echo "=== 소방 현장 WiFi 에뮬레이션 시작 ==="
        echo "인터페이스: $INTERFACE"
        echo "지연: 200ms ±50ms, 손실: 10%, 대역폭: 1Mbit"

        sudo tc qdisc add dev $INTERFACE root netem \
            delay 200ms 50ms distribution normal \
            loss 10% \
            rate 1mbit

        echo "✅ 에뮬레이션 활성화"
        echo "   ros2 topic hz /orchestrator/mission_state  # 지연 확인"
        ;;

    stop)
        echo "=== 네트워크 에뮬레이션 중지 ==="
        sudo tc qdisc del dev $INTERFACE root 2>/dev/null
        echo "✅ 원복 완료"
        ;;

    status)
        echo "=== 현재 네트워크 상태 ==="
        tc qdisc show dev $INTERFACE 2>/dev/null
        ;;

    mild)
        echo "=== 약한 WiFi (사무실 수준) ==="
        sudo tc qdisc replace dev $INTERFACE root netem \
            delay 50ms 10ms loss 2%
        echo "✅ 약한 에뮬레이션 (지연 50ms, 손실 2%)"
        ;;

    severe)
        echo "=== 극심한 WiFi (지하 화재) ==="
        sudo tc qdisc replace dev $INTERFACE root netem \
            delay 500ms 200ms distribution normal \
            loss 15% \
            rate 500kbit
        echo "✅ 극심 에뮬레이션 (지연 500ms, 손실 15%)"
        ;;

    *)
        echo "사용법: $0 {start|stop|status|mild|severe}"
        echo ""
        echo "프리셋:"
        echo "  start   소방 현장 WiFi (지연 200ms, 손실 10%)"
        echo "  mild    사무실 WiFi (지연 50ms, 손실 2%)"
        echo "  severe  지하 화재 (지연 500ms, 손실 15%)"
        echo "  stop    원복"
        exit 1
        ;;
esac
