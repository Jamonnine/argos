#!/bin/bash
# ARGOS 데모 녹화 가이드 (S-A4)
# =================================
# OBS Studio로 Gazebo GUI 화면을 녹화하여 GitHub Release + NFRI 발표자료 제작
#
# 사전 준비:
#   1. OBS Studio 설치 (Windows)
#   2. WSL2에서 Gazebo GUI 모드로 ARGOS 실행
#   3. OBS로 Gazebo 창 캡처 → 1분+ 녹화
#
# 시나리오별 녹화 명령어:

echo "=== ARGOS 데모 녹화 시나리오 ==="
echo ""
echo "1. 단일 로봇 자율탐색 (기본)"
echo "   ros2 launch argos_description navigation.launch.py explore:=true"
echo ""
echo "2. 화재 현장 탐색"
echo "   ros2 launch argos_description navigation.launch.py \\"
echo "     world:=\$(ros2 pkg prefix argos_description)/share/argos_description/worlds/fire_building.sdf \\"
echo "     explore:=true"
echo ""
echo "3. Zenoh 통신 (DDS 대비 3배+)"
echo "   ros2 launch argos_description navigation_zenoh.launch.py explore:=true"
echo ""
echo "4. 멀티로봇 (2UGV + 1Drone)"
echo "   ros2 launch argos_description exploration.launch.py"
echo ""
echo "5. 셰르파 소방로봇 (NFRI 제원)"
echo "   # YAML 파라미터로 RSP 실행 필요 — multi_sherpa.launch.py 사용"
echo "   ros2 launch argos_description multi_sherpa.launch.py"
echo ""
echo "=== 녹화 팁 ==="
echo "- OBS: 1920x1080, 30fps, mp4"
echo "- Gazebo GUI에서 Follow 모드로 로봇 추적"
echo "- 최소 1분 이상 풀런 녹화"
echo "- 화재 월드에서 연기+화점+로봇 탐색 장면이 가장 임팩트"
