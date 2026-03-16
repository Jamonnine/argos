#!/bin/bash

##############################################################################
# ARGOS 환경 검증 스크립트
# ROS 2 + Gazebo Harmonic + GPU 지원 여부를 종합 점검한다.
# 용도: 개발 환경 초기화 확인, CI 사전 점검, 트러블슈팅
##############################################################################

# 색상 코드
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 결과 집계
PASS=0
WARN=0
FAIL=0

pass() { echo -e "    ${GREEN}[PASS]${NC} $1"; ((PASS++)); }
warn() { echo -e "    ${YELLOW}[WARN]${NC} $1"; ((WARN++)); }
fail() { echo -e "    ${RED}[FAIL]${NC} $1"; ((FAIL++)); }

echo "=========================================================================="
echo "              ARGOS 환경 검증 스크립트 (ROS2 + Gazebo + GPU)"
echo "=========================================================================="
echo ""

# ── [1] OS ─────────────────────────────────────────────────────────────────
echo -e "${BLUE}[1] 운영체제${NC}"
if command -v lsb_release &>/dev/null; then
    OS_DESC=$(lsb_release -d | sed 's/Description:\t//')
    echo "    $OS_DESC"
    if echo "$OS_DESC" | grep -q "Ubuntu 24"; then
        pass "Ubuntu 24.04 (ROS 2 Jazzy 권장)"
    else
        warn "Ubuntu 24.04 권장 — 현재: $OS_DESC"
    fi
else
    warn "lsb_release 없음 — OS 확인 불가"
fi
echo ""

# ── [2] ROS 2 Jazzy ─────────────────────────────────────────────────────────
echo -e "${BLUE}[2] ROS 2 Jazzy${NC}"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [ -f "$ROS_SETUP" ]; then
    # shellcheck source=/dev/null
    source "$ROS_SETUP"
    pass "ROS 2 Jazzy 설치 확인: $ROS_SETUP"
    echo "    ROS_DISTRO=$ROS_DISTRO  ROS_VERSION=$ROS_VERSION"
else
    fail "ROS 2 Jazzy 없음: $ROS_SETUP 미존재"
    echo -e "    설치: ${CYAN}sudo apt install ros-jazzy-desktop${NC}"
fi

if command -v ros2 &>/dev/null; then
    ROS2_VER=$(ros2 --version 2>/dev/null | head -1)
    pass "ros2 명령 사용 가능: $ROS2_VER"
else
    fail "ros2 명령 없음 — PATH 설정 확인"
fi
echo ""

# ── [3] Gazebo Harmonic ──────────────────────────────────────────────────────
echo -e "${BLUE}[3] Gazebo Harmonic${NC}"
if command -v gz &>/dev/null; then
    GZ_VER=$(gz --version 2>/dev/null | head -1)
    pass "gz (Gazebo Harmonic) 사용 가능: $GZ_VER"
elif command -v ign &>/dev/null; then
    IGN_VER=$(ign --version 2>/dev/null | head -1)
    warn "ign (Ignition) 발견: $IGN_VER — Gazebo Harmonic 아닐 수 있음"
else
    fail "Gazebo Harmonic (gz) 없음"
    echo -e "    설치: ${CYAN}sudo apt install gz-harmonic${NC}"
fi

# ros_gz 브릿지 확인
if ros2 pkg list 2>/dev/null | grep -q "^ros_gz"; then
    pass "ros_gz 브릿지 패키지 확인"
else
    warn "ros_gz 없음 — ROS-Gazebo 통신 불가"
    echo -e "    설치: ${CYAN}sudo apt install ros-jazzy-ros-gz${NC}"
fi
echo ""

# ── [4] GPU / CUDA ───────────────────────────────────────────────────────────
echo -e "${BLUE}[4] GPU / CUDA (YOLO 추론용)${NC}"
if command -v nvidia-smi &>/dev/null; then
    GPU_INFO=$(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader 2>/dev/null | head -1)
    if [ -n "$GPU_INFO" ]; then
        pass "NVIDIA GPU 감지: $GPU_INFO"
    else
        warn "nvidia-smi 있으나 GPU 응답 없음"
    fi
else
    warn "NVIDIA GPU 없음 — YOLO 추론은 CPU 폴백 (속도 저하)"
fi

# CUDA 확인
if command -v nvcc &>/dev/null; then
    CUDA_VER=$(nvcc --version | grep "release" | awk '{print $6}' | sed 's/,//')
    pass "CUDA $CUDA_VER 확인"
elif [ -d /usr/local/cuda ]; then
    warn "CUDA 디렉토리 존재하나 nvcc 없음 — PATH 설정 확인"
    echo "    export PATH=\$PATH:/usr/local/cuda/bin"
else
    warn "CUDA 없음 — GPU 가속 미사용"
fi
echo ""

# ── [5] Python ───────────────────────────────────────────────────────────────
echo -e "${BLUE}[5] Python${NC}"
if command -v python3 &>/dev/null; then
    PY_VER=$(python3 --version)
    pass "$PY_VER"
    # ARGOS 필수 Python 패키지 확인
    for pkg in numpy scipy cv2; do
        if python3 -c "import $pkg" 2>/dev/null; then
            pass "python3: $pkg 확인"
        else
            warn "python3: $pkg 없음"
            echo -e "    설치: ${CYAN}pip install $pkg${NC}"
        fi
    done
else
    fail "python3 없음"
fi
echo ""

# ── [6] Colcon ───────────────────────────────────────────────────────────────
echo -e "${BLUE}[6] Colcon 빌드 도구${NC}"
if command -v colcon &>/dev/null; then
    pass "colcon 사용 가능"
else
    fail "colcon 없음"
    echo -e "    설치: ${CYAN}pip install colcon-common-extensions${NC}"
fi
echo ""

# ── [7] rosdep ───────────────────────────────────────────────────────────────
echo -e "${BLUE}[7] rosdep${NC}"
if command -v rosdep &>/dev/null; then
    pass "rosdep 사용 가능"
    if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        pass "rosdep 초기화 완료"
    else
        warn "rosdep 미초기화"
        echo -e "    실행: ${CYAN}sudo rosdep init && rosdep update${NC}"
    fi
else
    fail "rosdep 없음"
    echo -e "    설치: ${CYAN}pip install rosdep && sudo rosdep init${NC}"
fi
echo ""

# ── [8] ARGOS 워크스페이스 ─────────────────────────────────────────────────
echo -e "${BLUE}[8] ARGOS ROS 2 워크스페이스${NC}"
WS_PATH="${HOME}/ros2_ws"
if [ -d "$WS_PATH" ]; then
    pass "워크스페이스 존재: $WS_PATH"
    if [ -d "$WS_PATH/install" ]; then
        pass "빌드 완료 (install/ 디렉토리 확인)"
        if [ -f "$WS_PATH/install/argos_bringup/share/argos_bringup/package.xml" ]; then
            pass "argos_bringup 패키지 설치 확인"
        else
            warn "argos_bringup 미설치 — colcon build 필요"
            echo -e "    실행: ${CYAN}cd ~/ros2_ws && colcon build --symlink-install${NC}"
        fi
    else
        warn "미빌드 상태"
        echo -e "    실행: ${CYAN}cd ~/ros2_ws && colcon build --symlink-install${NC}"
    fi
else
    warn "워크스페이스 없음: $WS_PATH"
    echo "    심볼릭 링크: ln -s /mnt/c/Users/USER/Desktop/SUPERAGENT/projects/argos/ros2_ws ~/ros2_ws"
fi
echo ""

# ── [9] DDS 미들웨어 ──────────────────────────────────────────────────────────
echo -e "${BLUE}[9] DDS 미들웨어${NC}"
RMW="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
echo "    RMW_IMPLEMENTATION: $RMW"
if echo "$RMW" | grep -q "cyclone"; then
    pass "CycloneDDS — 멀티로봇 환경 권장 설정"
elif echo "$RMW" | grep -q "fastrtps"; then
    warn "FastRTPS — 멀티로봇 환경은 CycloneDDS 권장"
    echo -e "    전환: ${CYAN}export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp${NC}"
else
    warn "RMW: $RMW (확인 필요)"
fi
echo ""

# ── [10] ROS 2 동작 확인 ──────────────────────────────────────────────────────
echo -e "${BLUE}[10] ROS 2 동작 확인${NC}"
if timeout 3s ros2 topic list &>/dev/null; then
    pass "ros2 topic list 정상 응답"
else
    warn "ros2 데몬 미시작 (첫 실행 시 정상 — 다음 명령은 정상 동작)"
fi
echo ""

# ── 최종 결과 ──────────────────────────────────────────────────────────────
echo "=========================================================================="
echo -e "  결과 요약:  ${GREEN}PASS ${PASS}${NC}  /  ${YELLOW}WARN ${WARN}${NC}  /  ${RED}FAIL ${FAIL}${NC}"
echo "=========================================================================="

if [ "$FAIL" -gt 0 ]; then
    echo -e "${RED}필수 항목 ${FAIL}개 실패 — 위 항목을 먼저 해결하세요.${NC}"
    exit 1
elif [ "$WARN" -gt 0 ]; then
    echo -e "${YELLOW}선택 항목 ${WARN}개 경고 — 일부 기능 제한될 수 있습니다.${NC}"
    exit 0
else
    echo -e "${GREEN}모든 환경 검증 통과 — ARGOS 실행 준비 완료.${NC}"
    exit 0
fi
