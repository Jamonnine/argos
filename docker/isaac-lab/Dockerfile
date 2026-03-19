# ARGOS Isaac Lab GPU 가속 시뮬레이션 환경
#
# 베이스 이미지: NVIDIA Isaac Sim 4.5.0
# 목적: RTX 4050 로컬 GPU로 100+ 병렬 환경 학습 (Phase E)
#
# 빌드:
#   cd docker/isaac-lab
#   docker build -t argos-isaac-lab:latest .
#
# 사용:
#   docker compose up isaac-lab
#
# 참고:
#   - Isaac Sim 4.5.0 requires NVIDIA driver >= 525.85.12
#   - RTX 4050 기준 권장 병렬 환경 수: 64~128 (VRAM 6GB)
#   - Isaac Lab 0.3+ 부터 gymnasium 호환 인터페이스 공식 지원

FROM nvcr.io/nvidia/isaac-sim:4.5.0

LABEL maintainer="ARGOS Project <Jamonnine>"
LABEL description="ARGOS Phase E — Isaac Lab GPU 병렬 학습 환경 (RTX 4050)"

# 기본 의존성 설치
# python3-pip: Isaac Sim 내장 Python에 패키지 추가
# curl/git: Isaac Lab 설치 및 assets 다운로드
RUN apt-get update -q && \
    apt-get install -y -q \
      git \
      curl \
      wget \
      python3-pip \
      libgl1-mesa-glx \
      libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Isaac Lab 설치
# Isaac Lab은 Isaac Sim 위에 gymnasium 인터페이스를 제공하는 레이어
# 0.3.x 기준 RL Games, RSL-RL, skrl 공식 지원
# 참고: https://isaac-sim.github.io/IsaacLab/
RUN git clone --depth 1 \
      https://github.com/isaac-sim/IsaacLab.git \
      /workspace/IsaacLab \
    && cd /workspace/IsaacLab \
    && ./isaaclab.sh --install

# ARGOS Python 학습 패키지 설치
# gymnasium: MARL 환경 인터페이스
# stable-baselines3: PPO/SAC 기본 알고리즘 (Level 1~2)
# ray[rllib]: 멀티에이전트 학습 (Level 3)
# numpy, torch: 수치 연산 (Isaac Sim 내 torch는 CUDA 지원 포함)
COPY requirements-training.txt /tmp/requirements-training.txt
RUN pip install --no-cache-dir -r /tmp/requirements-training.txt

# ARGOS 학습 코드 마운트 포인트
# (docker-compose.yml volumes로 host 디렉토리 연결)
WORKDIR /workspace

# GPU 가속 렌더링을 위한 환경변수
# DISPLAY 없는 headless 모드로 Isaac Sim 실행
ENV DISPLAY=""
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV OMNI_KIT_ALLOW_ROOT=1

# Isaac Sim headless 실행 설정
# --no-window: GUI 비활성화 (학습 서버용)
# --allow-root: Docker 내 root 실행 허용
ENV ISAACSIM_PYTHON_EXE="/isaac-sim/python.sh"

# 학습 실행 기본 명령
# 직접 실행 시 override: docker run ... python training/isaac_argos.py
CMD ["bash", "-c", "echo 'ARGOS Isaac Lab 환경 준비 완료. training/isaac_argos.py를 실행하세요.'"]
