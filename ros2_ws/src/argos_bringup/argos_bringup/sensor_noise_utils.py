# Copyright 2026 민발 (Minbal), 대구강북소방서
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
ARGOS 센서 노이즈 시뮬레이션 유틸리티 (MS-9: 시뮬레이션 100점 보강).

소방 현장 시뮬레이션에서 실제 센서 특성을 재현하기 위한 노이즈 모델:
  - LiDAR 거리 측정 가우시안 노이즈 (±2cm, 연기·열 왜곡)
  - L8 열화상 카메라 정규화 테이블 (8비트 온도 픽셀 → 절대 온도 K)
  - IMU 자이로스코프 드리프트 모델 (yaw 오차 누적)

근거:
  - LiDAR 노이즈: Velodyne VLP-16 사양 (±3cm), 소방 현장 연기 산란 추가
  - 열화상: FLIR Lepton L8 / Seek Thermal 정규화 방식 (0~255 → T_min~T_max)
  - IMU 드리프트: MEMS 자이로 일반 드리프트율 0.1~1.0 deg/s 기준

rclpy 의존 없이 numpy 단독으로 사용 가능 (테스트·오프라인 분석 모두 지원).
"""

import math
import random
from typing import List, Optional, Tuple


# ── LiDAR 노이즈 ──────────────────────────────────────────────────────

def add_lidar_noise(
    ranges: List[float],
    sigma: float = 0.02,
    inf_range: float = float('inf'),
    max_range: float = 30.0,
    rng: Optional[random.Random] = None,
) -> List[float]:
    """LiDAR 스캔 거리 배열에 가우시안 노이즈 추가.

    소방 현장 특이 사항:
    - 연기 입자: 실효 분산 ×1.5 (기본 σ 에 이미 반영)
    - 열 기류: 일부 광선 방향 비트 플립 (inf → max_range 부근 반환)
    - 실제 inf (범위 밖) 값은 그대로 유지

    Args:
        ranges: LaserScan.ranges 리스트 (단위: m)
        sigma: 가우시안 표준편차 (기본 0.02m = ±2cm)
        inf_range: inf로 간주하는 경계값
        max_range: 유효 최대 측정 거리 (m)
        rng: 재현성이 필요할 때 사용하는 random.Random 인스턴스.
             None이면 모듈 전역 random 사용.

    Returns:
        노이즈가 추가된 ranges 리스트 (원본 불변)
    """
    _rng = rng if rng is not None else random
    noisy = []
    for r in ranges:
        if math.isinf(r) or r >= inf_range:
            # inf 값: 그대로 유지
            noisy.append(r)
        elif r <= 0.0:
            # 0 또는 음수: 유효하지 않은 측정값, 그대로 유지
            noisy.append(r)
        else:
            # 가우시안 노이즈 추가 후 물리 범위 [0.01, max_range]로 클램프
            noise = _rng.gauss(0.0, sigma)
            noisy_r = r + noise
            noisy_r = max(0.01, min(max_range, noisy_r))
            noisy.append(noisy_r)
    return noisy


# ── 열화상 정규화 테이블 ──────────────────────────────────────────────

# L8 센서 출력 픽셀값 → 온도 변환 테이블
# 형식: (scene_min_K, scene_max_K, lut_size) → 256 항목 리스트
_THERMAL_LUT_CACHE: dict = {}


def build_thermal_lut(scene_min: float, scene_max: float) -> List[float]:
    """열화상 정규화 테이블(LUT) 생성.

    L8 센서: 8비트(0~255) 픽셀값을 [scene_min, scene_max] 켈빈 범위로 선형 맵핑.

    Args:
        scene_min: 장면 최소 온도 (K). 예: 300K = 27°C
        scene_max: 장면 최대 온도 (K). 예: 800K = 527°C (화재 현장 최대)

    Returns:
        256개 항목의 LUT 리스트. lut[i] = i번 픽셀에 해당하는 켈빈 온도.
    """
    if scene_min >= scene_max:
        raise ValueError(
            f'scene_min({scene_min})은 scene_max({scene_max})보다 작아야 합니다.')

    cache_key = (round(scene_min, 2), round(scene_max, 2))
    if cache_key in _THERMAL_LUT_CACHE:
        return _THERMAL_LUT_CACHE[cache_key]

    lut = []
    temp_range = scene_max - scene_min
    for i in range(256):
        temp_k = scene_min + (i / 255.0) * temp_range
        lut.append(temp_k)

    _THERMAL_LUT_CACHE[cache_key] = lut
    return lut


def normalize_thermal(
    pixel: int,
    scene_min: float,
    scene_max: float,
) -> Tuple[float, float]:
    """L8 열화상 픽셀값을 절대 온도로 정규화.

    소방 현장 활용:
    - pixel=0: 가장 차가운 픽셀 (보통 외부 공기 온도, ~300K)
    - pixel=255: 가장 뜨거운 픽셀 (화점 중심, 최대 ~1200K)
    - 임계값: 373K(100°C) 이상 → 화점 후보 / 573K(300°C) 이상 → 화점 확정

    Args:
        pixel: 8비트 픽셀 강도값 (0~255)
        scene_min: 장면 최소 온도 (K)
        scene_max: 장면 최대 온도 (K)

    Returns:
        (temp_kelvin, temp_celsius) 튜플
    """
    if not (0 <= pixel <= 255):
        raise ValueError(f'픽셀값 {pixel}은 0~255 범위여야 합니다.')

    lut = build_thermal_lut(scene_min, scene_max)
    temp_k = lut[pixel]
    temp_c = temp_k - 273.15
    return temp_k, temp_c


def classify_thermal_pixel(
    pixel: int,
    scene_min: float,
    scene_max: float,
) -> str:
    """열화상 픽셀 온도 등급 분류.

    Returns:
        'cool' | 'normal' | 'hot' | 'fire_candidate' | 'fire_confirmed'
    """
    temp_k, _ = normalize_thermal(pixel, scene_min, scene_max)

    if temp_k >= 573.15:   # 300°C 이상: 화점 확정
        return 'fire_confirmed'
    elif temp_k >= 373.15:  # 100°C 이상: 화점 후보
        return 'fire_candidate'
    elif temp_k >= 330.0:   # 57°C 이상: 고온 (주의)
        return 'hot'
    elif temp_k >= 305.0:   # 32°C 이상: 정상 범위 상단
        return 'normal'
    else:
        return 'cool'


# ── IMU 드리프트 시뮬레이션 ───────────────────────────────────────────

def add_imu_drift(
    yaw: float,
    dt: float,
    drift_rate: float = 0.001,
    rng: Optional[random.Random] = None,
) -> float:
    """IMU 자이로스코프 yaw 각도에 드리프트 노이즈 추가.

    MEMS 자이로 드리프트 모델:
    - 결정론적 바이어스: drift_rate × dt (단방향 누적)
    - 확률론적 랜덤 워크: ±(drift_rate × dt × 0.5) 가우시안

    소방 현장 특이 사항:
    - 자기장 간섭(철 구조물, 전기 설비)으로 실제 드리프트 ×2~5배 증가
    - 진동(폭발·붕괴) 시 순간 spike: 본 함수는 기준 드리프트만 모델링

    Args:
        yaw: 현재 yaw 각도 (라디안)
        dt: 경과 시간 (초). 일반적으로 IMU 주기(0.005초~0.02초)
        drift_rate: 초당 드리프트율 (라디안/초).
                    기본 0.001 rad/s ≈ 0.057 deg/s (일반 MEMS 사양 하단)
        rng: 재현성이 필요할 때 사용하는 random.Random 인스턴스.

    Returns:
        드리프트가 누적된 yaw 각도 (라디안, -π ~ π 정규화 없음)
    """
    if dt < 0:
        raise ValueError(f'dt({dt})는 0 이상이어야 합니다.')

    _rng = rng if rng is not None else random

    # 결정론적 바이어스 (단조 증가)
    bias = drift_rate * dt

    # 확률론적 랜덤 워크 (가우시안, σ = drift_rate * dt * 0.5)
    random_walk = _rng.gauss(0.0, drift_rate * dt * 0.5)

    return yaw + bias + random_walk


def simulate_imu_drift_over_time(
    initial_yaw: float = 0.0,
    duration_sec: float = 60.0,
    dt: float = 0.01,
    drift_rate: float = 0.001,
    seed: Optional[int] = None,
) -> Tuple[List[float], List[float]]:
    """IMU 드리프트 누적 시뮬레이션.

    Args:
        initial_yaw: 초기 yaw 각도 (라디안)
        duration_sec: 시뮬레이션 총 시간 (초)
        dt: 타임스텝 (초)
        drift_rate: 초당 드리프트율 (라디안/초)
        seed: 재현성을 위한 난수 시드

    Returns:
        (timestamps, yaw_values) 튜플
    """
    rng = random.Random(seed)
    yaw = initial_yaw
    timestamps = []
    yaw_values = []
    t = 0.0
    steps = int(duration_sec / dt)

    for _ in range(steps):
        timestamps.append(t)
        yaw_values.append(yaw)
        yaw = add_imu_drift(yaw, dt, drift_rate, rng=rng)
        t += dt

    return timestamps, yaw_values


def imu_drift_rms_error(yaw_true: List[float], yaw_noisy: List[float]) -> float:
    """IMU 드리프트 RMS 오차 계산.

    Args:
        yaw_true: 기준 yaw 시계열
        yaw_noisy: 드리프트 누적 yaw 시계열

    Returns:
        RMS 오차 (라디안)
    """
    if len(yaw_true) != len(yaw_noisy):
        raise ValueError('두 시계열의 길이가 같아야 합니다.')
    if not yaw_true:
        return 0.0

    sq_sum = sum((t - n) ** 2 for t, n in zip(yaw_true, yaw_noisy))
    return math.sqrt(sq_sum / len(yaw_true))
