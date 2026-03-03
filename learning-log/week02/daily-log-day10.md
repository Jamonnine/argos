# Day 10: 깊이 카메라와 3D 인식 심화

**날짜**: 2026-02-15 ~ 2026-02-16
**학습 주제**: RGB-D 카메라, Point Cloud, TF 시스템, Unprojection
**소요 시간**: 약 6시간 (분산 학습)

---

## 📋 학습 목표 및 성과

### 목표
- [x] 깊이 카메라(RGB-D)의 원리 이해
- [x] Point Cloud 개념과 PointCloud2 메시지 구조
- [x] TF2 시스템 심층 이해
- [x] Unprojection 알고리즘 구현
- [x] 시뮬레이션 RGB-D 카메라 노드 작성
- [x] Depth → Point Cloud 변환 노드 작성
- [x] Launch 파일로 시스템 통합

### 성과
- ✅ Stereo Vision vs Active Sensing 트레이드오프 분석
- ✅ Camera Intrinsics (fx, fy, cx, cy) 물리적 의미 완전 이해
- ✅ Quaternion vs Euler Angles 비교
- ✅ NumPy 벡터화로 100배 성능 향상
- ✅ PointCloud2 바이너리 구조 이해
- ✅ TF를 "관점 전환"으로 철학적 이해

---

## 🎯 핵심 개념

### 1. 깊이 카메라의 본질

**깊이 모호성(Depth Ambiguity) 문제**:

일반 RGB 카메라는 3D → 2D 투영(Projection)을 수행합니다. 이 과정에서 **Z(깊이) 정보가 손실**됩니다. 같은 픽셀 좌표에 무한히 많은 3D 점들이 대응될 수 있습니다. 카메라 렌즈에서 출발하는 광선(Ray) 위의 모든 점이 같은 픽셀에 찍히니까요.

**Pinhole Camera Model (핀홀 카메라 모델)**:

```
u = fx * (X / Z) + cx
v = fy * (Y / Z) + cy
```

핵심은 **Z로 나눈다는 것**입니다. 같은 X 좌표라도, Z가 2배 멀면 u는 절반만 이동합니다. 이게 바로 **원근법(Perspective)**의 수학적 표현입니다.

**RGB-D 카메라의 해결책**:

손실된 Z 정보를 **다시 측정**합니다:
- **Stereo Vision**: 두 카메라 이미지의 차이(Disparity, 시차)로 거리 계산
- **Structured Light**: 적외선 패턴을 투사하고 왜곡 분석
- **Time-of-Flight (ToF)**: 빛의 왕복 시간 측정

---

### 2. 깊이 측정 방법 비교

#### Stereo Vision (스테레오 비전)

**원리**: 인간의 두 눈처럼, 두 카메라 사이의 시차를 이용합니다.

**Disparity (시차)**:
- 왼쪽 이미지의 점 P가 픽셀 (u_L, v)에 나타남
- 오른쪽 이미지의 같은 점 P가 픽셀 (u_R, v)에 나타남
- Disparity = u_L - u_R

**Depth 계산**:
```
Depth = (Baseline × Focal Length) / Disparity
```

- **Baseline**: 두 카메라 사이의 물리적 거리 (예: 10cm)
- **Focal Length**: 카메라 초점 거리 (예: 600픽셀)
- **Disparity**: 픽셀 위치 차이 (예: 50픽셀)

**트레이드오프**:

**장점**:
- **Passive(수동적)**: 조명 발사 불필요, 태양광 사용 가능
- **실외 적합**: 햇빛에 강함
- **텍스처 풍부한 환경**: 벽돌, 나무 등 잘 인식

**단점**:
- **Correspondence Problem(대응 문제)**: 왼쪽 점이 오른쪽 어디에 있는지 찾기 매우 어려움
  - 특히 무늬 없는 흰 벽: 모든 픽셀이 비슷해서 매칭 불가능
- **계산 비용 높음**: 모든 픽셀에 대해 대응점 검색 → GPU 필수
- **Baseline 제약**: 거리가 클수록 정확하지만 장치가 커짐

**실무 사례**:
- ZED Camera: 120mm Baseline, 실외 자율주행 로봇
- Intel RealSense D435: Stereo + IR Projector 하이브리드
- Tesla Autopilot: 멀티 카메라 Stereo Matching

---

#### Structured Light (구조광)

**원리**: 알려진 패턴(점무늬, 격자 등)을 적외선으로 투사하고, 표면에서의 왜곡을 분석합니다.

**동작**:
1. IR Projector가 패턴 투사 (예: 9×9 점 격자)
2. IR Camera가 반사된 패턴 촬영
3. 패턴 왜곡 정도로 깊이 계산

**트레이드오프**:

**장점**:
- **텍스처 독립적**: 흰 벽도 인식 가능 (자체 패턴 생성)
- **실내 최적**: 조명 일정한 환경에서 매우 정확
- **단거리 고정밀**: 0.5~5m 범위에서 mm 단위 정확도

**단점**:
- **햇빛에 약함**: 태양광의 적외선이 패턴을 덮어버림 → 실외 불가
- **짧은 범위**: 보통 5m 이내만 측정
- **간섭**: 같은 공간에 두 대의 Structured Light 카메라 사용 불가 (패턴 겹침)

**실무 사례**:
- Xbox Kinect v1: 실내 게임 센서
- Intel RealSense SR300: 노트북 얼굴 인식

---

#### Time-of-Flight (ToF, 비행 시간)

**원리**: 레이저 펄스를 발사하고, 반사되어 돌아오는 시간을 측정합니다.

**Distance 계산**:
```
Distance = (Speed of Light × Time) / 2
```

- **Speed of Light**: 299,792,458 m/s (광속)
- **Time**: 왕복 시간 (예: 6.7 나노초)
- **/ 2**: 왕복이므로 절반이 편도 거리

**트레이드오프**:

**장점**:
- **빠름**: 전체 프레임을 동시에 측정 (Flash LiDAR)
- **장거리**: 수십~수백 미터 (차량용 LiDAR는 100m+)
- **단순 계산**: Correspondence Problem 없음, 시간만 측정

**단점**:
- **저해상도**: 일반적으로 320×240 (RGB는 1920×1080인데 비해)
- **반사 표면 약함**: 거울, 유리, 검은색 표면에서 오류
- **비용 높음**: 고정밀 ToF는 매우 비쌈 (수백만 원)

**실무 사례**:
- Microsoft Kinect v2 (Azure Kinect): 실내 3D 스캔
- iPhone/iPad LiDAR: AR 앱용
- Velodyne, Ouster: 자율주행차 LiDAR

---

### 3. Camera Intrinsics 심층 이해

**Intrinsic Matrix**:
```
K = [fx,  0, cx]
    [ 0, fy, cy]
    [ 0,  0,  1]
```

#### fx, fy (Focal Length, 초점 거리)

**물리적 의미**: 렌즈에서 이미지 센서까지의 거리를 픽셀 단위로 환산한 값.

**계산**:
```
fx = f_mm / pixel_size_mm
```

**예시**:
- 렌즈 초점 거리: 4mm
- 센서 픽셀 크기: 0.0064mm (6.4 마이크로미터)
- fx = 4 / 0.0064 = 625 픽셀

**FOV (Field of View, 시야각)와의 관계**:
```
FOV = 2 × arctan(width / (2 × fx))
```

- **fx 크다 (예: 1000)**: 망원 렌즈, FOV 좁음 (30도), "줌인"
- **fx 작다 (예: 300)**: 광각 렌즈, FOV 넓음 (90도), "전체를 한눈에"

**실제 카메라 비교**:
- 스마트폰: fx ≈ 400~600
- 망원 렌즈 (새 사진): fx ≈ 1500+
- 어안 렌즈: fx ≈ 200 (180도 FOV)

**왜 fx ≠ fy일 수 있나?**

이상적으로는 같아야 하지만, 센서 픽셀이 완벽한 정사각형이 아니면 다를 수 있습니다. 제조 공정에서 픽셀이 약간 직사각형이면, X축과 Y축의 "픽셀당 mm"가 달라집니다. 현대 카메라는 대부분 fx ≈ fy입니다.

---

#### cx, cy (Principal Point, 주점)

**물리적 의미**: 광축(Optical Axis)이 이미지 평면과 만나는 점.

광축은 "렌즈 중심에서 직각으로 나가는 선"입니다. 이상적으로는 **이미지 정중앙**이어야 합니다:
- cx = width / 2 = 320 (640픽셀 이미지)
- cy = height / 2 = 240 (480픽셀 이미지)

**제조 오차**:

하지만 실제로는 렌즈와 센서 정렬이 완벽하지 않아서 약간 틀어질 수 있습니다:
- cx = 318.7 (정중앙에서 1.3픽셀 왼쪽)
- cy = 241.2 (정중앙에서 1.2픽셀 아래)

이 오차가 작아 보이지만, Unprojection에서 누적되면 Point Cloud가 왜곡됩니다. 5미터 거리에서 수 cm 오차가 생길 수 있습니다.

**캘리브레이션의 중요성**:

그래서 **Camera Calibration(카메라 캘리브레이션)**이 필수입니다:
1. 체스보드 패턴을 여러 각도에서 촬영
2. OpenCV `calibrateCamera()` 함수로 계산
3. 정확한 fx, fy, cx, cy, 왜곡 계수 획득

캘리브레이션 없이 사용하면, 로봇 팔이 엉뚱한 곳을 집습니다!

---

### 4. TF (Transform): 관점의 전환

#### 좌표계 = 관점 (Philosophical Understanding)

**좌표계(Frame)**는 "누구의 관점에서 세상을 보는가?"입니다.

**base_link 관점** (로봇의 눈):
- "나는 로봇이다"
- "내 몸통 중심이 원점이다"
- "앞쪽이 +X, 왼쪽이 +Y, 위쪽이 +Z"
- "빨간 공이 내 앞으로 1미터에 있다"

**camera_frame 관점** (카메라의 눈):
- "나는 카메라다"
- "내 렌즈 위치가 원점이다"
- "카메라가 보는 방향이 +Z, 오른쪽이 +X, 아래쪽이 +Y"
- "빨간 공이 내 앞으로 0.9미터에 있다"

#### Transform의 의미

**Transform**: A 좌표계에서 B 좌표계로 가는 "방법"입니다.

두 가지 정보:
1. **Translation (위치 이동)**: "X축으로 0.1m, Y축으로 0m, Z축으로 0.3m 이동"
2. **Rotation (회전)**: "X축 기준 0도, Y축 기준 -30도, Z축 기준 0도 회전"

**수학적 표현 - Homogeneous Transformation Matrix (4×4 동차 변환 행렬)**:

```
T = [R | t]   =  [r11  r12  r13 | tx]
    [0 | 1]      [r21  r22  r23 | ty]
                 [r31  r32  r33 | tz]
                 [ 0    0    0  | 1 ]
```

- **R (3×3)**: Rotation Matrix (회전 행렬)
- **t (3×1)**: Translation Vector (위치 벡터)

**왜 4×4인가?**

3D 공간이니 3×3이면 될 것 같은데, 왜 4×4일까요? 이건 **위치 이동과 회전을 하나의 행렬 곱셈으로 표현**하기 위한 수학적 트릭입니다.

3×3 행렬은 회전만 표현합니다:
```
P' = R × P
```

위치 이동을 포함하려면 별도로 더해야 합니다:
```
P' = R × P + t  (행렬 곱셈 + 벡터 덧셈)
```

이렇게 두 단계로 나뉘면, 여러 Transform을 연속으로 적용할 때 복잡해집니다.

하지만 4×4 동차 행렬을 쓰면:
```
P' = T × P  (행렬 곱셈 한 번!)
```

그리고 여러 Transform을 적용할 때도:
```
T_total = T1 × T2 × T3  (행렬 곱셈만!)
```

이게 바로 컴퓨터 그래픽스와 로봇공학에서 동차 좌표계를 쓰는 이유입니다. **연산을 통일**할 수 있으니까요.

---

#### Quaternion: 회전의 현명한 표현

**회전을 표현하는 방법**:
1. **Euler Angles**: Roll, Pitch, Yaw (직관적)
2. **Rotation Matrix**: 3×3 행렬 (수학적)
3. **Quaternion**: 4개 숫자 (w, x, y, z) (안전)

**ROS 2는 Quaternion을 표준으로 사용합니다.** 왜일까요?

#### Euler Angles의 치명적 문제: Gimbal Lock

**Gimbal Lock (짐벌 락)**:

특정 각도에서 **한 축의 자유도가 사라지는 현상**입니다.

**예시**:
- Pitch를 90도로 하면
- Roll과 Yaw가 같은 축이 되어버림
- 갑자기 한 방향으로 회전 불가능

**역사적 사례**:

아폴로 11호가 달 착륙선을 조종할 때 Gimbal Lock 문제를 겪었습니다. 특정 자세에서 우주선이 제어 불능 상태가 될 뻔했죠. 그 이후로 항공우주 산업에서는 **Quaternion을 표준**으로 씁니다.

#### Quaternion의 장점

1. **Gimbal Lock 없음**: 수학적으로 증명됨
2. **보간 쉬움**: "A 자세에서 B 자세로 부드럽게 전환" → SLERP
3. **메모리 효율**: 4개 vs 9개 (Rotation Matrix)
4. **계산 효율**: 회전 합성이 빠름

**SLERP (Spherical Linear Interpolation, 구면 선형 보간)**:

두 Quaternion 사이를 **구의 표면을 따라** 최단 거리로 보간합니다. 일반 선형 보간(LERP)과 달리, 회전 속도가 일정합니다.

**로봇 팔 예시**:
- 현재 자세: q1
- 목표 자세: q2
- 중간 자세 (50%): SLERP(q1, q2, 0.5)
- → 부드러운 움직임, 속도 일정!

#### Quaternion의 단점

1. **직관성 낮음**: (w=0.7, x=0, y=0.7, z=0)을 보고 어느 방향인지 바로 알기 어려움
2. **정규화 필요**: 항상 `w² + x² + y² + z² = 1`을 유지해야 함

**실무 워크플로우**:
- URDF 파일: Euler Angles로 작성 (사람이 읽기 쉬움)
- ROS 2 메시지: Quaternion 사용 (안전)
- TF 라이브러리: 자동 변환 제공

---

### 5. Point Cloud: 3D 세계의 디지털 표현

#### Point Cloud란?

**3D 공간에 떠 있는 점들의 집합**입니다. 각 점은:
- **(X, Y, Z)**: 3D 좌표
- **(R, G, B)**: 색상 (선택)
- **(Nx, Ny, Nz)**: 법선 벡터 (선택)
- **Intensity**: 반사 강도 (선택)

**비유**: 밤하늘의 별자리. 별들이 흩어져 있듯이, 3D 점들이 공간에 흩어져 있고, 충분히 밀집되면 물체 표면처럼 보입니다.

#### 왜 Point Cloud인가?

**왜 Mesh(삼각형 메쉬)가 아닌 점들의 집합인가?**

**센서 데이터의 본질**:

깊이 카메라는 **샘플링(Sampling) 장치**입니다. 640×480 해상도면, **307,200개의 독립적인 거리 측정**을 수행합니다. 이 측정값들은 서로 연결되어 있지 않습니다.

**부분 관측 (Partial Observation)**:

로봇은 물체의 한쪽 면만 봅니다. 컵을 정면에서 보면 뒷면은 안 보이죠. Mesh는 닫힌 표면을 가정하지만, Point Cloud는 **"내가 본 부분만 표현"**합니다.

#### Organized vs Unorganized Point Cloud

**Organized (구조화)**:
- 2D 그리드 구조 유지: height × width
- RGB-D 카메라 직접 출력
- 이웃 점 찾기 O(1) (인덱스 ±1)
- 법선 계산 빠름

**Unorganized (비구조화)**:
- 순서 없는 1D 배열: width = N, height = 1
- LiDAR, 여러 소스 병합
- 메모리 효율 (빈 공간 저장 안 함)
- 이웃 검색 O(log N) (kd-tree 필요)

**트레이드오프**:

| 측면 | Organized | Unorganized |
|------|-----------|-------------|
| 이웃 검색 | O(1) 초고속 | O(log N) kd-tree |
| 메모리 | 빈 공간도 저장 | 실제 점만 저장 |
| 법선 계산 | 빠름 | 느림 |
| Downsampling | 구조 깨짐 | 영향 없음 |

**설계 결정**: 가능한 한 Organized 형태를 유지하세요. 알고리즘이 10~100배 빠릅니다. 하지만 여러 센서 병합이나 Voxel Downsampling을 거치면 Unorganized로 변환될 수밖에 없습니다.

---

### 6. PointCloud2 메시지: 성능 vs 가독성

#### 설계 철학

PointCloud2는 **성능을 위해 바이너리 형식**을 씁니다.

**두 가지 설계 방향**:

**방향 A (가독성)**:
```python
PointCloud:
  Point[] points:
    - float x, y, z
    - uint8 r, g, b
```

장점: 읽기 쉬움, 타입 안전
단점: 메모리 정렬 낭비, 직렬화 비용 높음

**방향 B (성능, ROS 2 선택)**:
```python
data: uint8[]  # Flat byte array
fields: PointField[]  # 해석 방법
```

장점:
- **Zero-copy 가능**: GPU로 직접 전송
- **압축 효율**: 전체 배열을 한 번에 압축
- **네트워크 효율**: 오버헤드 최소화

단점:
- 가독성 낮음
- 타입 안전성 낮음

**트레이드오프**: ROS 2는 "실시간 로봇 시스템"이므로 **성능 선택**. 30Hz Point Cloud 스트리밍에서 매 프레임 10ms 절약은 전체 파이프라인의 33% 시간 절약!

---

### 7. NumPy 벡터화: 100배 성능 향상

#### 순진한 구현 (Naive)

```python
# ❌ 느림: Python for loop
points = []
for v in range(480):
    for u in range(640):
        d = depth[v, u]
        if d > 0:
            x = (u - cx) * d / fx
            y = (v - cy) * d / fy
            z = d
            points.append((x, y, z))
```

문제:
- **307,200번 Python 반복문** (640×480)
- 각 반복마다 타입 체크, 메모리 할당
- **30 FPS면? 초당 920만 번 반복!**

#### 벡터화 구현

```python
# ✅ 빠름: NumPy 벡터화
u, v = np.meshgrid(np.arange(width), np.arange(height))
x = (u - cx) * depth / fx  # 한 줄에 307,200개 계산!
y = (v - cy) * depth / fy
z = depth
```

**NumPy 벡터화의 비밀**:
- **C/Fortran 코드**로 실행 (Python 인터프리터 건너뜀)
- **SIMD**: CPU가 한 번에 여러 데이터 처리 (AVX-512: 16개 float 동시 곱셈)
- **메모리 접근 최적화**: 연속 메모리 블록 읽기 (캐시 효율 ↑)

**성능 차이**:
- For loop: ~500ms
- 벡터화: ~5ms
- **100배 빠름!**

---

## 🏗️ 시스템 아키텍처

### 전체 데이터 흐름

```
[시뮬레이션 RGB-D 카메라]
  - generate_rgb_image(): 3D 장면을 2D로 투영
  - generate_depth_image(): 각 픽셀의 거리 계산
  - generate_camera_info(): Intrinsics 발행
  - broadcast_static_tf(): base_link → camera_frame
      ↓
  발행 토픽:
  - /camera/image_raw (RGB)
  - /camera/depth/image_raw (Depth)
  - /camera/camera_info (Intrinsics)
      ↓
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  (ROS 2 DDS: 프로세스 간 통신)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
      ↓
[Depth to PointCloud 변환기]
  1. ROS Image → OpenCV Mat (cv_bridge)
  2. Unprojection (NumPy 벡터화)
      x = (u - cx) * depth / fx
      y = (v - cy) * depth / fy
      z = depth
  3. RGB 색상 추가
  4. PointCloud2 메시지 생성 (flat byte array)
  5. 발행: /camera/points
      ↓
[RViz2 또는 다른 노드]
  - 3D 시각화
  - Point Cloud Processing (PCL)
  - 객체 인식
  - 3D SLAM
```

---

## 🔑 핵심 설계 원칙

### 1. 센서 추상화 (Sensor Abstraction)

시뮬레이션 카메라든 실제 카메라든, **같은 ROS 2 토픽**을 발행합니다. Downstream 노드(`depth_to_pointcloud`)는 센서 종류를 모릅니다. 그냥 토픽을 구독할 뿐이죠.

**Dependency Inversion Principle**: 구체적인 하드웨어(RealSense)가 아닌, 추상적인 인터페이스(ROS Topic)에 의존합니다.

**테스트 전략**: 실제 카메라 없이도 개발/테스트 가능!

---

### 2. 모듈화 (Modularity)

카메라 위치를 바꾸면? (예: 로봇 팔 끝으로 이동)
- 카메라 드라이버 코드: **한 글자도 수정 불필요**
- TF 정의만 변경: `base_link → camera_frame (0.1, 0, 0.3)` → `arm_end → camera_frame (0.05, 0, 0)`
- 알고리즘 코드: **수정 불필요** (TF가 자동 변환)

---

### 3. 성능 최적화

- **NumPy 벡터화**: for loop 제거 → 100배 향상
- **Flat Byte Array**: Zero-copy GPU 전송 가능
- **RGB 패킹**: 3 bytes → 4 bytes (메모리 정렬)
- **QoS 튜닝**: 오래된 프레임 버리기 (Latency ↓)

---

## 🛠️ 실습 코드 핵심

### Unprojection (핵심 알고리즘)

```python
# 픽셀 그리드 생성 (벡터화!)
u, v = np.meshgrid(np.arange(width), np.arange(height), indexing='xy')

# Unprojection 공식
x = (u - self.cx) * depth_image / self.fx
y = (v - self.cy) * depth_image / self.fy
z = depth_image

# 유효한 점만 필터링
valid_mask = (z > 0.0) & (z < 10.0) & np.isfinite(z)
x = x[valid_mask]
y = y[valid_mask]
z = z[valid_mask]
```

**설계 포인트**:
- **meshgrid**: 2D 그리드를 한 번에 생성, for loop 불필요
- **Broadcasting**: NumPy가 자동으로 배열 크기 맞춤
- **Boolean Masking**: 조건 만족하는 점만 선택, 매우 효율적

---

### RGB 패킹 (Bit Packing)

```python
# RGB (3 bytes) → uint32 (4 bytes)
r = rgb[:, 2].astype(np.uint32)  # BGR 순서!
g = rgb[:, 1].astype(np.uint32)
b = rgb[:, 0].astype(np.uint32)

rgb_packed = (r << 16) | (g << 8) | b
# 예: R=255, G=128, B=64 → 0x00FF8040
```

**왜 패킹?**
- Point Cloud Library (PCL, Open3D)는 RGB를 하나의 float로 저장하는 관습
- GPU 셰이더 처리 편함 (모든 점 데이터가 float)
- **Type Punning**: 정수를 float처럼 취급 (`reinterpret_cast`)

---

## 📊 학습 성과 자가 평가

| 평가 항목 | 점수 (1-5) | 비고 |
|----------|-----------|------|
| RGB-D 카메라 원리 | 5/5 | Stereo, Structured Light, ToF 트레이드오프 완전 이해 |
| Camera Intrinsics | 5/5 | fx, fy, cx, cy 물리적 의미 및 FOV 관계 명확 |
| TF 시스템 | 5/5 | "관점 전환"으로 철학적 이해, Transform Chain |
| Quaternion vs Euler | 5/5 | Gimbal Lock, SLERP, 실무 워크플로우 |
| Point Cloud 구조 | 5/5 | Organized vs Unorganized, PointCloud2 바이너리 |
| Unprojection 구현 | 5/5 | NumPy 벡터화, 100배 성능 향상 |
| 센서 추상화 설계 | 5/5 | DIP, 모듈화, 테스트 전략 |

**종합 평가**: 5.0/5 - **Excellent**

Day 10은 "깊이 카메라의 모든 것"을 다뤘습니다. 단순히 사용법이 아니라, **왜 이렇게 설계되었는가**까지 깊이 이해했습니다. 특히 투영/역투영의 수학적/물리적 배경, TF의 철학적 의미, 성능 최적화 기법까지 시니어 레벨의 통찰을 얻었습니다.

---

## 🚀 다음 단계

### Day 11: Point Cloud Processing (PCL)

**목표**:
- Point Cloud Library (PCL) 기초
- Downsampling (Voxel Grid)
- Outlier Removal
- Segmentation (Plane, Cluster)
- Normal Estimation

**최종 목표**: Point Cloud에서 물체를 분리하고, 3D Bounding Box 계산 → MoveIt으로 집기

---

### Week 2 로드맵

**Day 8 (완료)**: MoveIt 2 기초 ✅
**Day 9 (완료)**: 컴퓨터 비전 + ROS 2 ✅
**Day 10 (완료)**: RGB-D 카메라, Point Cloud ✅
**Day 11-12 (다음)**: Point Cloud Processing + Vision-MoveIt 통합
**Day 13-14**: 통합 프로젝트 (Navigation + Manipulation + Vision)

---

## 🎉 Day 10 완료!

**오늘의 핵심 메시지**:

"깊이 카메라는 로봇에게 '3D 세계를 이해하는 능력'을 줍니다. 2D 이미지는 '본다(See)'는 것이고, Depth는 '재다(Measure)'는 것입니다. Point Cloud는 이 둘을 합쳐서 '이해한다(Understand)'로 승화시킵니다. 그리고 TF는 이 모든 정보를 '로봇의 관점'으로 통합합니다."

**다음**: Day 11 - Point Cloud에서 의미 추출하기 (Processing) 🔍

---

> "The real voyage of discovery consists not in seeking new landscapes, but in having new eyes."
> (진정한 발견의 여행은 새로운 풍경을 찾는 것이 아니라, 새로운 눈을 가지는 것이다) - Marcel Proust

로봇도 이제 "새로운 눈" (3D Vision)을 가졌습니다! 🤖👁️✨
