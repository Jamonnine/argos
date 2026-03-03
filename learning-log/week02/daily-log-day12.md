# Day 12: Vision-MoveIt 통합 (Perception → Planning → Execution)

**날짜**: 2026-02-18
**학습 주제**: Vision-MoveIt Bridge 구현 — 감지된 물체를 팔이 집으러 가도록 연결
**소요 시간**: 약 2시간

---

## 📋 학습 목표 및 성과

### 목표
- [x] Perception-Planning-Execution (PPE) 루프 아키텍처 이해
- [x] TF2 좌표계 변환 원리 이해 및 적용
- [x] Vision-to-MoveIt Bridge 노드 구현
- [x] Simulated Arm Controller (상태 머신) 구현
- [x] 전체 파이프라인 RViz2 시각화 확인

### 성과
- ✅ `/cluster_markers` → TF2 변환 → `/arm_target_pose` 브릿지 완성
- ✅ IDLE → PLANNING → MOVING → REACHED 상태 머신 구현
- ✅ 물체 감지 시 자동으로 파지(grasp) 목표 좌표 계산
- ✅ RViz2에서 황금색 구(파지 목표) + 색상 변하는 구(팔 상태) 시각화

---

## 🎯 핵심 개념

### 1. Perception-Planning-Execution (PPE) Loop

로봇 자율 시스템의 근본 구조입니다. Sense-Think-Act Cycle이라고도 부릅니다.

```
[Perception]  →  [Bridge]  →  [Planning]  →  [Execution]
/cluster_markers → 좌표변환 → /arm_target_pose → 팔 이동
   (보기)          (생각)         (계획)           (행동)
```

각 단계가 분리된 이유는 관심사 분리(Separation of Concerns) 원칙 때문입니다.
카메라 교체, 팔 교체가 발생해도 Bridge의 표준 인터페이스(`/arm_target_pose`)는 그대로 유지됩니다.

---

### 2. TF2 좌표계 변환

카메라는 `camera_frame` 기준, 팔은 `base_link` 기준으로 좌표를 이야기합니다.
TF2가 두 좌표계 사이의 변환 관계를 실시간으로 관리합니다.

```python
# camera_frame 기준 좌표 → base_link 기준 좌표로 변환
target_in_base = self.tf_buffer.transform(target_pose, 'base_link')
```

**왜 TF2가 필요한가**: 로봇 시스템에는 수십 개의 좌표계가 존재합니다 (바퀴, 몸통, 카메라, 각 관절...). 이 모든 좌표계의 관계를 수동으로 관리하면 코드가 폭발적으로 복잡해집니다. TF2는 이 좌표계 관계를 자동으로 추적하고 계산해줍니다.

---

### 3. MoveIt 아키텍처 (시뮬레이션으로 학습)

실제 MoveIt의 내부 구조를 흉내낸 시뮬레이터를 구현하여 아키텍처를 이해했습니다.

```
MoveGroupInterface (실제)    arm_controller_sim.py (시뮬)
├── 역기구학 (IK 계산)        ├── PLANNING 상태 (0.8초 대기)
├── OMPL 경로 계획            ├── MOVING 상태 (Smooth Step 보간)
├── 충돌 감지                 └── REACHED 상태 (목표 도착)
└── 궤적 실행
```

**Smooth Step 보간**: `t_smooth = t² × (3 - 2t)` 공식으로 시작과 끝에서 부드럽게 감속합니다. 일반 선형 보간보다 자연스러운 팔 움직임을 만듭니다.

---

### 4. 파지 자세(Grasp Pose) 설계

물체의 중심 좌표로 바로 이동하면 팔이 물체와 충돌합니다. 물체 위 10cm 지점으로 접근해야 합니다.

```python
GRASP_APPROACH_OFFSET_Z = 0.10  # 물체 위 10cm
target.pose.position.z = obj_z + GRASP_APPROACH_OFFSET_Z

# 그리퍼가 아래를 향하도록 quaternion 설정
# (0, 0.707, 0, 0.707) = Y축 90도 회전 = 그리퍼 아래 방향
target.pose.orientation.y = 0.707
target.pose.orientation.w = 0.707
```

**쿼터니언(Quaternion)**: 3D 회전을 4개 숫자 (x, y, z, w)로 표현하는 방법. Euler 각도(Roll, Pitch, Yaw)의 짐벌락(Gimbal Lock) 문제를 해결한 수학적 표현입니다.

---

## 🏗️ 생성된 파일

```
ros2_ws/src/my_robot_bringup/scripts/
├── vision_moveit_bridge.py    (신규 - Bridge 노드)
└── arm_controller_sim.py      (신규 - 팔 시뮬레이터)

ros2_ws/src/my_robot_bringup/launch/
└── rgbd_pointcloud.launch.py  (수정 - 2개 노드 추가)

ros2_ws/src/my_robot_bringup/config/
└── pointcloud_pipeline.rviz   (수정 - 황금색 구 + 팔 마커 추가)
```

---

## 📊 전체 파이프라인 완성본

| 단계 | 노드 | 입력 | 출력 |
|------|------|------|------|
| 카메라 | rgbd_camera_simulator | - | /camera/depth + /camera/image_raw |
| 변환 | depth_to_pointcloud | depth image | /camera/points (307,200pt) |
| 다운샘플 | voxel_grid_downsampler | 307,200pt | /points_downsampled (8,224pt) |
| 노이즈 제거 | statistical_outlier_removal | 8,224pt | /points_filtered (7,962pt) |
| 평면 분리 | plane_segmentation | 7,962pt | /points_obstacles (22pt) |
| 클러스터링 | euclidean_clustering | 22pt | /cluster_markers |
| **브릿지** | **vision_moveit_bridge** | **/cluster_markers** | **/arm_target_pose** |
| **팔 제어** | **arm_controller_sim** | **/arm_target_pose** | **/arm_visualization** |

---

## 🚀 다음 단계

### Day 13: 실제 MoveIt2 연결 또는 Gazebo 로봇 통합

**옵션 A**: 실제 MoveIt2 설치 + Universal Robots (UR5) URDF로 실제 팔 시뮬레이션
**옵션 B**: Gazebo에서 이동 로봇(TurtleBot3) + 카메라 센서 통합

---

## 🎉 Day 12 완료!

오늘의 핵심 메시지:

"Bridge 노드는 단순한 중간 전달자가 아닙니다. 서로 다른 언어를 쓰는 두 시스템(카메라의 픽셀 좌표와 팔의 관절 공간)이 대화할 수 있도록 번역하는 외교관입니다. 좋은 시스템 설계에서 이 외교관의 역할이 명확하게 정의되어 있을수록, 시스템 전체의 유지보수성과 확장성이 높아집니다."
