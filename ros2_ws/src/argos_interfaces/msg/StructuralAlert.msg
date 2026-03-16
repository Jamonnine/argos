# StructuralAlert.msg
# 구조물 안전성 경고 — LiDAR 포인트클라우드 변화 기반
#
# 화재 현장에서 바닥 함몰, 천장 붕괴, 벽체 균열 등을 감지.
# LiDAR 스캔을 시간 경과에 따라 비교하여 구조 변화를 탐지.

std_msgs/Header header

# 감지 로봇
string robot_id

# 경고 정보
string alert_type                      # "floor_collapse" | "ceiling_sag" | "wall_crack" | "debris" | "structural_shift"
string severity                        # "warning" | "danger" | "critical"
float32 confidence                     # 0.0 ~ 1.0

# 위치
geometry_msgs/PointStamped location    # 맵 좌표
float32 affected_radius_m              # 영향 반경 (미터)

# 변화량
float32 displacement_m                 # 구조물 변위량 (미터)
float32 change_rate_m_per_s            # 변화 속도 (m/s). 빠를수록 위험

# 대응
bool area_blocked                      # 이 영역 통행 차단 권고
string[] recommended_actions            # ["reroute", "evacuate", "monitor"] 등
