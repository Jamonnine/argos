# DetectedObject.msg
# AI 객체 감지 결과 단일 항목
#
# 설계 원칙:
# - Header 포함 → frame_id로 좌표계 명시 (camera_frame, base_link, map 등)
# - 2D bbox + 3D pose 둘 다 → 카메라 전용과 3D 공간 모두 지원
# - confidence → 하류 노드가 임계값 필터링 가능

std_msgs/Header header          # stamp + frame_id (어느 좌표계에서 측정했는가)

# AI 인식 결과
string class_name               # "box", "person", "door", "obstacle"
float32 confidence              # 0.0 ~ 1.0 (확신도)
int32 track_id                  # 객체 추적 ID (-1이면 미추적)

# 2D 바운딩 박스 (이미지 픽셀 좌표)
sensor_msgs/RegionOfInterest bbox_2d

# 3D 위치 (header.frame_id 기준 좌표계)
geometry_msgs/Pose pose_3d      # position + orientation
float32 distance                # 로봇에서 객체까지 거리 (미터)
