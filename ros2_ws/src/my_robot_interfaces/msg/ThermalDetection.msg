# ThermalDetection.msg
# 열화상 카메라 화점(hotspot) 감지 결과
#
# 소방 현장에서 열화상 카메라가 감지한 고온 영역 정보.
# L8 카메라 기준: 상위 N% 적응형 임계값으로 화점 판별.

std_msgs/Header header              # stamp + frame_id (thermal_camera_optical_link)

# 온도 측정 (L8 → kelvin 역산)
float32 max_temperature_kelvin      # 감지 영역 최고 온도
float32 mean_temperature_kelvin     # 감지 영역 평균 온도

# 감지 위치
sensor_msgs/RegionOfInterest bbox   # 열화상 이미지 내 2D 바운딩 박스
geometry_msgs/Point centroid_image  # 열화상 이미지 내 중심점 (x=col, y=row, z=0)

# 분류
float32 confidence                  # 감지 신뢰도 0.0 ~ 1.0
string severity                     # "low" / "medium" / "high" / "critical"
float32 area_ratio                  # 화점 면적 / 전체 이미지 면적 (0.0~1.0)
