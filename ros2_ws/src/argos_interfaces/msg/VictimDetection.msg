# VictimDetection.msg
# 열화상/RGB 기반 피해자(인체) 감지 결과
#
# 화재 현장 구조 임무의 핵심 데이터.
# 열화상에서 체온(~36°C) 시그니처로 인체를 감지.

std_msgs/Header header

# 감지 로봇
string robot_id

# 감지 위치
geometry_msgs/PointStamped location    # 맵 좌표
sensor_msgs/RegionOfInterest bbox      # 이미지 내 바운딩박스

# 감지 정보
float32 confidence                     # 0.0 ~ 1.0
string detection_source                # "thermal" | "rgb" | "both"
float32 body_temperature_kelvin        # 추정 체온 (열화상 시)

# 상태 추정
string estimated_status                # "moving" | "stationary" | "unknown"
float32 time_stationary_sec            # 움직임 없는 시간 (초). 0=움직이는 중

# 우선순위 (오케스트레이터용)
uint8 rescue_priority                  # 1=최우선(움직임없음) 2=높음 3=보통
