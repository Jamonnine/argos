# FireAlert.msg
# 화점 감지 시 오케스트레이터에 보고하는 알림 메시지.
# ThermalDetection(이미지 좌표)을 맵 좌표로 변환한 고수준 알림.

std_msgs/Header header

# 감지 로봇
string robot_id

# 화점 위치 (맵 좌표)
geometry_msgs/PointStamped location

# 열화상 정보
float32 max_temperature_kelvin
string severity                         # "low" | "medium" | "high" | "critical"
float32 confidence                      # 0.0 ~ 1.0

# 상태
bool active                             # true=진행 중, false=진화됨/오탐
