# AudioEvent.msg
# 음향 이벤트 감지 — 폭발, 가스 누출, 구조 요청, 붕괴 소리
#
# 시각 센서의 사각지대를 보완하는 음향 감지 시스템.
# 벽 뒤, 연기 속, 어둠 속에서도 소리로 위험과 생존자를 탐지.

std_msgs/Header header

# 감지 로봇
string robot_id

# 이벤트 분류
string event_type                      # "explosion" | "gas_leak" | "cry_for_help" |
                                       # "collapse" | "glass_break" | "alarm" |
                                       # "breathing" | "unknown"
float32 confidence                     # 0.0 ~ 1.0

# 음향 특성
float32 intensity_db                   # 소리 강도 (dB)
float32 frequency_hz                   # 주파수 대역 (Hz). 가스 누출=고주파, 폭발=저주파
float32 duration_sec                   # 이벤트 지속 시간 (초)

# 방향/위치
float32 direction_rad                  # 감지 방향 (라디안, 로봇 기준). NaN=무지향
geometry_msgs/PointStamped estimated_location  # 추정 위치 (맵 좌표). 부정확할 수 있음

# 위험도
string danger_level                    # "info" | "caution" | "danger" | "critical"
bool immediate_response_needed         # 즉시 대응 필요 여부
