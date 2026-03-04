# RobotStatus.msg
# 각 로봇이 주기적으로 오케스트레이터에 보고하는 상태 메시지.
# Deadline QoS로 5초 내 수신 없으면 통신 두절로 판단.

std_msgs/Header header

# 로봇 식별
string robot_id                         # "argos1", "argos2", ...
string robot_type                       # "ugv" | "drone" | "legged"

# 상태
uint8 STATE_IDLE = 0
uint8 STATE_EXPLORING = 1
uint8 STATE_ON_MISSION = 2
uint8 STATE_RETURNING = 3
uint8 STATE_ERROR = 4
uint8 STATE_COMM_LOST = 5
uint8 state

# 위치
geometry_msgs/PoseStamped pose

# 자원
float32 battery_percent                 # 0.0 ~ 100.0 (시뮬레이션에서는 고정값)

# 임무 진행
string current_mission                  # "explore" | "monitor_fire" | "return" | "idle"
float32 mission_progress                # 0.0 ~ 1.0

# 탐색 정보
uint32 frontiers_remaining              # 남은 프론티어 수
float32 coverage_percent                # 탐색 커버리지 (추정)

# 능력 (로봇 타입별 다름)
string[] capabilities                   # ["thermal", "lidar", "depth", "gripper"]
