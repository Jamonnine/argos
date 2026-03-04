# MissionState.msg
# 오케스트레이터가 퍼블리시하는 전체 임무 현황.
# 지휘관(인간)이 상황을 한눈에 파악하기 위한 종합 상태.

std_msgs/Header header

# 임무 단계
uint8 STAGE_INIT = 0
uint8 STAGE_EXPLORING = 1
uint8 STAGE_FIRE_RESPONSE = 2
uint8 STAGE_RETURNING = 3
uint8 STAGE_COMPLETE = 4
uint8 STAGE_PAUSED = 5
uint8 stage

# 전체 현황
uint8 total_robots
uint8 active_robots
uint8 comm_lost_robots
float32 overall_coverage_percent        # 전체 탐색 커버리지

# 화점 정보
uint8 fire_count                        # 감지된 화점 수
geometry_msgs/Point[] fire_locations     # 화점 좌표 목록

# 로봇별 요약
string[] robot_ids
uint8[] robot_states
string[] robot_missions

# 경과 시간
float64 elapsed_sec
