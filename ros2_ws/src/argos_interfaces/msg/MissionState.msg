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

# 화재 대응
string primary_responder              # 화재 대응 주담당 로봇 ID (없으면 빈 문자열)

# 8중 센싱 현황 (v2)
string gas_danger_level                # "safe"|"caution"|"danger"|"critical" — 현장 가스 위험도
uint8 victims_detected_count           # 감지된 피해자 수
uint8 blocked_areas_count              # 통행 차단 영역 수
uint8 audio_alerts_count               # 최근 음향 이벤트 수

# 임무 제한 (소방 전문가 권고: 부수 임무 모드)
string[] mission_restrictions              # ["no_entry_zone_A", "monitor_only"] 등

# 화재 확산 속도 (센서 퓨전: Kalman 추적)
float32 fire_spread_rate_mps               # m/s (0=정지, >0.5=급속 확산)

# 경과 시간
float64 elapsed_sec
