# TemperatureStatistics.msg
# 온도 센서 데이터의 통계 정보

# 타임스탬프 (언제 계산되었는가)
builtin_interfaces/Time timestamp

# 통계 값들
float64 mean              # 평균 온도
float64 std_dev           # 표준 편차
float64 min_value         # 최소 온도
float64 max_value         # 최대 온도
uint32 sample_count       # 샘플 개수

# 윈도우 정보 (몇 개의 샘플로 계산했는가)
uint32 window_size        # 사용된 샘플 수
