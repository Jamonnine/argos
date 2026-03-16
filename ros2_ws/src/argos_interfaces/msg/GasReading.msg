# GasReading.msg
# 가스 센서 측정 결과 — CO/O2/LEL 3종 동시 측정
#
# 소방 현장 진입 판단의 핵심 데이터.
# CO 50ppm 이상: 주의, 200ppm 이상: 위험, 800ppm 이상: 즉시 철수
# O2 18% 이하: 질식 위험, 23.5% 이상: 산소 과잉(폭발 위험)
# LEL 10% 이상: 폭발 주의, 25% 이상: 즉시 철수

std_msgs/Header header

# 센서 식별
string robot_id
string sensor_frame                    # 센서 TF 프레임

# 가스 농도
float32 co_ppm                         # 일산화탄소 (ppm). 화재 사망 원인 1위
float32 o2_percent                     # 산소 농도 (%). 정상=20.9%
float32 lel_percent                    # 가연성 가스 폭발하한 (%). 정상=0%
float32 co2_ppm                        # 이산화탄소 (ppm). 연소 부산물
float32 hcn_ppm                        # 시안화수소 (ppm). 플라스틱 연소 시 발생

# 위치 (맵 좌표)
geometry_msgs/PointStamped location

# 위험도 판정
string danger_level                    # "safe" | "caution" | "danger" | "critical"
string[] hazard_types                  # ["co_high", "o2_low", "lel_explosive"] 등
bool evacuate_recommended              # 즉시 철수 권고 여부
