# NetworkDiagnostics.msg
# 로봇별 통신 상태 진단 — 패킷 손실, 지연, 신호 강도
#
# 통신/네트워크 전문가 권고: 소방 현장 WiFi 불안정 환경에서
# 지휘관이 통신 품질을 실시간으로 파악할 수 있도록.

std_msgs/Header header

string robot_id

# 통신 품질
uint32 seq_number                      # 발행 시퀀스 (패킷 손실 감지용)
float32 packet_loss_percent            # 추정 패킷 손실율 (%)
float32 latency_ms                     # 추정 지연 (밀리초)
int16 rssi_dbm                         # WiFi 신호 강도 (dBm, -90~-20)
uint8 link_quality                     # 0~100 (추정 통신 성공률)

# 대역폭
float32 messages_per_sec               # 초당 수신 메시지 수
float32 bytes_per_sec                  # 초당 바이트 수

# 상태
string status                          # "excellent"|"good"|"fair"|"poor"|"critical"
