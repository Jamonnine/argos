# DetectionArray.msg
# AI 객체 감지 결과 배열 (한 프레임의 전체 감지 결과)
#
# 설계 원칙:
# - 배열 메시지에도 Header 포함 → 언제 촬영된 프레임인지 타임스탬프 보존
# - 개별 DetectedObject에도 각자 Header가 있어서 좌표계 혼용 가능
#   (일부는 camera_frame, 일부는 이미 map으로 변환된 상태 가능)

std_msgs/Header header           # 프레임 전체의 타임스탬프 + 소스 frame_id
argos_interfaces/DetectedObject[] detections  # 감지된 객체 목록
int32 frame_id                   # 카메라 프레임 번호 (디버깅용)
string detector_name             # "yolo_v8", "detectron2" 등 (추적용)
