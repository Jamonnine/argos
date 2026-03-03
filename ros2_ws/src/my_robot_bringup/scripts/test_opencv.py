#!/usr/bin/env python3
"""
간단한 OpenCV 예제: 색상 감지
특정 색상(빨간색) 범위의 픽셀을 찾고 윤곽선 그리기
"""

import cv2
import numpy as np

def main():
    # 테스트용 이미지 생성 (실제로는 카메라에서 읽음)
    # 640x480 크기의 검은 배경
    image = np.zeros((480, 640, 3), dtype=np.uint8)

    # 빨간 원 그리기 (테스트용 물체)
    cv2.circle(image, (200, 200), 50, (0, 0, 255), -1)  # BGR 순서!

    # 파란 사각형 그리기
    cv2.rectangle(image, (400, 300), (500, 400), (255, 0, 0), -1)

    print("원본 이미지 생성 완료")
    print(f"이미지 크기: {image.shape}")  # (height, width, channels)

    # BGR → HSV 변환
    # HSV = Hue(색상), Saturation(채도), Value(명도)
    # 색상 기반 필터링에 HSV가 RGB보다 훨씬 효과적
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 빨간색 범위 정의 (HSV)
    # 빨간색은 HSV에서 0-10 또는 170-180 범위
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # 마스크 생성: 빨간색 픽셀만 True
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2  # 두 범위 합치기

    print(f"빨간색 픽셀 수: {np.sum(mask > 0)}")

    # 윤곽선 찾기
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    print(f"발견된 윤곽선 개수: {len(contours)}")

    # 결과 이미지에 윤곽선 그리기
    result = image.copy()
    cv2.drawContours(result, contours, -1, (0, 255, 0), 2)  # 녹색 윤곽선

    # 각 윤곽선의 중심점 계산 및 표시
    for i, contour in enumerate(contours):
        # 모멘트 계산 (윤곽선의 기하학적 특성)
        M = cv2.moments(contour)

        if M["m00"] != 0:  # 면적이 0이 아니면
            # 중심점 (무게중심)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # 면적
            area = cv2.contourArea(contour)

            print(f"물체 {i+1}: 중심 ({cx}, {cy}), 면적: {area:.1f}")

            # 중심점에 십자 표시
            cv2.drawMarker(result, (cx, cy), (0, 255, 255),
                          markerType=cv2.MARKER_CROSS,
                          markerSize=20, thickness=2)

            # 텍스트 표시
            cv2.putText(result, f"Object {i+1}", (cx - 30, cy - 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # 이미지 저장 (WSL에서는 GUI 창이 안 뜰 수 있으므로)
    cv2.imwrite('/tmp/opencv_test_original.png', image)
    cv2.imwrite('/tmp/opencv_test_mask.png', mask)
    cv2.imwrite('/tmp/opencv_test_result.png', result)

    print("\n결과 이미지 저장 완료:")
    print("  /tmp/opencv_test_original.png")
    print("  /tmp/opencv_test_mask.png")
    print("  /tmp/opencv_test_result.png")
    print("\nWindows에서 확인: \\\\wsl$\\Ubuntu\\tmp\\opencv_test_*.png")

if __name__ == '__main__':
    main()
