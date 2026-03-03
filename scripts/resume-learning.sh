#!/bin/bash
# ARGOS 학습 세션 빠른 복구 스크립트

echo "📚 ARGOS ROS 2 학습 세션 목록"
echo "================================"
echo ""
echo "주요 학습 세션:"
echo ""
echo "1) 2월 16일 - RGB-D & Point Cloud 심화 (6.0MB) ⭐ 가장 최근 학습"
echo "   claude --session c398e962-2f79-4f45-9170-92c622db34b0"
echo ""
echo "2) 2월 14일 - 학습 세션 (111KB)"
echo "   claude --session cbf31686-4147-4bd6-8d51-b02113c671a0"
echo ""
echo "3) 2월 11일 - 학습 세션 (215KB)"
echo "   claude --session 6bf75b23-5d2e-48c3-8a3c-f543f83a3d09"
echo ""
echo "4) 2월 8일 - 초기 대형 학습 (1.3MB)"
echo "   claude --session 4409aa9c-6f66-4878-bce9-e6d98c20e32a"
echo ""
echo "================================"
echo ""
read -p "복구할 세션 번호 (1-4) 또는 q (종료): " choice

case $choice in
    1)
        echo "2월 16일 Point Cloud 학습 세션을 복구합니다..."
        claude --session c398e962-2f79-4f45-9170-92c622db34b0
        ;;
    2)
        echo "2월 14일 학습 세션을 복구합니다..."
        claude --session cbf31686-4147-4bd6-8d51-b02113c671a0
        ;;
    3)
        echo "2월 11일 학습 세션을 복구합니다..."
        claude --session 6bf75b23-5d2e-48c3-8a3c-f543f83a3d09
        ;;
    4)
        echo "2월 8일 초기 학습 세션을 복구합니다..."
        claude --session 4409aa9c-6f66-4878-bce9-e6d98c20e32a
        ;;
    q|Q)
        echo "종료합니다."
        exit 0
        ;;
    *)
        echo "잘못된 선택입니다."
        exit 1
        ;;
esac
