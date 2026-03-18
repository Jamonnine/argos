# ARGOS Fire & Smoke Detection Model

불꽃/연기 바운딩박스 탐지 모델. D-Fire 데이터셋 + YOLOv8n 파인튜닝.

## Quick Start

### 1. 데이터셋 다운로드

D-Fire 데이터셋 (21,527장, fire+smoke 2클래스):
- **Kaggle**: https://www.kaggle.com/datasets/sayedgamal99/smoke-fire-detection-yolo
- **OneDrive**: https://1drv.ms/u/c/c0bd25b6b048b01d/EbLgD7bES4FDvUN37Grxn8QBF5gIBBc7YV2qklF08GCiBw

### 2. 폴더 구조

```
models/fire-smoke/
├── data.yaml          # 데이터셋 설정
├── train.py           # 학습 스크립트
├── test.py            # 테스트/벤치마크
├── README.md
└── datasets/d-fire/   # ← 여기에 D-Fire 압축 해제
    ├── images/{train,val,test}/
    └── labels/{train,val,test}/
```

### 3. 학습

```bash
pip install ultralytics
python train.py                  # GPU 자동 감지, 30 epochs
python train.py --imgsz 960      # 고해상도 (AI-Hub 모델과 동일)
```

### 4. 테스트

```bash
python test.py
python test.py --image path/to/fire.jpg
```

### 5. ARGOS 통합

학습 결과 `runs/detect/fire-smoke/weights/best.pt`를 ARGOS 노드에서 사용.

## 관련 모델

| 모델 | 역할 | 라이선스 |
|------|------|---------|
| **이 모델** (fire-smoke) | 불꽃/연기 위치 탐지 | 학술용 |
| AI-Hub YOLOv8s | 화재현장 객체 15종 탐지 | AGPL-3.0 |
| ARGOS hotspot_detector | 열화상 온도 임계값 | Apache-2.0 |
| AI-Hub SlowFast | 영상 화재/연기/정상 분류 | AGPL-3.0 |
