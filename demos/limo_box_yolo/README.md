# Webots LIMO 상자 인식 YOLO 데모

Webots에서 촬영한 갈색 목표 상자로 YOLO11n을 추가 학습한 결과입니다.
기존 `py_bt_ros` 실행기와 분리된 학습·ROS 인식 예제이며, 기존 BT 코드는 변경하지 않습니다.

## 구성

- `models/best.pt`: 학습 완료 모델 (box 한 클래스)
- `models/yolo11n.pt`: 기반 사전학습 가중치
- `dataset/`: 촬영 RGB 이미지, YOLO 라벨과 촬영 위치 기록
- `data.yaml`: 데이터셋 설정. 사람 자세 인식용 설정과는 별개입니다.
- `scripts/train.py`, `evaluate.py`: 추가 학습·별도 테스트 평가
- `scripts/detector.py`, `depth_geometry.py`: ROS 2 실시간 RGB-D 인식
- `reports/`: 학습 곡선 CSV, 테스트 결과, 탐지 예시

## 검증 결과와 범위

18 epoch, 입력 320px, CPU로 학습했습니다. 데이터는 학습 280장(배경 40), 검증 70장(배경 10),
테스트 70장(배경 10)이며 각 분할의 상자 배치를 분리했습니다.
confidence=0.5, IoU=0.5의 별도 테스트에서 TP 59 / FP 4 / FN 1,
정밀도 93.65%, 재현율 98.33%였습니다. 배경 10장 중 1장에서 오탐이 발생했습니다.
이 수치는 현재 시뮬레이터의 목표 상자에 대한 결과이며 실제 상자나 다른 환경의 성능을 뜻하지 않습니다.

이미지는 Webots에서 LIMO 카메라와 동일한 시야각·높이·크롭을 사용해 수집했고,
상자 전용 렌더링 마스크로 정답 라벨을 만들었습니다. 실시간 추론에는 마스크나 실제 상자 좌표를 사용하지 않습니다.

## 실시간 인식

ROS 2 Jazzy와 LIMO 카메라가 실행 중인 환경에서, YOLO가 설치된 Python을 지정합니다.

```bash
cd demos/limo_box_yolo
source /opt/ros/jazzy/setup.bash
YOLO_PYTHON=~/yolo_project/.venv/bin/python ./scripts/run_detector.sh
```

필요하면 별도 가상환경에 `requirements-yolo.txt`를 설치합니다. PyTorch는 사용 하드웨어에 맞는 빌드를 사용하세요.

입력은 `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`입니다.
깊이 영상은 RGB와 정렬된 16UC1(mm) 또는 32FC1(m) 형식이어야 합니다.
출력은 `/yolo/detections`(JSON String), `/yolo/annotated_image`(Image)입니다.
인식은 RGB로 수행하며 깊이·카메라 내부 파라미터가 유효하면 카메라 기준 표면 좌표를 덧붙입니다.
최신 프레임만 처리하며 별도 rqt 창이나 이동 명령을 생성하지 않습니다.

현재 작업 공간의 이동·접근·회전 제어 노드는 별도 `webots_ros2_limo` 패키지에 있으며 이 폴더에는 포함하지 않았습니다.

## 재학습·평가

```bash
YOLO_CONFIG_DIR="$PWD/config" ~/yolo_project/.venv/bin/python scripts/train.py
YOLO_CONFIG_DIR="$PWD/config" ~/yolo_project/.venv/bin/python scripts/evaluate.py
```

스크립트는 위치와 관계없이 폴더 기준으로 데이터 경로를 해석하고 `data.local.yaml`을 생성합니다.
재학습하면 `models/best.pt`를 새 결과로 갱신합니다. 임시 학습 출력·캐시·가상환경은 Git에서 제외됩니다.
가중치 해시는 `models/SHA256SUMS.json`에 기록돼 있습니다.

기반 구현: [Ultralytics YOLO](https://github.com/ultralytics/ultralytics).
