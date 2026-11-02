# LIMO 인간 더미 YOLO 인식 데모 (실행용)

학습 이미지와 라벨 없이, 학습된 `rescue_dummy` 모델로 Webots 인간 더미를 인식하는 묶음입니다. 초록색 상의·파란색 바지의 Pedestrian 모델을 대상으로 학습했습니다.

## 구성

- `models/best.pt`: 학습 완료한 YOLO11n 더미 모델
- `scripts/detector.py`, `scripts/run_detector.sh`: LIMO 카메라 실시간 탐지
- `scripts/overlay.py`: 선택적인 Webots 내부 탐지 상태 표시용 Supervisor 컨트롤러
- `simulation/webots_ros2_limo/`: LIMO 시뮬레이터 패키지와 `limo_human_demo.wbt`
- `requirements-yolo.txt`: YOLO 의존성

학습 사진·라벨·재학습 가중치·검증 이미지는 제외했습니다. 이 묶음만으로 재학습하지는 않습니다. 원본 학습 자료는 기존 `dummy_yolo/`에 보관되어 있습니다.

## 시뮬레이터 실행

ROS 2 Jazzy, Webots R2025a 및 LIMO ROS 의존성이 필요합니다. Webots 기본 모델은 최초 실행 시 다운로드가 필요할 수 있습니다.

`simulation/webots_ros2_limo`를 사용할 ROS 작업공간의 `src/`에 복사합니다. 같은 패키지가 이미 있으면 중복 등록하지 마세요. 그 작업공간 루트에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select webots_ros2_limo --symlink-install
source install/setup.bash
ros2 launch webots_ros2_limo robot_launch.py world:=limo_human_demo.wbt nav:=false slam:=false
```

## 실시간 인식

다른 터미널에서 ROS와 작업공간 환경을 불러오고, 이 데모 폴더로 이동한 후 실행하세요. YOLO Python 환경은 ROS Python 라이브러리를 읽을 수 있어야 합니다.

```bash
source /opt/ros/jazzy/setup.bash
# 작업공간의 install/setup.bash도 source 하세요.
export YOLO_PYTHON=/path/to/yolo/venv/bin/python
"$YOLO_PYTHON" -m pip install -r requirements-yolo.txt
bash scripts/run_detector.sh
```

`/camera/color/image_raw`를 받아 `/yolo/detections`와 `/yolo/annotated_image`를 발행합니다. 상자 탐지 노드는 중지하고 실행하세요. rqt 창이나 로봇 이동·회전은 자동으로 실행하지 않습니다.

`overlay.py`는 Webots 외부 Supervisor 로봇을 생성하고 연결해야 사용하는 선택 기능입니다. 카메라 탐지는 `run_detector.sh`만으로 실행됩니다.

## 검증 범위

시험 이미지에서 confidence 0.5 / IoU 0.5 기준 정밀도 91.9%, 재현율 95%였습니다. 실제 LIMO 카메라 정지 장면에서도 30/30 프레임 탐지를 확인했습니다. 같은 시뮬레이터 더미와 방에서 확인한 결과로, 실제 사람이나 주행 중 인식 성능을 보장하는 수치는 아닙니다.

시뮬레이터 라이선스는 `simulation/webots_ros2_limo/LICENSE`를 참조하세요. Ultralytics·가중치·Webots 기본 모델의 사용 조건은 각 배포 조건을 따릅니다.
