# Human dummy YOLO perception for ROS 2

기존 카메라의 RGB·정렬된 깊이 영상으로 인간 더미를 인식하고, 기존 BT가 구독할 감지 여부와 3차원 좌표를 발행하는 독립 ROS 2 패키지입니다.

**포함:** YOLO 추론, 학습된 `models/best.pt`, 깊이 좌표 계산, 설정·launch·의존성·테스트.
**제외:** 시뮬레이터·맵·로봇 모델, 이동·접근·회전 제어, BT 실행기, 학습 이미지·라벨·학습 코드.

기존 시뮬레이터 또는 실제 카메라를 먼저 실행해야 합니다. 이 패키지는 시뮬레이터를 실행하거나 인간 더미를 배치하지 않습니다. 기존 프로젝트의 BT/이동 명령도 변경하지 않습니다.

## 1. 설치와 실행 — 저장소를 클론한 경우

ROS 2 Jazzy / Ubuntu 24.04 / Python 3.12 환경 기준입니다. ROS 의존성은 `package.xml`, YOLO Python 의존성은 `requirements-yolo.txt`에 있습니다. `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`가 설치된 ROS 환경이 필요합니다.

`py_bt_ros` 저장소 루트에서 실행합니다. 예제 경로는 개인 노트북의 절대 경로를 사용하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages "$HOME/.venvs/human-yolo"
source "$HOME/.venvs/human-yolo/bin/activate"
python -m pip install -r demos/limo_dummy_yolo/requirements-yolo.txt

# 기존 시뮬레이터의 /clock을 사용한다면 true, 실제 카메라라면 false
bash demos/limo_dummy_yolo/scripts/run_detector.sh -p use_sim_time:=true
```

`--system-site-packages`는 ROS의 시스템 Python 의존성을 함께 사용하기 위해 필요합니다. 기존 YOLO 환경을 사용할 때는 그 환경에서 ROS Python 모듈도 import되는지 확인하세요. 다른 인터프리터를 지정하려면 `YOLO_PYTHON=/path/to/venv/bin/python`을 설정합니다.

입력 토픽 이름이 다르면 YAML 또는 실행 인자로 변경합니다.

```bash
bash demos/limo_dummy_yolo/scripts/run_detector.sh \
  -p use_sim_time:=true \
  -p rgb_topic:=/robot/camera/image_raw \
  -p depth_topic:=/robot/camera/aligned_depth/image_raw \
  -p camera_info_topic:=/robot/camera/camera_info
```

입력 RGB는 `rgb8`/`bgr8`, 깊이는 `16UC1`(mm) 또는 `32FC1`(m)이어야 합니다. **깊이는 RGB 픽셀에 정렬되어야 하며, RGB·깊이·CameraInfo의 frame_id와 해상도가 일치해야 합니다.** 정렬되지 않은 별도 깊이 카메라를 그대로 입력하지 마세요. 프레임·해상도가 같다는 검사는 정렬 자체를 수행하지 않습니다.

## 2. ROS 패키지로 빌드하는 경우

이 폴더의 패키지 이름은 `human_yolo`입니다. YOLO 의존성을 설치한 위 가상환경을 활성화하고, ROS 작업공간 루트에서 **해당 폴더만** 빌드합니다. 아래는 저장소가 `src/py_bt_ros`에 있는 예입니다.

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/.venvs/human-yolo/bin/activate"
python -m colcon build \
  --base-paths src/py_bt_ros/demos/limo_dummy_yolo \
  --packages-select human_yolo
source install/setup.bash
ros2 launch human_yolo detector.launch.py use_sim_time:=true
```

`model_path`, `params_file` launch 인자로 모델·설정을 바꿀 수 있습니다. 기본 모델은 패키지 설치 위치에서 찾습니다. 빌드 없이 실행하는 1번 방식도 같은 토픽을 제공합니다.

## 3. 입력·출력 토픽

| 방향 | 기본 토픽 | 메시지 | 의미 |
|---|---|---|---|
| 입력 | `/camera/color/image_raw` | `sensor_msgs/Image` | RGB 영상 |
| 입력 | `/camera/depth/image_raw` | `sensor_msgs/Image` | RGB에 정렬된 깊이 영상 |
| 입력 | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RGB 픽셀 기준 내부 파라미터 |
| 출력 | `/human/detected` | `std_msgs/Bool` | 최신 유효 RGB에 대상 클래스가 있으면 true |
| 출력 | `/human/position_valid` | `std_msgs/Bool` | 해당 RGB에 대응하는 깊이 기반 좌표가 있으면 true |
| 출력 | `/human/position` | `geometry_msgs/PointStamped` | 유효할 때만 발행하는 대표 대상 좌표 |
| 출력 | `/human/detections` | `std_msgs/String` | 촬영 시각·클래스·신뢰도·좌표가 담긴 JSON |

출력 토픽은 ROS remap으로 변경할 수 있습니다. 입력은 sensor-data QoS(best effort), 출력은 reliable/volatile입니다. 노드가 시작하면 카메라를 기다리는 동안 Bool 출력은 false입니다. 실행 중 파라미터 변경 대신 수정된 설정으로 재시작하세요.

### 좌표 규약

- 단위: **미터**.
- 기준: 입력 RGB의 `header.frame_id`인 **카메라 광학 좌표계**. x=오른쪽, y=아래쪽, z=카메라 전방.
- `PointStamped.header.stamp`는 추론 완료 시각이 아닌 **RGB 촬영 시각**입니다.
- 좌표는 탐지 영역의 전경 깊이와 픽셀 중앙값으로 추정한 대표점입니다. 기하학적 몸체 중심이나 지도 좌표가 아닙니다.
- 다리 사이 배경을 거리로 사용하지 않도록 전경 깊이 군집을 선택합니다.
- 여러 대상이 있으면 **유효한 깊이를 가진 대상 중 신뢰도가 가장 높은 대상**의 좌표를 발행합니다. 모든 대상은 JSON에서 볼 수 있습니다.
- 지도나 로봇 기준 좌표가 필요한 BT는 촬영 시각의 TF로 변환해야 합니다. 이 패키지는 TF·이동 제어를 수행하지 않습니다.

### 미탐지·깊이 실패·오래된 정보

- 미탐지: `detected=false`, `position_valid=false`, 좌표 발행 없음.
- RGB 탐지는 성공했지만 깊이/CameraInfo가 없거나, 깊이 시각 차이가 기본 0.10초를 넘거나, 정렬 조건/유효 깊이 검사를 통과하지 못하면: `detected=true`, `position_valid=false`, 좌표 발행 없음.
- 영상이 끊기거나 추론 결과가 촬영 시각 기준 기본 1.0초보다 오래되면 watchdog이 두 Bool을 false로 만듭니다. 검사 주기는 0.2초입니다.
- 실패 시 `[0,0,0]` 같은 가짜 좌표를 발행하지 않습니다. 이전 PointStamped를 보관한 BT는 **반드시 position_valid와 좌표 timestamp를 함께 확인**해야 합니다.
- 노드가 강제 종료되면 false 메시지를 보장할 수 없습니다. BT는 노드 생존과 무관하게 자체 freshness 제한을 적용해야 합니다. `/human/detections` 역시 마지막 메시지의 stamp를 검사해야 합니다.
- Bool·PointStamped는 서로 다른 토픽이므로 원자적인 묶음이 아닙니다. 한 프레임의 완전한 결과가 필요하면 JSON 메시지를 한 번에 해석하세요.
- 시뮬레이션에서는 인식 노드·BT·카메라가 동일한 ROS 시간(`/clock`)을 사용해야 합니다.

BT에서는 이 토픽들을 구독해 블랙보드에 넣고, `detected && position_valid && age(position.header.stamp) <= input_timeout`일 때만 좌표를 사용하면 됩니다. ROS 구독 타입 예:

```python
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped

node.create_subscription(Bool, '/human/detected', on_detected, 10)
node.create_subscription(Bool, '/human/position_valid', on_position_valid, 10)
node.create_subscription(PointStamped, '/human/position', on_position, 10)
# on_* 콜백은 기존 BT의 blackboard에 최신 값과 수신 시각을 저장하도록 구현합니다.
```

확인 명령:

```bash
ros2 topic echo /human/detected
ros2 topic echo /human/position_valid
ros2 topic echo /human/position
ros2 topic echo /human/detections
```

JSON은 `stamp`, `frame_id`, `detected`, `position_valid`, `selected_index`, `detections`, `inference_ms`를 제공합니다. 각 탐지는 `class_name`, `confidence`, `xyxy`와, 깊이가 유효한 경우에만 `position_camera: {x,y,z}`를 포함합니다. 이전 데모의 `/yolo/detections`와 토픽 이름이 다르므로 구독부를 위 규약에 맞춰야 합니다.

## 4. 모델·설정

`best.pt`는 YOLO11n 기반 1개 클래스(`rescue_dummy`) 모델입니다. 초록색 상의·파란색 바지의 시뮬레이터 Pedestrian으로 학습했습니다. 다른 사람·복장·실제 환경까지 일반화한 사람 인식 모델은 아닙니다. 기존 모델 시험 결과(confidence 0.5 / IoU 0.5)는 정밀도 91.9%, 재현율 95%이며 이번 PR은 재학습을 포함하지 않습니다.

주요 설정은 `config/detector.yaml`에 있습니다. 기본 CPU/320px/신뢰도 0.5, 깊이 범위 0.15~3.0m입니다. `model_path`가 없으면 다운로드로 대체하지 않고 오류로 종료합니다. 배포 모델의 클래스와 `target_class`가 다르면 실행을 거부합니다.

소스 코드는 저장소의 GPL-3.0 라이선스를 따릅니다(`LICENSE`). Ultralytics 및 기반 가중치의 라이선스·출처는 해당 프로젝트와 체크포인트 메타데이터를 참조하세요.

## 5. 검증

```bash
python -m pip install pytest
# 이 패키지 폴더에서 실행
PYTHONPATH=. ROS_DOMAIN_ID=173 ROS_LOG_DIR=/tmp/human_yolo_tests \
  python -m pytest -q tests
```

11개 기하/동기화 테스트와 1개 실제 ROS pub/sub 테스트를 포함합니다. ROS 테스트는 결정적인 추론 대역을 사용해 탐지·좌표·깊이 실패·미탐지·영상 중단 계약을 검증합니다. 시뮬레이터나 학습 이미지가 필요하지 않습니다. 실제 모델을 사용한 추론과 설치된 패키지 실행도 별도로 확인합니다.

## main 공장 BT 호환 출력

main의 `limo_factory_search`가 사용하는 `/perception/is_person` (`Bool`)과 `/perception/person_pose` (`PoseStamped`)도 함께 발행합니다. `mock_detector:=false`로 기존 모의 감지를 끄고 이 노드를 별도로 실행하세요.

`/perception/is_person`은 **유효한 깊이 좌표까지 있는 경우에만 true**입니다. RGB에서만 보이는 경우는 `/human/detected=true`지만 이 호환 출력은 false로 유지하여 이전 Pose가 잘못 사용되는 것을 줄입니다. Pose의 촬영 시각·frame_id·위치는 PointStamped와 같고, orientation은 추정하지 않으므로 identity이며 BT는 위치만 사용해야 합니다. main의 기록 노드가 카메라→map TF를 조회하므로 기존 시뮬레이터의 TF와 Nav2/SLAM이 필요합니다.

시뮬레이터 저장소의 공장 월드·launch는 이 PR의 파일이 아닙니다. 해당 파일들이 없는 시뮬레이터 main에 이 인식 패키지만 추가해도 공장 시나리오 자체가 생성되지는 않습니다.
