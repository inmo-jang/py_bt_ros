# LIMO 3대 실행 환경 — main에 추가하는 패키지

기존 main 파일을 수정하지 않고 공장맵의 LIMO 3대와 `multirobot_bt`를 실행합니다. 제출·머지 대상은 새 `ros_packages/multirobot_sim/`와 `ros_packages/multirobot_bt/` 두 폴더입니다. 기존 `human_yolo`와 팀 BT 엔진은 그대로 재사용합니다.

## 포함 범위

- 공장맵 복사본의 LIMO 3대와 로봇별 Webots 모터·센서 어댑터.
- robot1/robot2/robot3 이름 공간, 개별 odom/base_link/laser/camera TF.
- 공통 지도, 로봇별 AMCL 및 Nav2 3개.
- 기존 human_yolo 모델을 이용한 RGB-D 인식 3개.
- 외부 임무 실행 및 감지 시 전체 중단을 처리하는 멀티로봇 BT.
- RViz의 3대 라이다·경로·좌표축 및 팀 Pygame BT 시각화.

구역·웨이포인트·집결 위치·진입 순서를 계산하는 업무 할당은 포함하지 않습니다. 기본 시험 목표는 기존 main의 구역 1·2·3을 그대로 읽어 로봇 1·2·3에 연결합니다. 외부 업무 할당 모듈은 BT 입력 토픽으로 이를 대체할 수 있습니다. 지도 없는 탐색은 외부 탐색 모듈에서 연결합니다. 시작 위치는 환경 설정이며 업무 할당 결과가 아닙니다.

## 빌드

ROS 2 Jazzy, Webots R2025a, webots_ros2_driver, Nav2 및 README에 기재한 BT/Yolo Python 의존성이 필요합니다. 저장소를 워크스페이스 아래 임의 경로에 클론한 경우에도 사용할 수 있습니다. 기존 main 코드·설정·월드를 덮어쓰지 마세요.

워크스페이스 루트에서(현재 클론 폴더명 py_bt_ros_multi 기준):
```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths py_bt_ros_multi/ros_packages/multirobot_bt py_bt_ros_multi/ros_packages/multirobot_sim py_bt_ros_multi/ros_packages/human_yolo --build-base build_bt --install-base install_bt --symlink-install
source install_bt/setup.bash
ros2 launch multirobot_sim multirobot_sim.launch.py
```

이 명령은 Webots·로봇 3대·센서·TF·AMCL·Nav2·YOLO·BT·RViz·Pygame을 함께 실행합니다. 여섯 개 scan/odom 입력을 확인한 후 Nav2 등을 시작하고, BT는 Nav2 활성화까지 확인한 후 시작 요청을 허용합니다. **main 구역 목표를 읽어도 시작 요청 전에는 세 로봇 모두 대기합니다. launch만으로 자동 이동하지 않습니다.**

Python 환경은 인자로 지정할 수 있습니다. 기본값은 기존 로컬 가상환경이 있으면 사용하고 없으면 실행 Python입니다.
```bash
ros2 launch multirobot_sim multirobot_sim.launch.py python_executable:=/path/to/ros_pygame_python yolo_python:=/path/to/yolo_python
```

BT Python에는 ROS·pygame·PyYAML, YOLO Python에는 ROS·Ultralytics·PyTorch·OpenCV·기존 human_yolo 모듈이 필요합니다. `source install_bt/setup.bash`를 한 환경에서 실행하세요. 소스 트리를 자동으로 찾지 못하면 `repo_path:=/absolute/path/to/py_bt_ros`를 지정하세요. 기기별 경로는 소스에 하드코딩하지 않았습니다.

선택 옵션: `rviz:=false`, `pygame:=false`, `yolo:=false`. Webots 공식 원격 PROTO/텍스처가 처음에는 다운로드되므로 네트워크나 에셋 캐시가 필요합니다.

## main 구역 기본 목표

기본값 `use_main_regions:=true`로 `scenarios/limo_factory_search/config_sim.yaml`의 `mission.regions`를 실행 시 읽습니다. 좌표를 새 파일에 복사하지 않으며 main은 수정하지 않습니다.

| 로봇 | main 구역 | 이름 | 목표 (x, y, yaw) |
|---|---|---|---|
| robot1 | 1 (A) | Left space | (1.0, 8.6, -1.570796327) |
| robot2 | 2 (B) | Deep space | (1.2, -8.6, 1.570796327) |
| robot3 | 3 (C) | Person room | (4.85, 0.65, -1.570796327) |

Nav2 등이 준비된 뒤 `/multi_bt/start`를 호출하면 각 로봇이 위 건물 내부 목표로 이동합니다. 입구 통과 경로는 Nav2가 계산합니다. 현재 이 내부 진입은 실제 주행 검증 전입니다. 이 좌표들은 구역별 대표 지점이므로 구역 전체를 훑는 순찰 경로는 아닙니다. 한 지점 도착 후에는 같은 지점이 반복 목표가 됩니다. 여러 지점의 순찰 경로는 외부 모듈에서 전달해야 합니다.

외부 업무 할당만 사용할 때는 `use_main_regions:=false`로 기본 목표를 끌 수 있습니다. 초기 대기 상태에서 `/multi_bt/patrol_tasks`를 보내면 기본 목표를 덮어씁니다. 임무 실행 중에는 덮어쓰지 않습니다. main 설정의 `mock_person_regions` 등은 읽지 않으며 실제 YOLO 감지를 사용합니다.

## 업무 할당과 연결

1. 기본 main 목표를 사용하거나, 업무 할당 모듈이 `/multi_bt/patrol_tasks`에 robot1/2/3 각각의 경로를 보내 대체합니다.
2. `/multi_bt/start` 호출 후 세 대가 받은 경로로 순찰합니다.
3. 어느 로봇이든 인간 더미를 감지하면 전체 이동을 취소하고 정지합니다.
4. `/multi_bt/target`과 `/multi_bt/status`로 발견 좌표·임무 번호를 전달합니다.
5. WAIT_RESCUE_TASKS 상태에서 업무 할당 모듈이 현재 mission_id, 세 집결 목표와 순서를 `/multi_bt/rescue_tasks`에 보냅니다.
6. BT가 지정 순서대로 실행하고 세 대 도착 시 COMPLETE가 됩니다.

JSON 규약과 토픽 타입은 [BT README](../multirobot_bt/README.md)에 있습니다. 할당 모듈 없이 이 단계의 집결 목표를 자동 생성하지 않습니다.

```bash
ros2 service call /multi_bt/start std_srvs/srv/Trigger '{}'
ros2 topic echo /multi_bt/status
ros2 service call /multi_bt/stop std_srvs/srv/Trigger '{}'
```

RViz의 단일 전역 Goal 도구 대신 BT 할당 토픽을 사용합니다. 로봇별 목표를 직접 보내는 노드와 BT가 동시에 이동을 제어해서는 안 됩니다.

## 제어·지도 구성

- 공통 map → robotN/odom은 각 AMCL이, robotN/odom → robotN/base_link는 센서 어댑터가 발행합니다.
- Nav2가 각 로봇의 라이다로 이동 경로와 로컬 장애물 회피를 수행합니다. 이전 데모의 업무 할당·집결 계산·인공 장애물 누적 레이어는 포함하지 않습니다.
- BT는 `/multi_bt/emergency_stop`을 주기적으로 발행합니다. Webots 드라이버는 초기 정지 상태이며, BT 허가·최신 속도 명령이 모두 있어야 움직입니다. 허가 갱신이 1초(시뮬레이션 시간) 끊기면 모터를 정지합니다.
- `config/environment.yaml`은 맵·월드·시작 위치만 보관합니다. 원본 공장맵 출처는 `config/factory_source.yaml`에 기록했습니다. 지도는 SLAM 결과가 아닌 기존 라이다 측량 지도입니다.

## 실제 LIMO에서 사용할 때

`multirobot_sim`은 Webots 전용입니다. 실제 3대에서는 각 하드웨어 드라이버·카메라·라이다·TF·Nav2가 동일한 robotN 토픽 규약을 제공해야 합니다. 그런 환경에서 `multirobot_bt multirobot_bt.launch.py use_sim_time:=false`를 실행합니다. 실제 로봇 드라이버의 정지 인터록 연결은 기기에 맞춰야 하며, 실제 LIMO 3대의 통신 연결·실물 검증은 이번에 수행하지 않았습니다. 시뮬레이터와 실물의 위치를 동기화하는 디지털 트윈 기능도 포함하지 않습니다.

## 검증 범위

human_yolo·multirobot_bt·multirobot_sim 세 패키지 빌드, BT와 프레임·이름 공간·월드·드라이버 감시 시험 28개를 통과했습니다. 통합 launch 인자 파싱을 확인했고, 격리 ROS 도메인과 가상 화면에서 BT·Pygame의 시작 및 정상 종료를 확인했습니다. 기존 main 추적 파일 변경은 0개입니다. 이번 구성에서는 전체 시뮬레이터를 재실행하지 않습니다. 이전 통합 데모의 집결 완료 결과는 새 외부 업무 할당 연결의 완료 증거가 아닙니다. 실제 외부 할당 모듈과 연결한 전체 주행 검증은 별도로 필요합니다. 커밋·푸시는 하지 않았습니다.
