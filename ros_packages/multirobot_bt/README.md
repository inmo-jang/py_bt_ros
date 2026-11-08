# 멀티로봇 BT

외부에서 받은 로봇 3대의 임무를 실행하고, 구조 대상 발견 시 전체 순찰을 중단하여 구조 임무로 전환하는 ROS 2 패키지입니다. 기존 main BT 엔진(`modules/base_bt_nodes.py`)을 그대로 사용하며 기존 추적 파일을 수정하지 않습니다.

## 기능 및 외부 연동

- 포함: 3대 동시 순찰 BT, 인간 감지에 따른 우선순위 전환, 목표 취소, 세 대 정지 확인, 외부 구조 임무 대기, 지정 순서대로 구조 목표 실행, 완료·실패·운영자 중단 보고.
- 업무 할당 모듈: 구역 배정, 웨이포인트 생성, 집결 좌표와 진입 순서 계산. 이 패키지는 해당 결과를 입력으로 받습니다.
- 로봇 실행 환경: 실제·시뮬 로봇 연결, 이름 공간·Nav2·센서·TF 준비.
- 탐색 모듈: 지도 없는 탐색과 목표 생성. 생성한 목표를 같은 인터페이스로 연결할 수 있습니다.

맵·Webots 월드·로봇 드라이버·YOLO 모델·학습 데이터·구역 할당 알고리즘을 포함하지 않습니다. 충돌 회피와 실제 경로 계산은 외부 Nav2가 수행합니다. 이 패키지는 구호물품 전달이나 물리적 구출을 구현하지 않습니다.

## 간단한 멀티로봇 BT

사용자 선택에 따라 처음의 간단한 멀티로봇 구성으로 되돌렸습니다. 기존 main의 제어 노드 엔진과 Pygame 렌더러는 재사용하며, 공장 단일 로봇 시나리오 XML을 로봇마다 펼쳐 실행하는 방식은 사용하지 않습니다.

```text
ReactiveFallback: Rescue Priority
├─ Sequence: Rescue
│  ├─ HumanDetected
│  ├─ StopAllRobots
│  ├─ WaitRescueAssignment
│  └─ ExecuteAssignedRescue
└─ Parallel: Assigned Patrol
   ├─ robot1 Patrol
   ├─ robot2 Patrol
   └─ robot3 Patrol
```

HumanDetected는 한 번 감지된 구조 요청을 유지합니다. 세 로봇은 외부에서 받은 각 경로를 반복 순찰하고, 어느 로봇이든 인간을 감지하면 진행 중·승인 대기 중인 목표를 취소하고 모두 정지합니다. 최신 오도메트리로 정지를 확인한 뒤 발견 좌표를 map 기준으로 공유하고 외부 업무 할당을 기다립니다. 별도 RecordPerson 노드는 표시하거나 실행하지 않습니다.

구조 목표와 순서가 들어오면 한 대씩 이동합니다. 각 Nav2 성공과 지도상 도착 오차 0.4m 이내를 확인하고, 세 대 모두 도착하면 COMPLETE가 됩니다. 목표 선택·구역 배정은 이 BT에서 하지 않습니다.

상태: WAITING → PATROL → STOPPING → WAIT_RESCUE_TASKS → GATHER → COMPLETE. 실패 시 ERROR, 운영자 중단 시 STOPPED. 구조 단계별 대기 제한은 기본 900초(실제 시간)입니다. 새 임무는 start 서비스를 다시 호출해야 시작합니다.

Pygame은 위 간단한 구조를 그대로 표시합니다. 하위 공장 탐색 트리나 접기·펼치기 기능은 제거했습니다. 기존 main 파일과 시뮬레이터의 로봇 시작 위치는 변경하지 않습니다.

## main 구역 기본 목표

시뮬레이터 통합 launch는 main의 `scenarios/limo_factory_search/config_sim.yaml`을 `factory_regions_file` 파라미터로 전달합니다. robot1=구역1, robot2=구역2, robot3=구역3의 대표 좌표를 그대로 읽어 기본 목표로 등록합니다. 시작 요청 전에는 움직이지 않습니다. 외부 순찰 할당으로 대체할 수 있으며, BT 단독 실행의 파라미터 기본값은 빈 문자열이므로 기본 목표를 로드하지 않습니다. 실제 구역 전체 순찰에는 여러 웨이포인트 입력이 필요합니다.

## 팀원 연결 규약

로봇 이름은 robot1, robot2, robot3입니다. 모든 목표는 `frame_id` 파라미터(기본 map) 기준이며 거리 m, 각도 rad입니다. **BT가 해당 Nav2 목표를 소유하는 동안 다른 노드가 같은 로봇에 이동 목표나 속도를 발행하면 안 됩니다.** 시뮬레이터 드라이버는 `/multi_bt/emergency_stop` 허가·정지 신호와 연결되어 있습니다. 실물 드라이버의 정지 인터록은 하드웨어에 맞게 연결해야 합니다.

| 방향 | 이름 | 타입·내용 |
|---|---|---|
| 입력 | `/multi_bt/patrol_tasks` | std_msgs/String, 아래 JSON. transient-local 구독이므로 발행자도 transient-local 권장 |
| 입력 | `/multi_bt/rescue_tasks` | std_msgs/String, 현재 mission_id와 외부 지정 goals/order |
| 입력 | `/robotN/perception/is_person` | std_msgs/Bool, 유효한 인간 감지 |
| 입력 | `/robotN/perception/person_pose` | geometry_msgs/PoseStamped, 촬영 시각의 좌표와 frame_id 필요 |
| 입력 | `/robotN/odom` | nav_msgs/Odometry, 정지 확인 |
| 입출력 | `/robotN/navigate_to_pose` | nav2_msgs/action/NavigateToPose |
| 출력 | `/robotN/cmd_vel` | geometry_msgs/Twist, 중단·완료 시 0 속도 |
| 출력 | `/multi_bt/emergency_stop` | std_msgs/Bool, true 정지·false 이동 허가, transient-local 및 주기 갱신 |
| 출력 | `/multi_bt/target` | geometry_msgs/PoseStamped, 발견 좌표를 map으로 변환하여 공유, transient-local |
| 출력 | `/multi_bt/status` | std_msgs/String, 상태·mission_id·발견 로봇·도착 목록·오류 JSON, transient-local |
| 서비스 | `/multi_bt/start`, `/multi_bt/stop` | std_srvs/Trigger |

person_pose는 촬영 당시 TF로 map에 변환합니다. 깊이 계산·YOLO 추론은 외부 인식 패키지 역할입니다. 감지 후 최신 좌표를 얻지 못하면 정지 상태에서 기다리다가 제한 시간 초과로 오류를 보고합니다.

순찰 입력 예시(좌표는 실제 맵에 맞춰 **할당 모듈이 결정**):
```json
{"robot1":[[1,2,0],[2,2,1.57]],"robot2":[[-1,2,0]],"robot3":[[0,-2,0]]}
```

구조 입력 예시(아래 좌표는 규약 설명용이며 경로 안전성을 검증한 좌표가 아님):
```json
{"mission_id":1,"goals":{"robot1":[3,2,0],"robot2":[3,3,0],"robot3":[3,4,0]},"order":["robot3","robot1","robot2"]}
```

`/multi_bt/status`가 WAIT_RESCUE_TASKS가 된 뒤 현재 mission_id를 넣어 구조 임무를 보내야 합니다. 이전 임무 번호·중복 로봇·누락 로봇·비정상 좌표·0.7m 미만으로 겹치는 구조 목표는 거부합니다. 이 최소 간격 검사는 업무 할당을 대신하지 않습니다. 실제 바닥 범위·접근 가능성·통행 순서는 할당 모듈과 Nav2에서 확인해야 합니다. 좌표 전송만으로 구조 집결이 자동 생성되지는 않습니다.

## 빌드·실행

워크스페이스 루트(`/home/willy/limo_jazzy_ws`)에서:
```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths py_bt_ros_multi/ros_packages/multirobot_bt --build-base build_bt --install-base install_bt --symlink-install
source install_bt/setup.bash
ros2 launch multirobot_bt multirobot_bt.launch.py use_sim_time:=true
```

실행 Python은 현재 워크스페이스의 `.venv-view/bin/python`이 있으면 사용하고, 없으면 `/usr/bin/python3`를 사용합니다. `python_executable:=...`로 바꿀 수 있습니다. 기존 팀 BT 모듈이 pygame과 YAML을 import하므로 선택한 Python에 두 의존성이 필요합니다.

실제 로봇에서는 use_sim_time:=false를 사용합니다. 이 명령은 **BT만 실행**합니다. 로봇 3대·Nav2·인식 노드·TF는 연결된 모듈에서 먼저 준비하고, 순찰 할당을 발행한 다음 시작합니다.
```bash
ros2 service call /multi_bt/start std_srvs/srv/Trigger '{}'
ros2 topic echo /multi_bt/status
ros2 service call /multi_bt/stop std_srvs/srv/Trigger '{}'
```

## 검증 및 기존 데모

간단한 BT의 계약·상태·우선순위·승인 지연 취소 및 실행 환경 시험 28개와 패키지 빌드를 확인했습니다. 이전 공장 BT 재사용 방식의 시험 결과를 현재 구성의 실제 Webots 주행 검증으로 간주하지 않습니다. 외부 업무 할당 모듈과 연결한 전체 주행 검증은 별도로 필요합니다.

3대 실행 환경을 함께 사용하려면 `ros_packages/multirobot_bt/`와 `ros_packages/multirobot_sim/` 두 패키지를 추가합니다. [통합 실행 방법](../multirobot_sim/README.md)을 참고하세요.
