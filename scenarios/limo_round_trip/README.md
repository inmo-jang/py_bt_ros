# LIMO 시뮬레이터 + py_bt_ros

`dev_chanseo`의 `main.py`, XML 트리, ROS 액션 노드, Pygame 시각화 구조를 사용한다.
`webots_ros2_limo`는 시뮬레이터와 Nav2를 담당하고, 이 레포는 BT를 실행한다.
Turtlesim이나 TurtleBot 패키지는 이 시나리오에 필요하지 않다.

```xml
<Sequence>
  <MoveToGoal pose_key="goal_pose" />
  <WaitForSecond seconds="10" />
  <MoveToGoal pose_key="home_pose" />
</Sequence>
```

## 최초 준비 (Ubuntu 24.04 / ROS 2 Jazzy)

두 레포를 같은 워크스페이스에 둔다. 시뮬레이터 레포에도 `bt_round_trip_launch.py`,
Nav2/Jazzy 호환 설정과 RViz 설정이 필요하므로 두 레포의 변경 사항을 함께 공유한다.

```text
~/ros2_ws/src/py_bt_ros              # dev_chanseo
~/ros2_ws/src/webots_ros2_limo
```

ROS 2와 Webots는 시뮬레이터 레포 README에 따라 설치한다. BT용 기본 의존성:

```bash
sudo apt install python3-pygame python3-yaml ros-jazzy-nav2-msgs ros-jazzy-tf2-ros
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select webots_ros2_limo
```

`py_bt_ros`는 소스에서 실행하므로 별도 colcon 빌드가 필요 없다.

## 실행

터미널 1 — Webots 실행:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=42
ros2 launch webots_ros2_limo robot_launch.py
```

Webots가 재생되고 로봇·센서가 준비된 다음 터미널 2에서 실행:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=42
ros2 launch webots_ros2_limo bt_round_trip_launch.py
```

Nav2, AMCL, RViz와 **Pygame BT 창**이 열린다. 시뮬레이터와 Nav2는 순서대로
시작하며, Nav2가 이미 실행 중이면 마지막 명령에 `nav:=false`를 붙인다.
RViz Navigation 2 패널이 active가 될 때까지 기다린다.
기본 월드에서는 초기 위치를 자동 설정한다. 지도가 어긋나면 **2D Pose Estimate**로 보정한다.

RViz의 **2D Goal Pose**로 목표 위치와 방향을 지정하면 왕복 임무가 시작된다.
이 도구는 `/bt/goal_pose`로 연결되어 있다. Navigation 패널의 직접 목표 전송은 BT를 거치지 않는다.

또는 터미널 3에서 목표를 보낸다 (기본 월드의 가까운 목표 예시):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=42
ros2 topic pub --once -w 1 /bt/goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 0.8, y: 0.0}, orientation: {w: 1.0}}}'
```

관찰할 순서:

```text
IDLE → STARTING → MOVING_TO_GOAL → WAITING → RETURNING_HOME → COMPLETE
```

- 목표 수신 시 현재 `map → base_link` 위치와 방향을 복귀 지점으로 저장한다.
- 도착 판정은 Nav2의 위치·방향 허용 오차를 따른다.
- 대기는 **시뮬레이션 시간 10초**다. Webots 재생을 멈추면 대기도 멈춘다.
- 진행 중 새 목표는 무시한다. 완료·실패·취소 후 새 목표로 다시 실행할 수 있다.
- 잘못된 프레임/좌표 또는 준비되지 않은 TF는 `REJECTED`, Nav2 실패는 `FAILED`로 보고한다.
- Pygame에서 파란색은 실행 중, 초록색은 성공, 빨간색은 실패다.
- `P`는 BT tick만 일시정지한다. 이동 취소는 아래 토픽을 사용한다.
- BT 창을 닫거나 `Q`/Esc/Ctrl+C로 종료하면 실행 중인 Nav2 목표에 취소를 요청한다.

상태 확인:

```bash
ros2 topic echo /bt/mission_status std_msgs/msg/String --qos-durability transient_local
ros2 topic echo /bt/home_pose geometry_msgs/msg/PoseStamped --qos-durability transient_local
```

임무 취소 (BT tick이 실행 중일 때):

```bash
ros2 topic pub --once -w 1 /bt/cancel std_msgs/msg/Empty '{}'
```

BT 창을 먼저 종료하고 시뮬레이터를 종료한다. 모든 터미널에서 같은 `ROS_DOMAIN_ID`를 사용한다.

## BT 수정

- `default_bt.xml`: 실행 순서와 대기 시간. `seconds="10"`을 바꾼다.
- `bt_nodes.py`: `MoveToGoal`, `WaitForSecond` 구현과 노드 등록.
- `agent.py`: 목표 수신, 출발점 저장, 상태 보고, 반복 임무와 종료 처리.
- `config_sim.yaml`: 토픽·액션 이름, TF 프레임, 타임아웃, 시각화 설정.

시뮬레이터/Nav2가 이미 실행 중일 때 BT만 직접 실행할 수도 있다:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
cd ~/ros2_ws/src/py_bt_ros
python3 main.py --config scenarios/limo_round_trip/config_sim.yaml
```

BT 레포가 다른 경로라면:

```bash
ros2 launch webots_ros2_limo bt_round_trip_launch.py \
  py_bt_ros_root:=/absolute/path/to/py_bt_ros
```

## 검증

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/src/py_bt_ros
python3 -m unittest discover -s tests -v
```

10초 경계, 시뮬레이션 일시정지, 반복 실행, Nav2 목표 거절·실패·타임아웃,
지연된 목표 수락 시 취소를 검증한다. 실제 Webots + Jazzy Nav2 왕복도 확인했다.
학습용 Cyclo/OpenMANIPULATOR 컨테이너는 이 시뮬레이션 실행에 필요하지 않다.
