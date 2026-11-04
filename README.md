# py_bt_ros

**A Behaviour Tree (BT) based multi-robot coordination framework integrated with ROS 2**

## Overview

`py_bt_ros` is a modular framework that combines **Behaviour Trees** with **ROS 2** for autonomous multi-robot task coordination. It provides:

- **Flexible BT Architecture**: Define robot behaviors using XML-based Behaviour Tree specifications, editable with the visual editor [Groot2](https://www.behaviortree.dev/groot/)
- **Pluggable MRTA Algorithms**: Swap between different Multi-Robot Task Allocation plugins (Greedy, GRAPE, CBBA, Hungarian)
- **Distributed Decision Making**: Robots coordinate via ROS 2 messaging without centralized control
- **Simulation-Ready**: Works with [Webots](https://cyberbotics.com/) physics simulator and others within ROS 2

The framework is designed to be **scenario-agnostic** — easily define new scenarios by creating BT nodes and MRTA configurations.

## ROS perception packages

- [human_yolo](ros_packages/human_yolo/README.md): RGB-D human-dummy detection, trained weights and ROS detection/position topics for existing BTs. This package does not launch a simulator or control robot motion. Its Python dependencies are separate from the framework's root `requirements.txt`.

---

## Scenarios

### LIMO Factory Search

공장 1→2→3 구역을 이동하면서 매 틱 `IsPerson`을 확인한다. 발견하면 이동을
취소하고 사람의 지도 좌표를 기록한 뒤 SUCCESS로 종료한다. 현재는 모의 감지를 사용한다.
실행 명령: [scenarios/limo_factory_search/README.md](scenarios/limo_factory_search/README.md).

### LIMO Round Trip (Webots + Nav2)

`dev_chanseo`에 LIMO용 시나리오를 추가했다. XML BT와 Pygame 시각화를 사용하여
**목적지 이동 → 시뮬레이션 시간 10초 대기 → 출발점 복귀**를 수행한다.
시뮬레이터는 별도 `webots_ros2_limo` 레포에서 실행한다.

실행 명령과 구성 파일 → [scenarios/limo_round_trip/README.md](scenarios/limo_round_trip/README.md)

---

### 1. 🐢 [Turtle Catcher (Turtlesim)](scenarios/turtle_catcher/README.md)

**Lightweight BT + ROS 2 example using turtlesim**

- **Environment**: ROS 2 turtlesim (lightweight, no physics simulation needed)
- **Objective**: One turtle pursues a target turtle using BT-based navigation
- **Best For**: Quick prototyping, testing BT logic, learning the framework
- **Single Algorithm**: Simple greedy nearest-target approach

📍 For quick start → [See `scenarios/turtle_catcher/README.md`](scenarios/turtle_catcher/README.md)

---

### 2. 🔥 [Fire Suppression (Webots)](scenarios/simple/README.md)

**Multi-robot autonomous fire suppression using Webots physics simulator**

- **Environment**: Fire suppression arena with n Husky UGVs
- **Objective**: Robots collaboratively detect, approach, and suppress spreading fires
- **Algorithms**: Various decentralised multi-robot task allocation algorithms such as GRAPE, CBBA, and Hungarian
- **Features**: Network-aware task allocation, communication topology visualization, fire spread simulation
- **Validation Workflow**: Ported from [space-simulator](https://github.com/inmo-jang/space-simulator) — test algorithms there first, then validate with Webots physics simulation

📍 For detailed setup and usage → [See `scenarios/simple/README.md`](scenarios/simple/README.md)

---

## Maintainer

**Inmo Jang**  
Assistant Professor, Korea Aerospace University  
📧 [inmo.jang@kau.ac.kr](mailto:inmo.jang@kau.ac.kr)
