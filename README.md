# py_bt_ros

**A Behaviour Tree (BT) based multi-robot coordination framework integrated with ROS 2**

## Overview

`py_bt_ros` is a modular framework that combines **Behaviour Trees** with **ROS 2** for autonomous multi-robot task coordination. It provides:

- **Flexible BT Architecture**: Define robot behaviors using XML-based Behaviour Tree specifications, editable with the visual editor [Groot2](https://www.behaviortree.dev/groot/)
- **Pluggable MRTA Algorithms**: Swap between different Multi-Robot Task Allocation plugins (Greedy, GRAPE, CBBA, Hungarian)
- **Distributed Decision Making**: Robots coordinate via ROS 2 messaging without centralized control
- **Simulation-Ready**: Works with [Webots](https://cyberbotics.com/) physics simulator and others within ROS 2

The framework is designed to be **scenario-agnostic** — easily define new scenarios by creating BT nodes and MRTA configurations.

---

## Scenarios

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