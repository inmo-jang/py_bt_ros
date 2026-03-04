# Scenario: Multi-Robot Fire Suppression (Webots)

## Overview

This is a **multi-robot fire suppression scenario** running on Webots simulator, showcasing distributed task allocation algorithms.

The goal is to demonstrate that the BT (Behaviour Tree)-based multi-robot architecture implemented in space-simulator can run identically in a real physics simulator (Webots). The BT nodes, configuration files, and execution patterns are all kept consistent with space-simulator.

### Scenario Description

- Environment: Webots `fire_suppression.wbt` world (indoor fire suppression map)
- Robots: **Fire_UGV_N** (Husky-based UGVs)
- Objective: Each robot autonomously detects, approaches, and suppresses fires

> This scenario also supports a **Limo variant**. See the **Limo ver** section below.

Each robot is driven by an independent BT. It selects a fire to suppress based on a pluggable MRTA algorithm, approaches it, and repeatedly reduces its radius until the fire is extinguished. Fires spread to surrounding areas over time.

The following **Multi-Robot Task Allocation (MRTA) plugins** are available:

| Plugin | Config | Description |
|--------|--------|-------------|
| **Greedy** | `greedy.yaml` | Each robot independently picks the nearest available fire |
| **GRAPE** | `grape.yaml` | Distributed coalition formation via game-theoretic partition |
| **CBBA** | `cbba.yaml` | Consensus-based bundle algorithm for multi-task assignment |
| **Hungarian** | `hungarian.yaml` | Distributed Hungarian algorithm with network consensus for optimal task-agent matching |
---


![Demo](demo.gif)


## System Architecture

```
┌───────────────────────────────────────────────────────┐
│              Webots Simulation                        │
│  ┌────────────────┐    ┌────────────────────────────┐ │
│  │robot_supervisor│    │  world_supervisor          │ │
│  │ (pose_world)   │    │ (fire list, spawn/suppress)│ │
│  └────────────────┘    └────────────────────────────┘ │
└───────────────────────────────────────────────────────┘
         │ /Fire_UGV_N/pose_world           │ /world/fire/list
         │ /Fire_UGV_N/scan                 │ /world/fire/reduce
         ▼                                  ▼
┌──────────────────────┐           ┌──────────────────────┐
│  nav_action_server   │           │    BT Runner (x3)    │
│  (per robot)         │◄──────────│  GatherLocalInfo     │
│  NavigateToPose      │           │  AssignTask          │
│  Gap-based avoidance │           │  MoveToTarget        │
│  Rotation Shim       │           │  IsArrivedAtTarget   │
└──────────────────────┘           │  ExecuteTask         │
         │ /Fire_UGV_N/cmd_vel     │  IsTaskCompleted     │
         ▼                         │  Explore             │
    Webots Robot                   └──────────────────────┘
```

---

## Behaviour Tree

```
ReactiveSequence
├── GatherLocalInfo              ← Subscribe to pose_world and fire/list
└── ReactiveSequence
    ├── ReactiveFallback
    │   ├── AssignTask           ← Select nearest fire (⚠️ needs refinement)
    │   └── Explore              ← Random exploration when no fires available
    └── ReactiveFallback
        ├── IsTaskCompleted      ← SUCCESS if assigned fire is gone from the list
        └── ReactiveSequence
            ├── ReactiveFallback
            │   ├── IsArrivedAtTarget  ← Check if within fire radius
            │   └── MoveToTarget       ← Navigate via NavigateToPose action
            └── ExecuteTask            ← Publish fire_id to /world/fire/reduce
```

---

## ROS Interface

### World Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/world/fire/list` | `std_msgs/String` (JSON) | Pub | Active fire list `[{task_id, x, y, z, radius}, ...]` |
| `/world/fire/summary` | `std_msgs/UInt16MultiArray` | Pub | `[total_spawned, active, suppressed]` |
| `/world/base/pose` | `geometry_msgs/PoseStamped` | Pub | Base location |
| `/world/fire/spawn` | `std_srvs/Empty` (service) | Sub | Spawn a fire at a random location |
| `/world/fire/spawn_custom` | `std_msgs/Float64MultiArray` | Sub | Spawn fire at `[x, y, radius]` |
| `/world/fire/suppress` | `std_msgs/String` | Sub | Immediately remove fire (`"Fire_1"`) |
| `/world/fire/reduce` | `std_msgs/String` | Sub | Reduce fire radius by 0.1 (`"Fire_1"`) |
| `/world/visualisation/comm_topology` | `visualization_msgs/MarkerArray` | Pub | Communication links between robots as green lines (**debug mode only**) |

### Per-Robot Topics (`N` = 1, 2, 3)

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/Fire_UGV_N/pose_world` | `geometry_msgs/PoseStamped` | Pub | Robot position in Webots world frame |
| `/Fire_UGV_N/scan` | `sensor_msgs/LaserScan` | Pub | LiDAR scan data |
| `/Fire_UGV_N/cmd_vel` | `geometry_msgs/TwistStamped` | Sub | Velocity command |
| `/Fire_UGV_N/navigate_to_pose` | `nav2_msgs/NavigateToPose` (action) | Server | Navigation action to a goal pose |

---

## How to Run

### Prerequisites

```bash
cd scenarios/simple/sim_webots
colcon build --symlink-install
source install/local_setup.bash
```

### Step 1: Launch Webots Simulation

```bash
# Normal run
ros2 launch webots_ros2_husky robot_launch.py

# With debug visualisation topics (e.g. comm_topology)
ros2 launch webots_ros2_husky robot_launch.py debug:=true
```

### Step 2: Run Action Servers (one terminal per robot)

```bash
python3 scenarios/simple/action_servers/nav_action_server.py --ns /Fire_UGV_1
python3 scenarios/simple/action_servers/nav_action_server.py --ns /Fire_UGV_2
python3 scenarios/simple/action_servers/nav_action_server.py --ns /Fire_UGV_3
```

> **(Optional) Launch all robots at once with a shell script**
>
> ```bash
> # Default: 10 robots (Fire_UGV_1 ~ Fire_UGV_10)
> bash scenarios/simple/scripts/run_action_servers.sh
>
> # Or specify the number of robots
> bash scenarios/simple/scripts/run_action_servers.sh 3
> ```
>
> Press **Ctrl+C** to stop all action servers at once.

### Step 3: Run BT Runners (one terminal per robot)

Use `--ns` to specify each robot's namespace and `--config` to select the MRTA algorithm.

```bash
# Greedy
python3 main.py --config=scenarios/simple/configs/greedy.yaml --ns /Fire_UGV_1

# GRAPE
python3 main.py --config=scenarios/simple/configs/grape.yaml --ns /Fire_UGV_1

# CBBA
python3 main.py --config=scenarios/simple/configs/cbba.yaml --ns /Fire_UGV_1

# Hungarian
python3 main.py --config=scenarios/simple/configs/hungarian.yaml --ns /Fire_UGV_1
```

> **(Optional) Launch all robots at once with a shell script**
>
> ```bash
> # Usage: bash run_bt_runners.sh [NUM_ROBOTS] [CONFIG_NAME]
>
> # Default: 10 robots, grape.yaml
> bash scenarios/simple/scripts/run_bt_runners.sh
>
> # 6 robots with GRAPE
> bash scenarios/simple/scripts/run_bt_runners.sh 6 grape.yaml
>
> # 6 robots with CBBA
> bash scenarios/simple/scripts/run_bt_runners.sh 6 cbba.yaml
>
> # 6 robots with Greedy
> bash scenarios/simple/scripts/run_bt_runners.sh 6 greedy.yaml
>
> # 6 robots with Hungarian
> bash scenarios/simple/scripts/run_bt_runners.sh 6 hungarian.yaml
> ```
>
> Press **Ctrl+C** to stop all BT runners at once.

---

## How to Run (Limo ver)

This section describes how to run the same scenario with **Webots Limo robots**.

### Needs modification

The overall BT logic and MRTA plugins are the same. The main differences are:

1) Webots launch package

- Husky (default guide above): `webots_ros2_husky`
- Limo: `webots_ros2_limo`

2) Robot namespace prefix

- Husky: `/Fire_UGV_N` (e.g. `/Fire_UGV_1`)
- Limo: `/Limo_N` (e.g. `/Limo_1`)

Where this matters:

- **BT runner**: pass `--ns /Limo_N` to `main.py`
- **Action server**: pass `--ns /Limo_N` to `scenarios/simple/action_servers/nav_action_server.py`
- **Config default namespace (optional)**: update `agent.namespaces` in `scenarios/simple/configs/*.yaml`
    - From: `/Fire_UGV_1` → To: `/Limo_1`
    - (You can always override it with `--ns` per process)
- **Shell scripts (optional)**:
    - `scenarios/simple/scripts/run_action_servers.sh`: change `/Fire_UGV_$i` → `/Limo_$i`
    - `scenarios/simple/scripts/run_bt_runners.sh`: change `/Fire_UGV_$i` → `/Limo_$i`

3) RViz config / pose topic

- Use `scenarios/simple/sim_webots/webots_ros2_limo/default_view.rviz`
- Pose display topic becomes `/Limo_N/pose_world`

4) Navigation tuning (optional)

`scenarios/simple/action_servers/nav_action_server.py` contains Limo-friendly velocity hints (e.g. `max_linear_vel`, `max_angular_vel`). Adjust these if Limo feels too aggressive/slow.

### Step 1: Launch Webots Simulation (Limo)

```bash
ros2 launch webots_ros2_limo robot_launch.py

# With debug visualisation topics (e.g. comm_topology)
ros2 launch webots_ros2_limo robot_launch.py debug:=true
```

### Step 2: Run Action Servers (one terminal per robot)

```bash
python3 scenarios/simple/action_servers/nav_action_server.py --ns /Limo_1
python3 scenarios/simple/action_servers/nav_action_server.py --ns /Limo_2
python3 scenarios/simple/action_servers/nav_action_server.py --ns /Limo_3
```

### Step 3: Run BT Runners (one terminal per robot)

```bash
# Greedy
python3 main.py --config=scenarios/simple/configs/greedy.yaml --ns /Limo_1

# GRAPE
python3 main.py --config=scenarios/simple/configs/grape.yaml --ns /Limo_1

# CBBA
python3 main.py --config=scenarios/simple/configs/cbba.yaml --ns /Limo_1

# Hungarian
python3 main.py --config=scenarios/simple/configs/hungarian.yaml --ns /Limo_1
```

### (Optional) RViz Visualisation (Limo)

```bash
rviz2 -d scenarios/simple/sim_webots/webots_ros2_limo/default_view.rviz
```

### Manual Test Commands

```bash
# Immediately remove a fire
ros2 topic pub --once /world/fire/suppress std_msgs/msg/String "{data: 'Fire_1'}"

# Reduce fire radius by 0.1
ros2 topic pub --once /world/fire/reduce std_msgs/msg/String "{data: 'Fire_1'}"

# Spawn fire at a specific location (x, y, radius)
ros2 topic pub --once /world/fire/spawn_custom std_msgs/msg/Float64MultiArray "{data: [5.0, 3.0, 1.5]}"

# Spawn fire at a random location
ros2 service call /world/fire/spawn std_srvs/srv/Empty
```

---
### (Optional) RViz Visualisation

```
rviz2 -d scenarios/simple/sim_webots/webots_ros2_husky/default_view.rviz
```

Launch Webots with `debug:=true` (Step 1), then open RViz and add the following displays:

| Display | Topic | Notes |
|---------|-------|-------|
| **PoseStamped** | `/Fire_UGV_N/pose_world` | Set **Fixed Frame** to `world` |
| **MarkerArray** | `/world/visualisation/comm_topology` | Green lines between robots within comm_radius |
| **MarkerArray** | `/world/visualisation/task_assignment` | Cyan lines: robot → currently assigned fire |
| **MarkerArray** | `/world/visualisation/task_plan` | Orange lines: robot → planned task sequence (CBBA only) |

All visualisation markers have a 1 s lifetime and auto-expire when robots go silent.


---

## File Structure

```
simple/
├── README.md
├── default_bt.xml              # BT structure definition (Groot compatible)
├── bt_nodes.py                 # Custom BT node implementations
├── action_servers/
│   └── nav_action_server.py   # NavigateToPose action server (gap-based avoidance)
├── configs/
│   ├── greedy.yaml             # Greedy MRTA config
│   ├── grape.yaml              # GRAPE MRTA config
│   └── cbba.yaml               # CBBA MRTA config
└── sim_webots/
    ├── webots_ros2_husky/
    │   ├── launch/
    │   │   └── robot_launch.py               # Webots launch + robot driver setup
    │   ├── controllers/
    │   │   ├── robot_supervisor/
    │   │   │   └── robot_supervisor.py       # Publishes robot poses from Webots
    │   │   └── world_supervisor/
    │   │       └── world_supervisor.py       # Fire management (spawn/suppress/reduce/spread)
    │   ├── worlds/
    │   │   └── fire_suppression.wbt          # Webots world file (Husky)
    │   └── protos/
    │       └── Fire.proto                    # Fire visualization PROTO
    └── webots_ros2_limo/
        ├── launch/
        │   └── robot_launch.py               # Webots launch + robot driver setup (Limo)
        ├── controllers/
        │   ├── robot_supervisor/
        │   │   └── robot_supervisor.py       # Publishes robot poses from Webots
        │   └── world_supervisor/
        │       └── world_supervisor.py       # Fire management (spawn/suppress/reduce/spread)
        └── worlds/
            └── fire_suppression.wbt          # Webots world file (Limo)
```

---

## Key Parameters

### nav_action_server.py

| Parameter | Default | Description |
|-----------|---------|-------------|
| `control_rate` | 30.0 Hz | Control loop frequency |
| `max_linear_vel` | 1.0 m/s | Maximum forward speed |
| `max_angular_vel` | 0.5 rad/s | Maximum rotation speed |
| `goal_tolerance` | 0.3 m | Distance threshold for goal arrival |
| `rotation_shim_threshold` | 0.2 rad | Rotate in place when heading error exceeds this |
| `obstacle_distance_stop` | 0.5 m | Emergency stop distance |
| `obstacle_distance_slow` | 5.0 m | Deceleration start distance |
| `min_passable_dist` | 2.0 m | Minimum distance to qualify as a passable gap |

### world_supervisor.py (fire spread)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `fire_spread_interval` | 5.0 s | Spread attempt interval |
| `fire_spread_probability` | 0.8 | Probability of spread per attempt |
| `fire_spread_distance_min/max` | 2.0~5.0 m | Spread distance range |
| `fire_max_count` | 20 | Maximum number of simultaneous fires |

---

## Known Issues / TODO

- **`Explore`**: Falls back to ±10m default map bounds if `x_min/max`, `y_min/max` are not defined in the config.
