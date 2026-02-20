# Example: Fire & Rescue

## Overview

This scenario demonstrates a multi-robot fire rescue and firefighting mission using Behaviour Trees and ROS 2. The system consists of multiple autonomous robots coordinated by a central leader through a hierarchical Behaviour Tree architecture:

- **Leader (Mission-level BT)**: Manages the overall mission planning and task assignment for all follower robots
- **Fire UGVs** (2 units): Ground vehicles equipped to suppress fires and navigate to fire hotspots
- **Rescue UGV** (1 unit): Ground vehicle specialized in rescue operations
- **UAVs** (2 units): Aerial vehicles (drones) for exploration and fire detection


![Demo](demo.gif)
[Watch the demo on YouTube](https://youtu.be/VtCJ-DGZtls)

## How to Run

### 1. Run the Webots Simulation

```bash
# Navigate to the `example_webots_fire_rescue/webots_sim_pkg` directory
cd scenarios/example_webots_fire_rescue/webots_sim_pkg

# Build the workspace
colcon build --symlink-install

# Source the installed package environment
source install/local_setup.bash

# Launch the Webots simulation
ros2 launch webots_ros2_husky robot_launch.py
```

### 2. Run py_bt_ros

#### Run the Action Servers

```bash
# Run the Fire UGV Action Server:
python3 scenarios/example_simple/action_servers/nav_action_server.py --ns /Fire_UGV_1

python3 scenarios/example_simple/action_servers/nav_action_server.py --ns /Fire_UGV_2

# Run the Rescue UGV Action Server:
python3 scenarios/example_simple/action_servers/nav_action_server.py --ns /Rescue_UGV_1
```

#### Run the Followers

```bash
# Run the Rescue UGV Follower:
python3 main.py --config=scenarios/example_webots_fire_rescue/configs/config_rescue_ugv_1.yaml

# Run the Fire UGV Follower:
python3 main.py --config=scenarios/example_webots_fire_rescue/configs/config_fire_ugv_1.yaml

python3 main.py --config=scenarios/example_webots_fire_rescue/configs/config_fire_ugv_2.yaml
```

```bash
# Run the UAV Followers:
python3 main.py --config=scenarios/example_webots_fire_rescue/configs/config_uav_1.yaml

python3 main.py --config=scenarios/example_webots_fire_rescue/configs/config_uav_2.yaml
```


