#!/bin/bash
set -e
DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
fi
export YOLO_CONFIG_DIR="$DEMO_ROOT/config"
exec "${YOLO_PYTHON:-python3}" "$DEMO_ROOT/scripts/detector.py" "$@"
