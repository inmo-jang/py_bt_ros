#!/usr/bin/env bash
set -euo pipefail
DEMO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$DEMO_DIR/simulation/webots_ros2_limo${PYTHONPATH:+:$PYTHONPATH}"
export YOLO_CONFIG_DIR="$DEMO_DIR/config"
exec "${YOLO_PYTHON:-python3}" "$DEMO_DIR/scripts/detector.py" --ros-args -r __node:=limo_yolo_human "$@"
