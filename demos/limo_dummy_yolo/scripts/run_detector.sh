#!/usr/bin/env bash
set -euo pipefail
DEMO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$DEMO_DIR${PYTHONPATH:+:$PYTHONPATH}"
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-${XDG_CACHE_HOME:-/tmp}/human_yolo}"
exec "${YOLO_PYTHON:-python3}" -m human_yolo.detector --ros-args \
  --params-file "$DEMO_DIR/config/detector.yaml" \
  -p "model_path:=$DEMO_DIR/models/best.pt" "$@"
