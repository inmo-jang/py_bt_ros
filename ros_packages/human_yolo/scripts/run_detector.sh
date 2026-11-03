#!/usr/bin/env bash
set -euo pipefail
PACKAGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$PACKAGE_DIR${PYTHONPATH:+:$PYTHONPATH}"
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-${XDG_CACHE_HOME:-/tmp}/human_yolo}"
exec "${YOLO_PYTHON:-python3}" -m human_yolo.detector --ros-args \
  --params-file "$PACKAGE_DIR/config/detector.yaml" \
  -p "model_path:=$PACKAGE_DIR/models/best.pt" "$@"
