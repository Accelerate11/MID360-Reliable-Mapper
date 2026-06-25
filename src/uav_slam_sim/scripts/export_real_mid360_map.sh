#!/usr/bin/env bash
set -eo pipefail

WS=/home/accelerate/cuadc_ws
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${1:-$WS/mid360_mapping_outputs}"
OUT_PCD="$OUT_DIR/real_mid360_fastlio_map_$STAMP.pcd"

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

exec python3 "$WS/src/uav_slam_sim/scripts/export_mid360_mapping_cloud.py" \
  --topic /fastlio_denoised_map \
  --output "$OUT_PCD" \
  --duration 3.0 \
  --z-min -20.0 \
  --z-max 10.0
