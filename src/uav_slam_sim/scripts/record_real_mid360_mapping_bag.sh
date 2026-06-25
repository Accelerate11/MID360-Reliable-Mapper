#!/usr/bin/env bash
set -eo pipefail

WS=/home/accelerate/cuadc_ws
OUT_ROOT="${1:-$WS/mid360_mapping_bags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$OUT_ROOT/real_mid360_mapping_$STAMP"

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p "$OUT_ROOT"

echo "Recording MID360 mapping bag:"
echo "  $OUT_DIR"
echo
echo "Stop recording with Ctrl+C."

exec ros2 bag record \
  -o "$OUT_DIR" \
  /livox/lidar \
  /livox/imu \
  /cloud_registered \
  /cloud_registered_filtered \
  /fastlio_denoised_map \
  /fastlio_occupancy_grid \
  /fastlio_occupancy_cells \
  /Odometry \
  /path \
  /tf \
  /tf_static
