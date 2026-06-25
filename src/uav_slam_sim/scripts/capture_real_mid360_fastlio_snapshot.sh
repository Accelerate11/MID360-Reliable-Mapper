#!/usr/bin/env bash
set -o pipefail

WS=/home/accelerate/cuadc_ws
LOG="$WS/real_mid360_fastlio_run.log"
OUTPUT="${1:-$WS/real_mid360_lasermap_snapshot.png}"

cd "$WS" || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_slam_sim real_mid360_fastlio.launch.py > "$LOG" 2>&1 &
launch_pid=$!

cleanup()
{
  kill "$launch_pid" >/dev/null 2>&1 || true
  wait "$launch_pid" >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep 25

python3 "$WS/src/uav_slam_sim/scripts/capture_mid360_snapshot.py" \
  --topic /Laser_map \
  --output "$OUTPUT" \
  --timeout 20

tail -80 "$LOG" || true
