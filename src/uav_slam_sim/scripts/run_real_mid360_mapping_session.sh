#!/usr/bin/env bash
set -eo pipefail

WS=/home/accelerate/cuadc_ws
SESSION_SECONDS="${1:-0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SESSION_ROOT="${MID360_SESSION_ROOT:-$WS/mid360_mapping_sessions}"
SESSION_DIR="$SESSION_ROOT/session_$STAMP"
LOG_DIR="$SESSION_DIR/logs"
MAP_DIR="$SESSION_DIR/maps"
BAG_DIR="$SESSION_DIR/bag"

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p "$LOG_DIR" "$MAP_DIR" "$BAG_DIR"

echo "MID360 mapping session:"
echo "  session_dir: $SESSION_DIR"
echo "  duration: ${SESSION_SECONDS}s (0 means until Ctrl+C)"
echo

"$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/stop_mid360_slam.sh" >/dev/null 2>&1 || true
sleep 1

ros2 launch uav_slam_sim real_mid360_mapping_view.launch.py >"$LOG_DIR/live_mapping.log" 2>&1 &
launch_pid=$!

record_pid=""
cleanup_done=0

cleanup()
{
  if [ "$cleanup_done" -eq 1 ]; then
    return
  fi
  cleanup_done=1

  echo
  echo "Stopping session processes..."
  if [ -n "$record_pid" ]; then
    kill "$record_pid" >/dev/null 2>&1 || true
    wait "$record_pid" >/dev/null 2>&1 || true
  fi

  echo "Exporting final map..."
  python3 "$WS/src/uav_slam_sim/scripts/export_mid360_mapping_cloud.py" \
    --topic /fastlio_denoised_map \
    --output "$MAP_DIR/final_fastlio_denoised_map.pcd" \
    --report "$MAP_DIR/final_fastlio_denoised_map.json" \
    --duration 2.0 \
    --z-min -20.0 \
    --z-max 10.0 \
    >"$LOG_DIR/export_map.log" 2>&1 || true

  echo "Analyzing final map..."
  "$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/analyze_real_mid360_map.sh" \
    "$MAP_DIR/final_fastlio_denoised_map.pcd" \
    "$MAP_DIR" \
    >"$LOG_DIR/analyze_map.log" 2>&1 || true

  "$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/check_real_mid360_mapping_quality.sh" \
    --duration 2 \
    >"$LOG_DIR/final_quality.txt" 2>&1 || true

  "$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/evaluate_real_mid360_mapping_session.sh" \
    "$SESSION_DIR" \
    >"$LOG_DIR/readiness.log" 2>&1 || true

  "$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/summarize_real_mid360_mapping_session.sh" \
    "$SESSION_DIR" \
    >"$LOG_DIR/summary.log" 2>&1 || true

  kill "$launch_pid" >/dev/null 2>&1 || true
  wait "$launch_pid" >/dev/null 2>&1 || true
  "$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/stop_mid360_slam.sh" >/dev/null 2>&1 || true

  echo "Session complete:"
  echo "  $SESSION_DIR"
  echo "  map: $MAP_DIR/final_fastlio_denoised_map.pcd"
  echo "  report: $MAP_DIR/final_fastlio_denoised_map.json"
  echo "  analysis: $MAP_DIR/final_fastlio_denoised_map.analysis.json"
  echo "  snapshot: $MAP_DIR/final_fastlio_denoised_map.png"
  echo "  quality: $LOG_DIR/final_quality.txt"
  echo "  readiness: $SESSION_DIR/mapping_readiness.md"
  echo "  summary: $SESSION_DIR/summary.md"
}

trap cleanup EXIT INT TERM

echo "Waiting for live mapping to initialize..."
sleep 8

echo "Starting rosbag record..."
ros2 bag record \
  -o "$BAG_DIR/real_mid360_mapping" \
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
  /tf_static \
  >"$LOG_DIR/rosbag_record.log" 2>&1 &
record_pid=$!

echo
echo "Live quality check:"
"$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/check_real_mid360_mapping_quality.sh" \
  --duration 3 || true
echo

if [ "$SESSION_SECONDS" -gt 0 ]; then
  echo "Recording for ${SESSION_SECONDS}s..."
  sleep "$SESSION_SECONDS"
else
  echo "Recording until Ctrl+C. Move MID360 slowly, then press Ctrl+C to finish and export."
  while true; do
    sleep 5
  done
fi
