#!/usr/bin/env bash
set -eo pipefail

WS=/home/accelerate/cuadc_ws
BAG_PATH="${1:-}"
OUT_ROOT="${2:-$WS/mid360_mapping_replays}"

if [ -z "$BAG_PATH" ]; then
  echo "Usage: $0 <rosbag_dir> [output_root]" >&2
  exit 2
fi

if [ ! -f "$BAG_PATH/metadata.yaml" ]; then
  echo "Bag directory does not contain metadata.yaml: $BAG_PATH" >&2
  exit 2
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_NAME="$(basename "$BAG_PATH")"
OUT_DIR="$OUT_ROOT/replay_${BAG_NAME}_$STAMP"
LOG_DIR="$OUT_DIR/logs"
MAP_DIR="$OUT_DIR/maps"

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p "$LOG_DIR" "$MAP_DIR"
printf '%s\n' "$BAG_PATH" > "$OUT_DIR/source_bag.txt"

echo "Replaying MID360 mapping bag:"
echo "  bag: $BAG_PATH"
echo "  output: $OUT_DIR"
echo

"$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/stop_mid360_slam.sh" >/dev/null 2>&1 || true
sleep 1

ros2 launch uav_slam_sim real_mid360_mapping_view.launch.py \
  start_livox:=false \
  start_fast_lio:=true \
  start_rviz:=false \
  >"$LOG_DIR/replay_mapping.log" 2>&1 &
launch_pid=$!
export_pid=""
quality_pid=""

cleanup()
{
  if [ -n "$export_pid" ]; then
    kill "$export_pid" >/dev/null 2>&1 || true
    wait "$export_pid" >/dev/null 2>&1 || true
  fi
  if [ -n "$quality_pid" ]; then
    kill "$quality_pid" >/dev/null 2>&1 || true
    wait "$quality_pid" >/dev/null 2>&1 || true
  fi
  kill "$launch_pid" >/dev/null 2>&1 || true
  wait "$launch_pid" >/dev/null 2>&1 || true
  "$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/stop_mid360_slam.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

sleep 5

echo "Starting replay map exporter..."
python3 "$WS/src/uav_slam_sim/scripts/export_mid360_mapping_cloud.py" \
  --topic /fastlio_denoised_map \
  --output "$MAP_DIR/replayed_fastlio_denoised_map.pcd" \
  --report "$MAP_DIR/replayed_fastlio_denoised_map.json" \
  --duration 180.0 \
  --idle-timeout 3.0 \
  --z-min -20.0 \
  --z-max 10.0 \
  >"$LOG_DIR/export_map.log" 2>&1 &
export_pid=$!

echo "Starting replay quality sampler..."
"$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/check_real_mid360_mapping_quality.sh" \
  --duration 12 \
  >"$LOG_DIR/final_quality.txt" 2>&1 &
quality_pid=$!

echo "Playing raw MID360 topics from bag..."
ros2 bag play "$BAG_PATH" \
  --topics /livox/lidar /livox/imu \
  --disable-keyboard-controls \
  >"$LOG_DIR/rosbag_play.log" 2>&1

wait "$export_pid" || true
wait "$quality_pid" || true

echo "Analyzing replayed map..."
"$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/analyze_real_mid360_map.sh" \
  "$MAP_DIR/replayed_fastlio_denoised_map.pcd" \
  "$MAP_DIR" \
  >"$LOG_DIR/analyze_map.log" 2>&1 || true

"$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/evaluate_real_mid360_mapping_session.sh" \
  "$OUT_DIR" \
  >"$LOG_DIR/readiness.log" 2>&1 || true

"$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/summarize_real_mid360_mapping_session.sh" \
  "$OUT_DIR" \
  >"$LOG_DIR/summary.log" 2>&1 || true

echo "Replay complete:"
echo "  $OUT_DIR"
echo "  map: $MAP_DIR/replayed_fastlio_denoised_map.pcd"
echo "  report: $MAP_DIR/replayed_fastlio_denoised_map.json"
echo "  analysis: $MAP_DIR/replayed_fastlio_denoised_map.analysis.json"
echo "  snapshot: $MAP_DIR/replayed_fastlio_denoised_map.png"
echo "  readiness: $OUT_DIR/mapping_readiness.md"
echo "  summary: $OUT_DIR/summary.md"
