#!/usr/bin/env bash
set -o pipefail

WS=/home/accelerate/cuadc_ws
LOG="$WS/real_mid360_fastlio_run.log"

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

sleep 15

echo "=== processes ==="
ps -eo pid,ppid,comm,args | grep -E 'livox_ros_driver2|fastlio_mapping|ros2 launch uav_slam_sim' | grep -v grep || true

echo "=== direct topic probe ==="
python3 "$WS/src/uav_slam_sim/scripts/probe_real_mid360_topics.py" || true

echo "=== launch log tail ==="
tail -120 "$LOG" || true
