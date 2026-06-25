#!/usr/bin/env bash
set -eo pipefail

WS=/home/accelerate/cuadc_ws
SESSION_DIR="${1:-}"

if [ -z "$SESSION_DIR" ]; then
  echo "Usage: $0 <mapping_session_or_replay_dir>" >&2
  exit 2
fi

if [ ! -d "$SESSION_DIR" ]; then
  echo "Session directory not found: $SESSION_DIR" >&2
  exit 2
fi

python3 "$WS/src/uav_slam_sim/scripts/evaluate_mid360_mapping_readiness.py" \
  "$SESSION_DIR" \
  --output "$SESSION_DIR/mapping_readiness.json" \
  --markdown "$SESSION_DIR/mapping_readiness.md"
