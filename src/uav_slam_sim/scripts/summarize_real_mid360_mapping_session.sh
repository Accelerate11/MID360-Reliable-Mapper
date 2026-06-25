#!/usr/bin/env bash
set -eo pipefail

WS=/home/accelerate/cuadc_ws
SESSION_DIR="${1:-}"

if [ -z "$SESSION_DIR" ]; then
  echo "Usage: $0 <session_dir>" >&2
  exit 2
fi

exec python3 "$WS/src/uav_slam_sim/scripts/summarize_mid360_mapping_session.py" "$SESSION_DIR"
