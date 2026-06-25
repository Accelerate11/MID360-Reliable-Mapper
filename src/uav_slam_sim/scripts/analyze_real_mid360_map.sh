#!/usr/bin/env bash
set -eo pipefail

WS=/home/accelerate/cuadc_ws
PCD_PATH="${1:-}"
OUT_DIR="${2:-}"

if [ -z "$PCD_PATH" ]; then
  echo "Usage: $0 <map.pcd> [output_dir]" >&2
  exit 2
fi

if [ ! -f "$PCD_PATH" ]; then
  echo "PCD file not found: $PCD_PATH" >&2
  exit 2
fi

if [ -z "$OUT_DIR" ]; then
  OUT_DIR="$(dirname "$PCD_PATH")"
fi

base="$(basename "$PCD_PATH" .pcd)"
mkdir -p "$OUT_DIR"

python3 "$WS/src/uav_slam_sim/scripts/analyze_mid360_pcd_map.py" \
  "$PCD_PATH" \
  --report "$OUT_DIR/${base}.analysis.json" \
  --snapshot "$OUT_DIR/${base}.png"
