#!/usr/bin/env bash
set -eo pipefail

WS=/home/accelerate/cuadc_ws
MAP_A="${1:-}"
MAP_B="${2:-}"
OUT_JSON="${3:-}"

if [ -z "$MAP_A" ] || [ -z "$MAP_B" ]; then
  echo "Usage: $0 <map_a.pcd> <map_b.pcd> [output_compare.json]" >&2
  exit 2
fi

if [ ! -f "$MAP_A" ]; then
  echo "Map A not found: $MAP_A" >&2
  exit 2
fi

if [ ! -f "$MAP_B" ]; then
  echo "Map B not found: $MAP_B" >&2
  exit 2
fi

if [ -z "$OUT_JSON" ]; then
  OUT_JSON="$(dirname "$MAP_B")/$(basename "$MAP_B" .pcd).compare.json"
fi

OUT_MD="${OUT_JSON%.json}.md"

python3 "$WS/src/uav_slam_sim/scripts/compare_mid360_pcd_maps.py" \
  "$MAP_A" \
  "$MAP_B" \
  --voxel-size 0.20 \
  --report "$OUT_JSON" \
  --markdown "$OUT_MD"
