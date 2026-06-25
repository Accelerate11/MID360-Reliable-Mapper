#!/usr/bin/env bash
set -eo pipefail

WS=/home/accelerate/cuadc_ws

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

exec python3 "$WS/src/uav_slam_sim/scripts/check_mid360_mapping_quality.py" "$@"
