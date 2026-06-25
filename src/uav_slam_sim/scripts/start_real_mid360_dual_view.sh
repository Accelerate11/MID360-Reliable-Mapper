#!/usr/bin/env bash
set -eo pipefail

WS=/home/accelerate/cuadc_ws

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

"$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/stop_mid360_slam.sh" >/dev/null 2>&1 || true

exec ros2 launch uav_slam_sim real_mid360_dual_view.launch.py
