#!/usr/bin/env bash
set -eo pipefail

cd /home/accelerate/cuadc_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_slam_sim real_mid360_mapping_view.launch.py
