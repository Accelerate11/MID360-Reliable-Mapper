#!/usr/bin/env bash
set -eo pipefail

cd /home/accelerate/cuadc_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch uav_slam_sim synthetic_mid360_fastlio_rviz.launch.py
