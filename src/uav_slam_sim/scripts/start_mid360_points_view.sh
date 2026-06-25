#!/usr/bin/env bash

exec > /tmp/mid360_points_view.log 2>&1

cd /home/accelerate/cuadc_ws
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
source install/setup.bash

exec ros2 launch uav_slam_sim mid360_points_view.launch.py
