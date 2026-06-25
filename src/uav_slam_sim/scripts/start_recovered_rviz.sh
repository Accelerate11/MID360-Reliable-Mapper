#!/usr/bin/env bash
set -eo pipefail

WS=/home/accelerate/cuadc_ws
RVIZ_RUNTIME=/home/accelerate/rviz_runtime
RVIZ_CFG="$RVIZ_RUNTIME/real_mid360_mapping_light.rviz"

pkill -x rviz2 2>/dev/null || true
sleep 0.5

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p "$RVIZ_RUNTIME"
cp "$WS/install/uav_slam_sim/share/uav_slam_sim/config/real_mid360_mapping_light.rviz" "$RVIZ_CFG"
chmod u+w "$RVIZ_CFG"

echo "Starting RViz with WSLg software OpenGL recovery mode..."
echo "Normal RViz INFO lines include: Stereo is NOT SUPPORTED, OpenGL version."

exec env \
  DISPLAY=:0 \
  WAYLAND_DISPLAY= \
  QT_QPA_PLATFORM=xcb \
  QT_X11_NO_MITSHM=1 \
  QT_OPENGL=software \
  LIBGL_ALWAYS_SOFTWARE=1 \
  LIBGL_DRI3_DISABLE=1 \
  QT_SCALE_FACTOR=1 \
  rviz2 -d "$RVIZ_CFG"