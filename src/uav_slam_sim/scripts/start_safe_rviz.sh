#!/usr/bin/env bash
set -eo pipefail

WS=/home/accelerate/cuadc_ws
RVIZ_RUNTIME=/home/accelerate/rviz_runtime
RVIZ_CFG="$RVIZ_RUNTIME/real_mid360_mapping_safe.rviz"

pkill -x rviz2 2>/dev/null || true
sleep 0.5

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p "$RVIZ_RUNTIME"
cp "$WS/src/uav_slam_sim/config/real_mid360_mapping_safe.rviz" "$RVIZ_CFG"
chmod u+w "$RVIZ_CFG"

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