#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
if [ -f /home/accelerate/cuadc_ws/install/setup.bash ]; then
  source /home/accelerate/cuadc_ws/install/setup.bash
fi

unset WAYLAND_DISPLAY
export DISPLAY="${DISPLAY:-:0}"
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export QT_OPENGL=software
export LIBGL_ALWAYS_SOFTWARE=1
export LIBGL_DRI3_DISABLE=1
export QT_SCALE_FACTOR=1

exec rviz2 -d /home/accelerate/rviz_runtime/real_mid360_mapping_light.rviz
