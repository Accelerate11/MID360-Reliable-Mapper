#!/usr/bin/env bash
set -eo pipefail

CONFIG="${1:-/home/accelerate/rviz_runtime/real_mid360_icp_simple.rviz}"

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
export QT_SCALE_FACTOR=1

rviz2 -d "$CONFIG" &
rviz_pid=$!
sleep 4
wmctrl -r RViz -b add,above || true
wmctrl -r RViz -e 0,80,60,1100,760 || true
xdotool search --name RViz windowactivate %@ || true
wait "$rviz_pid"
