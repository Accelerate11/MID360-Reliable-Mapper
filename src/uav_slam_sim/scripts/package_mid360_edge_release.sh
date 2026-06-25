#!/usr/bin/env bash
set -euo pipefail

WS=${1:-/home/accelerate/cuadc_ws}
STAMP=$(date +%Y%m%d_%H%M%S)
PKG=mid360_reliable_mapping_edge_${STAMP}
RELEASE_DIR=${WS}/releases
STAGING=$(mktemp -d)
OUT=${RELEASE_DIR}/${PKG}.tar.gz

cleanup() {
  if [ -n ${STAGING:-} ] && [ -d ${STAGING} ]; then
    case ${STAGING} in
      /tmp/*) rm -rf ${STAGING} ;;
    esac
  fi
}
trap cleanup EXIT

mkdir -p ${RELEASE_DIR}
mkdir -p ${STAGING}/${PKG}/src

copy_dir() {
  NAME=$1
  SRC=${WS}/src/${NAME}
  DST=${STAGING}/${PKG}/src/${NAME}
  if [ ! -d ${SRC} ]; then
    echo missing source dir: ${SRC}
    exit 1
  fi
  cp -a ${SRC} ${DST}
}

copy_dir uav_slam_sim
copy_dir FAST_LIO_ROS2
copy_dir livox_ros_driver2_real

find ${STAGING}/${PKG} -name .git -type d -prune -exec rm -rf {} +
find ${STAGING}/${PKG} -name __pycache__ -type d -prune -exec rm -rf {} +
find ${STAGING}/${PKG} -name build -type d -prune -exec rm -rf {} +
find ${STAGING}/${PKG} -name install -type d -prune -exec rm -rf {} +
find ${STAGING}/${PKG} -name log -type d -prune -exec rm -rf {} +
find ${STAGING}/${PKG} -name .colcon -type d -prune -exec rm -rf {} +
find ${STAGING}/${PKG} -name '*.db3' -type f -delete
find ${STAGING}/${PKG} -name '*.bag' -type f -delete
find ${STAGING}/${PKG} -name '*.pcd' -type f -delete
find ${STAGING}/${PKG} -name '*.pyc' -type f -delete

cat > ${STAGING}/${PKG}/EDGE_DEPLOYMENT_README.md <<EOF
# MID360 Reliable Mapping Edge Release

This package contains the source code needed to deploy the MID360 reliable mapping pipeline on an edge computer such as NVIDIA Jetson Orin Nano.

Main packages:

- src/uav_slam_sim
- src/FAST_LIO_ROS2
- src/livox_ros_driver2_real

Recommended build:

cd ~/cuadc_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

Headless run:

ros2 launch uav_slam_sim real_mid360_dual_view.launch.py start_dual_rviz:=false

Full Chinese documentation:

src/uav_slam_sim/README.md
src/uav_slam_sim/docs/archive_mid360_reliable_mapping.md
src/uav_slam_sim/docs/orin_nano_edge_deployment.md
EOF

cat > ${STAGING}/${PKG}/release_manifest.txt <<EOF
release_name=${PKG}
created_at=${STAMP}
workspace=${WS}
contains=src/uav_slam_sim
contains=src/FAST_LIO_ROS2
contains=src/livox_ros_driver2_real
ros_distro=humble
lidar=MID360
lidar_ip=192.168.1.141
recommended_host_ip=192.168.1.50
main_launch=ros2 launch uav_slam_sim real_mid360_dual_view.launch.py
headless_launch=ros2 launch uav_slam_sim real_mid360_dual_view.launch.py start_dual_rviz:=false
EOF

cd ${STAGING}
tar -czf ${OUT} ${PKG}

echo created release archive: ${OUT}
echo archive contents root: ${PKG}
