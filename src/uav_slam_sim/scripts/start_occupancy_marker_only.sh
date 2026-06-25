#!/usr/bin/env bash
set -eo pipefail
cd /home/accelerate/cuadc_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
exec ros2 run uav_slam_sim pointcloud_occupancy_grid_node --ros-args \
  -r __node:=fastlio_occupancy_grid \
  -p input_topic:=/cloud_registered_filtered \
  -p odom_topic:=/Odometry \
  -p grid_topic:=/fastlio_occupancy_grid \
  -p marker_topic:=/fastlio_occupancy_cells \
  -p output_frame:=camera_init \
  -p resolution:=0.10 \
  -p width_m:=30.0 \
  -p height_m:=30.0 \
  -p origin_x:=-15.0 \
  -p origin_y:=-15.0 \
  -p z_min:=-0.30 \
  -p z_max:=2.50