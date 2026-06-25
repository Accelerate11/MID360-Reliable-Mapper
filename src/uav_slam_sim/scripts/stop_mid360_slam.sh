#!/usr/bin/env bash
set -euo pipefail

pkill -f rviz2 2>/dev/null || true
pkill -f "[f]astlio_mapping" 2>/dev/null || true
pkill -f "[s]ynthetic_mid360_publisher.py" 2>/dev/null || true
pkill -f "[w]eb_mid360_icp_mapping_viewer.py" 2>/dev/null || true
pkill -f "[l]ivox_ros_driver2_node" 2>/dev/null || true
pkill -f "[l]ivox_custom_frame_aggregator_node" 2>/dev/null || true
pkill -f "[p]ointcloud_occupancy_grid_node" 2>/dev/null || true
pkill -f "[r]os2 launch uav_slam_sim" 2>/dev/null || true
