#!/usr/bin/env bash
set -eo pipefail

WS=/home/accelerate/cuadc_ws

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== MID360 mapping pipeline diagnostic ==="
date
echo

echo "=== network ==="
ping -c 2 -W 1 192.168.1.141 || true
echo

echo "=== processes ==="
ps -eo pid,pcpu,pmem,comm,args | grep -E 'ros2 launch uav_slam_sim|livox_ros|fastlio_mapping|fastlio_cloud|rviz2' | grep -v grep || true
echo

echo "=== ROS topics ==="
ros2 topic list | sort | grep -E 'livox|cloud|fastlio|Odometry|path|Laser|tf' || true
echo

echo "=== raw lidar point_num ==="
timeout 6 python3 - <<'PY' || true
import rclpy
from livox_ros_driver2.msg import CustomMsg

rclpy.init()
node = rclpy.create_node("mid360_point_num_probe")
seen = []

def callback(msg):
    print(msg.point_num)
    seen.append(True)

node.create_subscription(CustomMsg, "/livox/lidar", callback, 10)
while rclpy.ok() and not seen:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
PY
echo

echo "=== topic rates ==="
for topic in /livox/lidar /livox/imu /cloud_registered /cloud_registered_filtered /fastlio_denoised_map; do
  echo "--- $topic ---"
  timeout 5 ros2 topic hz "$topic" || true
done
echo

echo "=== cloud quality ==="
"$WS/install/uav_slam_sim/share/uav_slam_sim/scripts/check_real_mid360_mapping_quality.sh" \
  --duration 3 || true
