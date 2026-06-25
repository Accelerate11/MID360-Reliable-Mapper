#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from livox_ros_driver2.msg import CustomMsg


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RealMid360Probe(Node):
    def __init__(self):
        super().__init__("real_mid360_probe")
        self.start_time = time.monotonic()
        self.seen_lidar = False
        self.seen_lidar_frame = False
        self.seen_imu = False
        self.seen_imu_frame = False
        self.seen_cloud = False
        self.seen_map = False

        self.create_subscription(CustomMsg, "/livox/lidar", self.lidar_cb, 10)
        self.create_subscription(CustomMsg, "/livox/lidar_frame", self.lidar_frame_cb, 10)
        self.create_subscription(Imu, "/livox/imu", self.imu_cb, 10)
        self.create_subscription(Imu, "/livox/imu_frame", self.imu_frame_cb, 10)
        self.create_subscription(PointCloud2, "/cloud_registered", self.cloud_cb, 10)
        self.create_subscription(PointCloud2, "/Laser_map", self.map_cb, 10)

    def lidar_cb(self, msg):
        if self.seen_lidar:
            return
        self.seen_lidar = True
        self.print_lidar_stats("LIDAR_RAW", msg)

    def lidar_frame_cb(self, msg):
        if self.seen_lidar_frame:
            return
        self.seen_lidar_frame = True
        self.print_lidar_stats("LIDAR_FRAME", msg)

    def print_lidar_stats(self, label, msg):
        points = list(msg.points)
        tag_counts = {}
        line_counts = {}
        accepted = 0
        valid = 0
        last = [(0.0, 0.0, 0.0) for _ in points]

        for i, p in enumerate(points):
            tag_counts[p.tag] = tag_counts.get(p.tag, 0) + 1
            line_counts[p.line] = line_counts.get(p.line, 0) + 1
            if i == 0:
                continue
            if p.line < 4 and ((p.tag & 0x30) == 0x10 or (p.tag & 0x30) == 0x00):
                valid += 1
                x, y, z = float(p.x), float(p.y), float(p.z)
                prev = last[i - 1]
                last[i] = (x, y, z)
                moved = (
                    abs(x - prev[0]) > 1e-7
                    or abs(y - prev[1]) > 1e-7
                    or abs(z - prev[2]) > 1e-7
                )
                far = x * x + y * y + z * z > 0.5 * 0.5
                if moved and far:
                    accepted += 1

        first_points = []
        for p in points[:8]:
            dist = math.sqrt(float(p.x) ** 2 + float(p.y) ** 2 + float(p.z) ** 2)
            first_points.append(
                f"line={p.line},tag={p.tag},dist={dist:.3f},offset={p.offset_time}"
            )

        print(label, "header_sec", f"{stamp_to_sec(msg.header.stamp):.9f}")
        print(label, "point_num", msg.point_num, "len", len(points))
        print(label, "line_counts", sorted(line_counts.items()))
        print(label, "tag_counts", sorted(tag_counts.items()))
        print(label, "fastlio_like_valid", valid, "accepted", accepted)
        print(label, "first_points", "; ".join(first_points))

    def imu_cb(self, msg):
        if self.seen_imu:
            return
        self.seen_imu = True
        print("IMU_RAW header_sec", f"{stamp_to_sec(msg.header.stamp):.9f}")

    def imu_frame_cb(self, msg):
        if self.seen_imu_frame:
            return
        self.seen_imu_frame = True
        print("IMU_FRAME header_sec", f"{stamp_to_sec(msg.header.stamp):.9f}")

    def cloud_cb(self, msg):
        if self.seen_cloud:
            return
        self.seen_cloud = True
        print("CLOUD_REGISTERED width", msg.width, "height", msg.height, "stamp", f"{stamp_to_sec(msg.header.stamp):.9f}")

    def map_cb(self, msg):
        if self.seen_map:
            return
        self.seen_map = True
        print("LASER_MAP width", msg.width, "height", msg.height, "stamp", f"{stamp_to_sec(msg.header.stamp):.9f}")


def main():
    rclpy.init()
    node = RealMid360Probe()
    try:
        while time.monotonic() - node.start_time < 15.0:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
