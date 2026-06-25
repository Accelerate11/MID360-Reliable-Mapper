#!/usr/bin/env python3
import argparse
import math
import random

import rclpy
from rclpy.node import Node

from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import Imu


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SyntheticMid360Publisher(Node):
    def __init__(self, args):
        super().__init__("synthetic_mid360_publisher")
        self.args = args
        self.lidar_pub = self.create_publisher(CustomMsg, args.lidar_topic, 10)
        self.imu_pub = self.create_publisher(Imu, args.imu_topic, 100)
        self.scene = self.build_scene()
        self.start_time_ns = self.get_clock().now().nanoseconds
        self.scan_index = 0
        self.lidar_period_ns = int(1_000_000_000 / args.lidar_rate)
        self.imu_timer = self.create_timer(1.0 / args.imu_rate, self.publish_imu)
        self.lidar_timer = self.create_timer(1.0 / args.lidar_rate, self.publish_lidar)
        self.get_logger().info(
            f"Publishing synthetic MID360 data: {args.lidar_topic}, {args.imu_topic}, "
            f"scene_points={len(self.scene)}"
        )

    def elapsed(self, stamp=None):
        if stamp is None:
            now_ns = self.get_clock().now().nanoseconds
        else:
            now_ns = stamp_to_ns(stamp)
        return max(0.0, (now_ns - self.start_time_ns) * 1.0e-9)

    def pose_at(self, t):
        mt = max(0.0, t - self.args.warmup)
        ax = self.args.motion_x
        ay = self.args.motion_y
        az = self.args.motion_z
        wx = 0.18
        wy = 0.13
        wz = 0.11
        wyaw = 0.16

        x = ax * math.sin(wx * mt) ** 2
        y = ay * math.sin(wy * mt) ** 2
        z = self.args.sensor_height + az * math.sin(wz * mt) ** 2
        yaw = self.args.motion_yaw * math.sin(wyaw * mt) ** 2

        vx = ax * wx * math.sin(2.0 * wx * mt)
        vy = ay * wy * math.sin(2.0 * wy * mt)
        vz = az * wz * math.sin(2.0 * wz * mt)
        yaw_rate = self.args.motion_yaw * wyaw * math.sin(2.0 * wyaw * mt)

        acc_x = 2.0 * ax * wx * wx * math.cos(2.0 * wx * mt)
        acc_y = 2.0 * ay * wy * wy * math.cos(2.0 * wy * mt)
        acc_z = 2.0 * az * wz * wz * math.cos(2.0 * wz * mt)
        if t < self.args.warmup:
            vx = vy = vz = 0.0
            yaw_rate = 0.0
            acc_x = acc_y = acc_z = 0.0
        return (x, y, z, yaw), (vx, vy, vz), (acc_x, acc_y, acc_z), yaw_rate

    def world_to_lidar(self, point, pose):
        x, y, z, yaw = pose
        dx = point[0] - x
        dy = point[1] - y
        dz = point[2] - z
        c = math.cos(yaw)
        s = math.sin(yaw)
        return c * dx + s * dy, -s * dx + c * dy, dz

    def build_scene(self):
        points = []
        size = self.args.room_size * 0.5
        z_min = 0.0
        z_max = 2.8
        step = self.args.scene_step

        def add_point(x, y, z, intensity=90):
            points.append((x, y, z, intensity))

        value = -size
        while value <= size:
            z = z_min
            while z <= z_max:
                add_point(-size, value, z, 80)
                add_point(size, value, z, 110)
                add_point(value, -size, z, 130)
                add_point(value, size, z, 160)
                z += step
            value += step

        floor_x = -size
        while floor_x <= size:
            floor_y = -size
            while floor_y <= size:
                add_point(floor_x, floor_y, z_min, 55)
                floor_y += step * 2.0
            floor_x += step * 2.0

        obstacles = [(-1.7, -1.2), (1.4, -1.6), (-1.0, 1.6), (1.7, 1.2)]
        for cx, cy in obstacles:
            x = cx - 0.5
            while x <= cx + 0.5:
                z = z_min
                while z <= 1.5:
                    add_point(x, cy - 0.5, z, 210)
                    add_point(x, cy + 0.5, z, 210)
                    z += step
                x += step
            y = cy - 0.5
            while y <= cy + 0.5:
                z = z_min
                while z <= 1.5:
                    add_point(cx - 0.5, y, z, 230)
                    add_point(cx + 0.5, y, z, 230)
                    z += step
                y += step

        random.Random(7).shuffle(points)
        return points

    def publish_imu(self):
        stamp = self.get_clock().now().to_msg()
        t = self.elapsed(stamp)
        pose, _, acc_world, yaw_rate = self.pose_at(t)
        yaw = pose[3]
        c = math.cos(yaw)
        s = math.sin(yaw)
        world_ax = acc_world[0]
        world_ay = acc_world[1]
        world_az = acc_world[2] + 9.80665
        body_ax = c * world_ax + s * world_ay
        body_ay = -s * world_ax + c * world_ay

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "livox_frame"
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = yaw_rate
        msg.linear_acceleration.x = body_ax
        msg.linear_acceleration.y = body_ay
        msg.linear_acceleration.z = world_az
        msg.angular_velocity_covariance[0] = 0.0001
        msg.angular_velocity_covariance[4] = 0.0001
        msg.angular_velocity_covariance[8] = 0.0001
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01
        self.imu_pub.publish(msg)

    def publish_lidar(self):
        stamp = self.get_clock().now().to_msg()
        t = self.elapsed(stamp)
        pose, _, _, _ = self.pose_at(t)
        msg = CustomMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = "livox_frame"
        msg.timebase = stamp_to_ns(stamp)
        msg.lidar_id = 0

        visible = []
        min_range_sq = self.args.min_range * self.args.min_range
        max_range_sq = self.args.max_range * self.args.max_range
        for world_point in self.scene:
            lx, ly, lz = self.world_to_lidar(world_point, pose)
            range_sq = lx * lx + ly * ly + lz * lz
            if range_sq < min_range_sq or range_sq > max_range_sq:
                continue
            if lz < self.args.z_min or lz > self.args.z_max:
                continue
            visible.append((lx, ly, lz, world_point[3]))

        if not visible:
            self.get_logger().warn("Synthetic scene has no visible points for current pose.")
            return

        count = min(self.args.points_per_scan, len(visible))
        start = (self.scan_index * 173) % len(self.scene)
        selected = [visible[(start + i) % len(visible)] for i in range(count)]
        selected.sort(key=lambda item: math.atan2(item[1], item[0]))

        msg.points = []
        denom = max(1, count - 1)
        for i, (x, y, z, intensity) in enumerate(selected):
            point = CustomPoint()
            point.offset_time = int(i * (self.lidar_period_ns - 1_000_000) / denom)
            point.x = float(x)
            point.y = float(y)
            point.z = float(z)
            point.reflectivity = int(max(0, min(255, intensity)))
            point.tag = 0
            point.line = int(i % 4)
            msg.points.append(point)

        msg.point_num = len(msg.points)
        self.scan_index += 1
        self.lidar_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--lidar-topic", default="/synthetic/livox/lidar")
    parser.add_argument("--imu-topic", default="/synthetic/livox/imu")
    parser.add_argument("--lidar-rate", type=float, default=10.0)
    parser.add_argument("--imu-rate", type=float, default=200.0)
    parser.add_argument("--points-per-scan", type=int, default=4500)
    parser.add_argument("--room-size", type=float, default=8.0)
    parser.add_argument("--scene-step", type=float, default=0.16)
    parser.add_argument("--min-range", type=float, default=0.5)
    parser.add_argument("--max-range", type=float, default=12.0)
    parser.add_argument("--z-min", type=float, default=-2.0)
    parser.add_argument("--z-max", type=float, default=2.5)
    parser.add_argument("--sensor-height", type=float, default=1.0)
    parser.add_argument("--warmup", type=float, default=3.0)
    parser.add_argument("--motion-x", type=float, default=0.45)
    parser.add_argument("--motion-y", type=float, default=0.35)
    parser.add_argument("--motion-z", type=float, default=0.04)
    parser.add_argument("--motion-yaw", type=float, default=0.15)
    args = parser.parse_args()

    rclpy.init()
    node = SyntheticMid360Publisher(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
