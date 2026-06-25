#!/usr/bin/env python3
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class TopicStats:
    def __init__(self):
        self.count = 0
        self.nonzero_count = 0
        self.total_points = 0
        self.min_points = None
        self.max_points = 0
        self.first_time = None
        self.last_time = None
        self.last_points = 0
        self.last_frame = ""
        self.bounds = None

    def update(self, msg):
        now = time.monotonic()
        if self.first_time is None:
            self.first_time = now
        self.last_time = now
        self.count += 1
        self.last_frame = msg.header.frame_id

        xs = []
        ys = []
        zs = []
        points = 0
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue
            xs.append(x)
            ys.append(y)
            zs.append(z)
            points += 1
        self.last_points = points
        self.total_points += points
        self.min_points = points if self.min_points is None else min(self.min_points, points)
        self.max_points = max(self.max_points, points)
        if points > 0:
            self.nonzero_count += 1
        if points:
            self.bounds = {
                "x": [min(xs), max(xs)],
                "y": [min(ys), max(ys)],
                "z": [min(zs), max(zs)],
            }

    def to_dict(self):
        hz = 0.0
        if self.first_time is not None and self.last_time is not None and self.last_time > self.first_time:
            hz = max(0, self.count - 1) / (self.last_time - self.first_time)
        return {
            "messages": self.count,
            "nonzero_messages": self.nonzero_count,
            "hz_estimate": hz,
            "last_points": self.last_points,
            "avg_points": self.total_points / self.count if self.count else 0.0,
            "min_points": self.min_points if self.min_points is not None else 0,
            "max_points": self.max_points,
            "frame_id": self.last_frame,
            "bounds": self.bounds,
        }


class QualityNode(Node):
    def __init__(self, topics):
        super().__init__("mid360_mapping_quality_checker")
        self.stats = {topic: TopicStats() for topic in topics}
        self._cloud_subscriptions = [
            self.create_subscription(PointCloud2, topic, self.make_callback(topic), 5)
            for topic in topics
        ]

    def make_callback(self, topic):
        def callback(msg):
            self.stats[topic].update(msg)
        return callback


def main():
    parser = argparse.ArgumentParser(description="Check MID360 FAST-LIO2 mapping topic quality.")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument(
        "--topics",
        nargs="+",
        default=["/cloud_registered", "/cloud_registered_filtered", "/fastlio_denoised_map"],
    )
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = QualityNode(args.topics)
    deadline = time.monotonic() + max(0.5, args.duration)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        result = {topic: stats.to_dict() for topic, stats in node.stats.items()}
        if args.json:
            print(json.dumps(result, indent=2))
        else:
            for topic, stats in result.items():
                print(f"topic: {topic}")
                print(f"  messages: {stats['messages']}")
                print(f"  nonzero_messages: {stats['nonzero_messages']}")
                print(f"  hz: {stats['hz_estimate']:.2f}")
                print(f"  last_points: {stats['last_points']}")
                print(f"  avg_points: {stats['avg_points']:.1f}")
                print(f"  min_points: {stats['min_points']}")
                print(f"  max_points: {stats['max_points']}")
                print(f"  frame_id: {stats['frame_id']}")
                print(f"  bounds: {stats['bounds']}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
