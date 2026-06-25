#!/usr/bin/env python3
import argparse
import json
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CloudExporter(Node):
    def __init__(self, args):
        super().__init__("mid360_mapping_cloud_exporter")
        self.args = args
        self.latest_msg = None
        self.received = 0
        self.last_receive_time = None
        self.create_subscription(PointCloud2, args.topic, self.cloud_callback, 5)
        self.get_logger().info(
            f"Waiting for {args.topic}, duration={args.duration:.1f}s, "
            f"output={args.output}"
        )

    def cloud_callback(self, msg):
        self.latest_msg = msg
        self.received += 1
        self.last_receive_time = time.monotonic()

    def extract_points(self):
        if self.latest_msg is None:
            return []

        points = []
        min_range_sq = self.args.min_range * self.args.min_range
        max_range_sq = self.args.max_range * self.args.max_range if self.args.max_range > 0 else float("inf")
        stride = max(1, self.args.stride)

        for index, point in enumerate(
            point_cloud2.read_points(
                self.latest_msg,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True,
            )
        ):
            if index % stride != 0:
                continue
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            intensity = float(point[3]) if len(point) > 3 else 0.0
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue
            if z < self.args.z_min or z > self.args.z_max:
                continue
            range_sq = x * x + y * y + z * z
            if range_sq < min_range_sq or range_sq > max_range_sq:
                continue
            points.append((x, y, z, intensity))

        if self.args.max_points > 0 and len(points) > self.args.max_points:
            step = len(points) / float(self.args.max_points)
            points = [points[int(i * step)] for i in range(self.args.max_points)]

        return points


def write_ascii_pcd(path, points):
    with open(path, "w", encoding="ascii") as handle:
        handle.write("# .PCD v0.7 - Point Cloud Data file format\n")
        handle.write("VERSION 0.7\n")
        handle.write("FIELDS x y z intensity\n")
        handle.write("SIZE 4 4 4 4\n")
        handle.write("TYPE F F F F\n")
        handle.write("COUNT 1 1 1 1\n")
        handle.write(f"WIDTH {len(points)}\n")
        handle.write("HEIGHT 1\n")
        handle.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        handle.write(f"POINTS {len(points)}\n")
        handle.write("DATA ascii\n")
        for x, y, z, intensity in points:
            handle.write(f"{x:.6f} {y:.6f} {z:.6f} {intensity:.6f}\n")


def build_report(points, topic, frame_id, messages_received):
    report = {
        "topic": topic,
        "frame_id": frame_id,
        "messages_received": messages_received,
        "points": len(points),
        "created_unix_time": time.time(),
    }
    if not points:
        return report

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    report.update({
        "bounds": {
            "x": [min(xs), max(xs)],
            "y": [min(ys), max(ys)],
            "z": [min(zs), max(zs)],
        },
        "span_m": {
            "x": max(xs) - min(xs),
            "y": max(ys) - min(ys),
            "z": max(zs) - min(zs),
        },
        "centroid": [
            sum(xs) / len(points),
            sum(ys) / len(points),
            sum(zs) / len(points),
        ],
    })
    return report


def main():
    parser = argparse.ArgumentParser(description="Export a MID360 FAST-LIO2 PointCloud2 map to PCD.")
    parser.add_argument("--topic", default="/fastlio_denoised_map")
    parser.add_argument("--output", default="/home/accelerate/cuadc_ws/mid360_mapping_outputs/latest_map.pcd")
    parser.add_argument("--report", default="")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--idle-timeout", type=float, default=0.0)
    parser.add_argument("--min-range", type=float, default=0.0)
    parser.add_argument("--max-range", type=float, default=0.0)
    parser.add_argument("--z-min", type=float, default=-5.0)
    parser.add_argument("--z-max", type=float, default=8.0)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-points", type=int, default=0)
    args = parser.parse_args()

    rclpy.init()
    node = CloudExporter(args)
    deadline = time.monotonic() + max(0.1, args.duration)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                args.idle_timeout > 0.0 and
                node.received > 0 and
                node.last_receive_time is not None and
                time.monotonic() - node.last_receive_time >= args.idle_timeout
            ):
                break

        points = node.extract_points()
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        write_ascii_pcd(output, points)

        report_path = Path(args.report) if args.report else output.with_suffix(".json")
        report = build_report(
            points,
            args.topic,
            node.latest_msg.header.frame_id if node.latest_msg else "",
            node.received,
        )
        with open(report_path, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2)
            handle.write("\n")

        print(f"Saved PCD: {output}")
        print(f"Saved report: {report_path}")
        print(f"points={len(points)} messages={node.received}")
        if points:
            print(f"bounds={report['bounds']}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
