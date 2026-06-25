#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudCapture(Node):
    def __init__(self, topic):
        super().__init__("mid360_snapshot_capture")
        self.message = None
        self.subscription = self.create_subscription(
            PointCloud2,
            topic,
            self._callback,
            10,
        )

    def _callback(self, msg):
        if msg.width * msg.height == 0:
            return
        self.message = msg


def equalize_axes(ax, points):
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    centers = (mins + maxs) * 0.5
    radius = np.max(maxs - mins) * 0.55
    radius = max(radius, 1.0)
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(max(centers[2] - radius, -1.0), centers[2] + radius)


def render(points, output):
    if points.shape[0] > 80000:
        rng = np.random.default_rng(7)
        points = points[rng.choice(points.shape[0], 80000, replace=False)]

    if points.shape[0] > 200:
        lo = np.percentile(points[:, :3], 1, axis=0)
        hi = np.percentile(points[:, :3], 99, axis=0)
        mask = np.all((points[:, :3] >= lo) & (points[:, :3] <= hi), axis=1)
        if np.count_nonzero(mask) > 100:
            points = points[mask]

    distances = np.linalg.norm(points[:, :3], axis=1)
    fig = plt.figure(figsize=(14, 6), dpi=140)

    ax_top = fig.add_subplot(1, 2, 1)
    ax_top.scatter(points[:, 0], points[:, 1], c=distances, s=0.35, cmap="turbo")
    ax_top.set_title("MID360 point cloud - top view")
    ax_top.set_xlabel("x (m)")
    ax_top.set_ylabel("y (m)")
    ax_top.set_aspect("equal", adjustable="box")
    ax_top.grid(True, alpha=0.25)
    xy_min = np.percentile(points[:, :2], 1, axis=0)
    xy_max = np.percentile(points[:, :2], 99, axis=0)
    xy_center = (xy_min + xy_max) * 0.5
    xy_radius = max(np.max(xy_max - xy_min) * 0.55, 1.0)
    ax_top.set_xlim(xy_center[0] - xy_radius, xy_center[0] + xy_radius)
    ax_top.set_ylim(xy_center[1] - xy_radius, xy_center[1] + xy_radius)

    ax_3d = fig.add_subplot(1, 2, 2, projection="3d")
    ax_3d.scatter(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2], s=0.35, cmap="viridis")
    ax_3d.set_title("MID360 point cloud - 3D")
    ax_3d.set_xlabel("x (m)")
    ax_3d.set_ylabel("y (m)")
    ax_3d.set_zlabel("z (m)")
    ax_3d.view_init(elev=28, azim=-135)
    equalize_axes(ax_3d, points[:, :3])

    fig.tight_layout()
    fig.savefig(output)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/mid360/points")
    parser.add_argument("--output", default="/home/accelerate/cuadc_ws/mid360_points_snapshot.png")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = PointCloudCapture(args.topic)
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    while rclpy.ok() and node.message is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds > deadline:
            raise TimeoutError(f"No PointCloud2 received from {args.topic}")

    msg = node.message
    points = []
    for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        if all(math.isfinite(v) for v in (x, y, z)):
            points.append((x, y, z))

    if not points:
        raise RuntimeError("Received PointCloud2, but it contained no finite xyz points")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    render(np.asarray(points, dtype=np.float32), output)
    print(f"saved {len(points)} points to {output}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
