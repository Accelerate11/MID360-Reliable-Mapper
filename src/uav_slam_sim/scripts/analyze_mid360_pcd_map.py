#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def read_ascii_pcd(path):
    fields = []
    data_start = None
    header = {}
    with open(path, "r", encoding="ascii", errors="replace") as handle:
        lines = handle.readlines()

    for index, line in enumerate(lines):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        key = parts[0].upper()
        if key == "FIELDS":
            fields = parts[1:]
        elif key in {"WIDTH", "HEIGHT", "POINTS"} and len(parts) > 1:
            header[key.lower()] = int(parts[1])
        elif key == "DATA":
            if len(parts) < 2 or parts[1].lower() != "ascii":
                raise ValueError("Only ascii PCD files are supported.")
            data_start = index + 1
            break

    if data_start is None:
        raise ValueError(f"No DATA ascii section found in {path}")
    if not fields:
        raise ValueError(f"No FIELDS header found in {path}")

    required = ["x", "y", "z"]
    indices = []
    for field in required:
        if field not in fields:
            raise ValueError(f"PCD missing field: {field}")
        indices.append(fields.index(field))

    intensity_index = fields.index("intensity") if "intensity" in fields else None
    points = []
    for line in lines[data_start:]:
        stripped = line.strip()
        if not stripped:
            continue
        values = stripped.split()
        if len(values) < len(fields):
            continue
        x = float(values[indices[0]])
        y = float(values[indices[1]])
        z = float(values[indices[2]])
        intensity = float(values[intensity_index]) if intensity_index is not None else 0.0
        if all(math.isfinite(v) for v in (x, y, z)):
            points.append((x, y, z, intensity))

    return np.asarray(points, dtype=np.float32), header


def voxel_stats(points, voxel):
    if points.size == 0 or voxel <= 0:
        return {"voxel_size": voxel, "occupied_voxels": 0, "points_per_voxel": 0.0}
    keys = np.floor(points[:, :3] / voxel).astype(np.int64)
    occupied = len({tuple(row) for row in keys})
    return {
        "voxel_size": voxel,
        "occupied_voxels": occupied,
        "points_per_voxel": float(points.shape[0]) / float(max(occupied, 1)),
    }


def render_snapshot(points, output, max_points):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)

    draw = points
    if draw.shape[0] > max_points:
        rng = np.random.default_rng(11)
        draw = draw[rng.choice(draw.shape[0], max_points, replace=False)]

    if draw.shape[0] > 300:
        lo = np.percentile(draw[:, :3], 1, axis=0)
        hi = np.percentile(draw[:, :3], 99, axis=0)
        mask = np.all((draw[:, :3] >= lo) & (draw[:, :3] <= hi), axis=1)
        if np.count_nonzero(mask) > 100:
            draw = draw[mask]

    distances = np.linalg.norm(draw[:, :3], axis=1)
    fig = plt.figure(figsize=(14, 6), dpi=140)

    ax_top = fig.add_subplot(1, 2, 1)
    ax_top.scatter(draw[:, 0], draw[:, 1], c=distances, s=0.45, cmap="turbo")
    ax_top.set_title("MID360 map - top view")
    ax_top.set_xlabel("x (m)")
    ax_top.set_ylabel("y (m)")
    ax_top.set_aspect("equal", adjustable="box")
    ax_top.grid(True, alpha=0.25)

    xy_min = np.percentile(draw[:, :2], 1, axis=0)
    xy_max = np.percentile(draw[:, :2], 99, axis=0)
    xy_center = (xy_min + xy_max) * 0.5
    xy_radius = max(np.max(xy_max - xy_min) * 0.55, 1.0)
    ax_top.set_xlim(xy_center[0] - xy_radius, xy_center[0] + xy_radius)
    ax_top.set_ylim(xy_center[1] - xy_radius, xy_center[1] + xy_radius)

    ax_3d = fig.add_subplot(1, 2, 2, projection="3d")
    ax_3d.scatter(draw[:, 0], draw[:, 1], draw[:, 2], c=draw[:, 2], s=0.45, cmap="viridis")
    ax_3d.set_title("MID360 map - 3D")
    ax_3d.set_xlabel("x (m)")
    ax_3d.set_ylabel("y (m)")
    ax_3d.set_zlabel("z (m)")
    ax_3d.view_init(elev=28, azim=-135)

    mins = draw[:, :3].min(axis=0)
    maxs = draw[:, :3].max(axis=0)
    centers = (mins + maxs) * 0.5
    radius = max(np.max(maxs - mins) * 0.55, 1.0)
    ax_3d.set_xlim(centers[0] - radius, centers[0] + radius)
    ax_3d.set_ylim(centers[1] - radius, centers[1] + radius)
    ax_3d.set_zlim(centers[2] - radius, centers[2] + radius)

    fig.tight_layout()
    fig.savefig(output)
    plt.close(fig)


def analyze(points, pcd_path, voxel_sizes):
    report = {
        "pcd": str(Path(pcd_path).resolve()),
        "points": int(points.shape[0]),
    }
    if points.shape[0] == 0:
        return report

    xyz = points[:, :3]
    mins = xyz.min(axis=0)
    maxs = xyz.max(axis=0)
    spans = maxs - mins
    centroid = xyz.mean(axis=0)
    report.update({
        "bounds": {
            "x": [float(mins[0]), float(maxs[0])],
            "y": [float(mins[1]), float(maxs[1])],
            "z": [float(mins[2]), float(maxs[2])],
        },
        "span_m": {
            "x": float(spans[0]),
            "y": float(spans[1]),
            "z": float(spans[2]),
        },
        "centroid": [float(v) for v in centroid],
        "z_percentiles": {
            "p01": float(np.percentile(xyz[:, 2], 1)),
            "p50": float(np.percentile(xyz[:, 2], 50)),
            "p99": float(np.percentile(xyz[:, 2], 99)),
        },
        "voxel_stats": [voxel_stats(points, voxel) for voxel in voxel_sizes],
    })
    return report


def main():
    parser = argparse.ArgumentParser(description="Analyze a MID360 ascii PCD map.")
    parser.add_argument("pcd")
    parser.add_argument("--report", default="")
    parser.add_argument("--snapshot", default="")
    parser.add_argument("--snapshot-max-points", type=int, default=100000)
    parser.add_argument("--voxel-sizes", type=float, nargs="+", default=[0.05, 0.10, 0.20, 0.50])
    args = parser.parse_args()

    pcd_path = Path(args.pcd)
    points, _ = read_ascii_pcd(pcd_path)
    report = analyze(points, pcd_path, args.voxel_sizes)

    report_path = Path(args.report) if args.report else pcd_path.with_suffix(".analysis.json")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    if args.snapshot:
        if points.shape[0] == 0:
            raise RuntimeError("Cannot render an empty PCD map.")
        render_snapshot(points, args.snapshot, args.snapshot_max_points)
        report["snapshot"] = str(Path(args.snapshot).resolve())
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(f"Saved analysis: {report_path}")
    print(f"points={report.get('points', 0)}")
    if "span_m" in report:
        print(f"span_m={report['span_m']}")
    if args.snapshot:
        print(f"snapshot={args.snapshot}")


if __name__ == "__main__":
    main()
